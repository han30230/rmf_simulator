# RMF–VDA5050 통로 제어 실차 적용 준비 설계

## 목적

현재 PoC의 통로 제어 기능을 실제 VDA5050 로봇에 연결하기 전에, 시뮬레이션 전용 가정과 현장 필수 조건을 명확히 분리한다. 이 설계는 RMF Core Planner를 수정하지 않는다. 기존 VDA5050 Fleet Adapter가 작업을 로봇 Order로 변환하는 구조와, `traffic_control` 계층이 통로 진입 권한을 결정하는 구조를 유지한다.

소프트웨어는 통로 교통을 조정한다. 로봇의 안전 PLC, 범퍼, 레이저 스캐너, 비상정지 및 현장 안전 인증을 대체하지 않는다.

## 현재 기준선

- `traffic_control`은 Block, Holding Bay, 방향 batch, Passing Bay, 다중 Block Movement Authority를 관리한다.
- Fleet Adapter는 VDA5050 `state`, `connection`, `order`, `instantActions`를 사용한다.
- Arbiter의 MQTT monitor는 현재 `state`만 구독하고 위치 freshness 위주로 fail-closed 처리한다.
- Fleet Adapter와 Arbiter 모두 username/password/TLS를 구성하는 공통된 production 설정이 없다.
- 실행 스크립트는 Simulator를 자동 실행하고 개발용 RMF API JWT를 컨테이너 안에서 생성한다.
- 연결 통로 맵과 좌표 보정값은 시뮬레이션 데이터다.
- 작업, 예약, fault 상태는 메모리에 있으며 Arbiter 재시작 후 자동 복원되지 않는다.

## 범위 분할

실차 준비를 두 단계로 나눈다.

### 1단계: 배포 전 검증과 운행 가능 상태

이번 구현 범위다.

- simulation/production 프로필 분리
- production 설정 preflight validator
- MQTT password/TLS/mTLS 설정과 secret 주입
- VDA5050 connection 및 state 기반 robot eligibility
- `/health`와 `/ready` 상태
- Simulator를 실행하지 않는 production launcher
- 장애 주입 시뮬레이션과 회귀 테스트

### 2단계: 현장 복구와 운영 통제

1단계가 검증된 후 별도 설계와 현장 협의로 구현한다.

- 영속 저장소 및 재시작 reconciliation
- 인증된 운영자 recovery/acknowledge API
- 실제 WMS/MES 권한과 API 인증
- 이중화, 모니터링, 감사 로그와 알람 연동

1단계에서는 Arbiter 기동 직후 production readiness를 false로 유지하고 새 작업을 거부한다. 모든 필수 로봇이 fresh telemetry로 알려진 SafeStop에 정지해 있고 모든 관리 Block이 비어 있음이 확인된 clean-start 상태에서만 readiness를 자동으로 회복한다. 한 대라도 Block 내부, 위치 불명확 또는 offline이면 recovery required 상태를 유지한다. 불완전한 상태를 추측해 자동 복구하지 않는다.

## 구성 구조

### 배포 프로필

`deployment.mode`는 `simulation` 또는 `production`이다. 기존 설정은 명시적으로 simulation 프로필로 유지해 현재 시나리오를 깨뜨리지 않는다.

production 프로필은 다음 값을 요구한다.

- 실제 내비게이션 맵이며 `simulation_only: true`가 아님
- 로봇 roster의 manufacturer, serial number, 허용 map ID
- MQTT broker와 인증 방식
- 최소 3개의 서로 일직선이 아닌 RMF↔robot 기준점
- 실제 footprint, vicinity, 속도, 가속도 및 제동 관련 값
- telemetry와 connection timeout
- Holding Bay, Side Bay, 종점과 Block 경계
- 실제 RMF API URL과 외부에서 주입된 bearer token

`REPLACE_ME`, 빈 secret 참조, 시뮬레이션 전용 표식, 부족한 기준점, 중복 robot identity가 있으면 preflight가 실패한다. production launcher는 validator 성공 전 어떤 작업 서비스도 시작하지 않는다.

### Secret 처리

저장소에는 secret 값 대신 환경변수 이름 또는 파일 경로만 둔다. 지원 모드는 다음과 같다.

- username/password
- TLS server verification: CA file, server hostname verification
- mTLS: client certificate와 private key

production에서 TLS verification 비활성화는 허용하지 않는다. 로그, status, 예외 메시지는 password, token, private-key 경로의 민감한 값을 출력하지 않는다.

Fleet Adapter와 Arbiter MQTT monitor는 같은 연결 의미를 사용하지만 각 프로세스가 독립 MQTT client를 유지한다. 공통 설정 파서와 validator를 사용해 서로 다른 broker/security 설정으로 실행되는 실수를 막는다.

## 로봇 운행 가능 상태

`RobotTelemetry`는 위치뿐 아니라 다음 상태를 보관한다.

- 마지막 state 수신 시각
- 마지막 connection 수신 시각과 `connectionState`
- `agvPosition.positionInitialized`
- 현재 `mapId`
- `operatingMode`
- `safetyState.eStop` 및 field violation
- `paused`, `driving`
- 현재 VDA5050 errors 중 정책상 차단하는 error
- geometry로 판정한 Holding Bay 또는 Block

별도의 `RobotEligibilityPolicy`가 telemetry를 평가하고 `eligible`과 구조화된 reason 목록을 반환한다. 기본 production 조건은 다음과 같다.

1. 등록된 로봇이다.
2. connection이 ONLINE이며 신선하다.
3. state가 신선하고 position이 initialized 상태다.
4. map ID가 robot 설정의 허용 목록과 일치한다.
5. operating mode가 AUTOMATIC이다.
6. E-stop이 NONE이고 field violation이 없다.
7. paused가 아니며 차단 등급 error가 없다.
8. 위치가 알려진 SafeStop 또는 현재 권한과 일치하는 관리 Block이다.

새 작업 제출과 새로운 corridor authority 발급 전에 eligibility를 다시 평가한다. 실패 시 작업은 RMF에 전달되지 않으며 `robot_not_eligible`과 구체적인 reason을 반환한다.

권한 없이 통로 안에 나타난 로봇, 통로 내부에서 offline/E-stop/error가 된 로봇, 위치가 불명확한 로봇은 기존 fault 경로를 통해 관련 Block을 잠근다. SafeStop에서 offline인 로봇은 새 진입만 막고 실제 점유하지 않은 통로를 임의로 fault 처리하지 않는다.

## 데이터 흐름

1. Fleet Adapter와 Arbiter monitor가 같은 broker에 각자 연결한다.
2. Arbiter monitor가 등록된 로봇의 `state`와 `connection`을 구독한다.
3. Tracker가 메시지 header, identity, freshness와 payload를 검증한다.
4. Geometry tracker가 현재 SafeStop/Block을 판정한다.
5. Eligibility policy가 통신·안전·모드·위치 상태를 합성한다.
6. Task Gate가 작업 제출 및 각 rolling authority 발급 전에 eligibility를 확인한다.
7. 허용된 구간 작업만 RMF API로 전달된다.
8. telemetry가 release node 또는 SafeStop 도착을 증명하면 기존 방식으로 Block을 해제한다.

순서가 뒤바뀐 state는 VDA5050 header timestamp/headerId와 수신 순서를 이용해 무시한다. 잘못된 JSON, identity 불일치, 미등록 robot topic은 상태를 갱신하지 않고 reason이 있는 진단 이벤트를 남긴다.

## 상태 API

### `/health`

프로세스가 응답하고 내부 runner가 살아 있는지만 나타낸다. broker나 RMF 장애가 있어도 프로세스 자체가 살아 있으면 health는 응답하되 dependency 상태를 함께 표시한다.

### `/ready`

새 작업을 안전하게 받을 수 있을 때만 ready다. production readiness 조건은 다음과 같다.

- preflight 통과
- MQTT 연결됨
- RMF API 접근 가능
- 모든 필수 로봇의 connection/state가 수신됨
- 모든 필수 로봇의 위치가 알려진 상태
- 모든 로봇이 SafeStop에 있는 clean-start 검증을 통과해 recovery required가 해제됨

기존 `/traffic/status`에는 robot별 eligibility와 reason, deployment mode, readiness를 추가한다. secret은 포함하지 않는다.

## Production launcher

새 production launcher는 Simulator와 Visualizer를 시작하지 않는다. 다음 순서만 수행한다.

1. 설정 및 secret 파일 존재 여부 검사
2. preflight validator 실행
3. 기존 외부 MQTT/RMF endpoint 연결 검사
4. Fleet Adapter와 Arbiter를 production 설정으로 시작
5. `/ready`가 false인 상태에서 clean-start telemetry reconciliation 대기

모든 필수 로봇이 알려진 SafeStop에 정지했고 Block 점유가 없으면 clean-start가 성립한다. 그렇지 않으면 launcher는 계속 실행하되 작업을 받지 않으며 recovery required 원인을 노출한다.

개발용 JWT 생성과 `host.docker.internal` 고정값을 사용하지 않는다. RMF API token은 외부 secret으로 받는다. 장기 운영은 systemd 또는 현장 orchestration에 맡기며 launcher는 foreground 또는 명확한 exit code를 제공한다.

## 실패 처리

- MQTT 인증/TLS 실패: ready=false, 작업 거부, 재연결 시도
- connection OFFLINE/CONNECTIONBROKEN: 해당 로봇 진입 금지; 통로 안이면 관련 Block fault
- stale state: 통로 안이거나 authority 보유 중이면 fault; SafeStop이면 진입 금지
- E-stop, MANUAL, blocking error: 새 authority 금지; 통로 안이면 fault
- map mismatch/position uninitialized: 위치 불명확으로 진입 금지
- RMF API 401: 기존처럼 명시적 실패로 처리하고 예약 해제
- RMF API timeout/5xx: 불확실한 실행 결과를 구분하며 중복 Order를 자동 제출하지 않음
- Arbiter 재시작: 모든 로봇이 SafeStop이고 Block이 비어 있을 때만 clean-start 허용; 그 외에는 recovery required이며 자동 재개 금지

fault 해제와 재시작 복구는 2단계의 인증된 운영 절차가 생기기 전까지 production에서 자동화하지 않는다.

## 시뮬레이션 검증

정상 경로 1:1, 2:1, 2:2, 1:3, 동적 투입 검증은 그대로 유지한다. 다음 장애 시나리오를 추가한다.

1. **통로 내부 MQTT state 중단**: timeout 뒤 Block이 잠기고 반대 방향 진입이 거부되어야 한다.
2. **통로 내부 E-stop**: 즉시 eligibility가 false가 되고 관련 authority가 fail-closed 상태여야 한다.
3. **SafeStop에서 MANUAL 전환**: 해당 로봇 작업만 거부하고 비점유 통로는 불필요하게 fault 처리하지 않아야 한다.
4. **잘못된 mapId/초기화되지 않은 위치**: 알려진 좌표처럼 보이더라도 진입을 허용하지 않아야 한다.
5. **Arbiter 재시작**: telemetry 재수신 전 ready=false이며 작업을 거부해야 한다. 모든 로봇이 SafeStop인 경우에만 clean-start가 되고, 통로 내부 로봇이 있으면 recovery required가 유지되어야 한다.
6. **오래되거나 역순인 state**: 최신 위치와 Block 점유를 되돌리지 않아야 한다.
7. **Broker 단절 후 복구**: 단절 중 작업 거부, 재연결과 정상 telemetry 이후에만 readiness 회복.
8. **잘못된 production 설정**: placeholder, identity 중복, 부족한 calibration, insecure TLS가 각각 명확한 이유로 실패해야 한다.

시뮬레이터에는 특정 AGV 이름에 묶이지 않은 시간/위치/이벤트 기반 fault injection을 추가한다. 예를 들어 `when: entered_block`, `action: set_estop` 또는 `action: disconnect_state`처럼 설정한다. production Python에는 시나리오 robot ID나 node ID를 하드코딩하지 않는다.

## 테스트 전략

구현은 TDD로 진행한다.

- 설정 validator 단위 테스트
- secret redaction과 MQTT client 구성 테스트
- connection/state parsing 및 eligibility 표 기반 테스트
- Task Gate admission/rolling authority 차단 테스트
- `/health`, `/ready`, `/traffic/status` API 테스트
- fault injection simulator 테스트
- 현재 전체 테스트 134개 이상 회귀 실행
- 연결 통로 정상 시나리오와 장애 시나리오 실제 runtime 실행

실제 로봇과의 성공 판정은 시뮬레이션 통과만으로 내리지 않는다. 현장 commissioning에서 실제 topic/payload 캡처, 좌표 calibration residual, 제동거리, SafeStop 위치 및 E-stop 동작을 별도로 확인한다.

## 현장 적용 절차

1. 실제 로봇 설정 또는 vendor 문서에서 topic과 인증 방식을 확인한다.
2. 읽기 전용 MQTT 캡처로 state/connection payload 호환성을 검증한다.
3. 현장 맵을 만들고 최소 3개 비공선 기준점으로 좌표를 보정한다.
4. 로봇 footprint, safety margin, 속도와 최악 제동거리를 측정한다.
5. 실제 정지 가능한 종점·Side Bay만 SafeStop으로 지정한다.
6. production preflight를 통과시킨다.
7. 한 대 저속 주행, 동일 방향 다대, 반대 방향 교행 순으로 commissioning한다.
8. E-stop, 통신 단절, 프로세스 재시작 시험 후 운영 승인을 받는다.

## 완료 기준

1단계는 다음 조건을 모두 만족하면 완료다.

- 기존 시뮬레이션 프로필과 정상 시나리오가 회귀 없이 통과한다.
- 잘못되거나 불완전한 production 설정으로 서비스가 시작되지 않는다.
- MQTT 보안 설정이 Fleet Adapter와 Arbiter 모두에 적용되고 secret이 노출되지 않는다.
- offline, stale, E-stop, MANUAL, map mismatch, blocking error 상태에서 새 통로 권한이 발급되지 않는다.
- 통로 내부 장애는 관련 Block을 fail-closed 처리한다.
- health와 readiness가 원인과 함께 현재 상태를 정확히 나타낸다.
- 장애 주입 시뮬레이션이 위 실패 조건을 재현하고 자동 검증한다.
