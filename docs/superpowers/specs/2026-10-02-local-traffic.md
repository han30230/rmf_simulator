# RMF 국소 교행 개선: 구현 및 실제 시뮬레이션 작업 지시

이 문서 전체를 구현 에이전트에게 전달한다. 아래 설계는 검증할 가설이다. 성공을 미리 가정하지 말고 실제 소스, 빌드, 런타임 로그와 로봇 궤적으로 입증하라. 코드 변경 후 실제 RMF/VDA5050 시뮬레이터까지 실행하는 것이 목표다.

## 목표와 사용자 조건

- 기존 Open-RMF / ROS 2 Jazzy를 개선한다. 지정한 1차선 양방향 통로와 실제 교행 가능한 측면 베이가 대상이다.
- 최초부터 비충돌 경로와 적절한 대기를 계획하고, 일반 replan에도 적용한다.
- 핵심은 A/B의 1vs1 교행 진행 중 C가 신규 task를 받아 들어오고, 이후 D가 추가되어도 교착 없이 완료하는 것이다.
- 현재 주요 평가 규모는 2~4대이지만 알고리즘과 인터페이스에서 로봇 수를 4대로 하드코딩하지 않는다.
- 우회 거리 증가는 허용한다. 비용·계산시간·탐색 노드 예산을 확대하되 무제한 계산과 통신/제어 콜백 정지는 금지한다.
- 현재 위치에서 베이까지의 접근, 대기, 상대 통과, 원래 목적지까지의 복귀를 모두 검증한다.
- 후진/회전의 실제 지원 여부는 graph, vehicle traits, adapter와 simulator에서 확인한다. 임의로 후진을 허용하거나 금지하지 않는다.
- task 앞에서만 막는 외부 arbiter, VDA5050 명령을 사후 차단하는 방식으로 해결하지 않는다. RMF 내부 계획과 실행을 일치시킨다.
- 로봇 이름이나 waypoint 번호를 코드에 박지 않는다. 통로·베이·안전 대기 지점은 설정으로 지정할 수 있다.

## 0. 현재 환경 및 기준 실행부터 확인

1. AGENTS.md와 repository 지침을 읽고 기존 작업을 보존한다. 격리된 브랜치/작업 디렉터리를 사용한다.
2. 실제 rmf_ros2, rmf_traffic, adapter, simulator commit, ROS distro, build overlay와 실행 binary 경로를 기록한다. 소스를 수정했으나 설치된 옛 binary로 테스트하는 실수를 방지한다.
3. 기존 map/navigation graph, 로봇 footprint, bay 연결·방향·회전 가능성, passthrough/holding 플래그를 확인한다. 표시상 빈 베이와 물리적으로 교행 가능한 베이를 구분한다.
4. 기존 실패 시나리오를 동일 task 순서와 투입 시점으로 재현하여 baseline 로그를 보존한다. baseline이 재현되지 않으면 작업을 포기하지 말고 재현 가능한 조건과 차이를 기록한다.
5. 실패를 비용 제한, 탐색 saturation, 취소/timeout, graph 불연결, 물리적 탈출 불가, 스케줄 충돌, 계획 적용 실패로 구분한다. timeout은 해가 없다는 증명이 아니다.

## 확인된 upstream 구조 — 실제 체크아웃과 대조할 것

검토 기준은 rmf_ros2 02da9198b246980bbfd6d9fbc86685dfa2212bd6, rmf_traffic ab881a6e841b235a0b58b9acc87dae8549614676이다. 이 커밋으로 설치 환경을 강제로 변경하지 말고 실제 사용 버전의 차이를 먼저 확인한다.

- rmf_fleet_adapter/events/GoToPlace.cpp: Active::_find_plan()이 목표 선택 이후 services::FindPath를 생성한다. 시작 상태, goal, schedule snapshot, participant ID, profile, 시간 제한을 넘긴다. 결과를 _execute_plan()에 전달한다.
- 최초 GoToPlace 이동과 일반 replan은 _find_plan()을 사용한다. 협상은 별도 _respond() → services::Negotiate::path 경로다. 모든 task 비용 계산/충전/주차가 자동으로 같은 경로를 탄다고 가정하지 말고 각각 추적한다.
- FindPath → jobs::SearchForPath → jobs::Planning → traffic Planner 구조다. compliant 성공을 우선하고 그렇지 않으면 greedy 결과를 반환할 수 있다.
- greedy는 schedule validator가 없다. compliant는 ScheduleRouteValidator를 사용한다. greedy 성공을 공동 비충돌 계획 성공으로 취급하면 안 된다.
- SearchForPath의 초기 compliant 비용 한계와 이후 재개 시 비용 한계 갱신을 모두 확인한다. Planning의 saturation 설정도 확인한다.
- 해당 SearchForPath.cpp의 compliant 설정 뒤에 greedy_options.interrupter(...)를 다시 호출하는 줄이 있다. 실제 버전에서 compliant 취소/시간 제한이 어디에서 적용되는지 추적하고 테스트하라. 줄 하나만 보고 timeout이 확실히 작동하거나 확실히 안 작동한다고 단정하지 않는다.
- GoToPlace 기준 내부 5초 계획 제한과 외부 10초 timeout이 존재한다. 시간 확대 시 관련 wrapper와 재시도 정책을 함께 점검한다.
- traffic DifferentialDrivePlanner::expand_hold()가 대기 trajectory를 만들고 validator로 검사한다. passthrough waypoint에서는 대기 후보가 거절된다. holding flag만으로 베이 우회가 강제된다고 가정하지 않는다.
- agv::Plan 및 Plan::Waypoint 생성자는 private이다. start→bay Plan과 bay→goal Plan을 public API로 쉽게 합칠 수 있다고 가정하지 않는다. 필요 시 최소한의 내부 composition 또는 단계 실행 인터페이스를 설계하고 전체 의미를 검증한다.
- ExecutePlan이 schedule itinerary와 이동 phases/dependencies를 처리한다. 일정만 바꾸고 로봇 명령을 바꾸지 않거나 그 반대로 처리하지 않는다.
- WaitForTraffic는 dependency의 reached 또는 deprecated 상태를 처리한다. 폐기된 dependency를 물리적 통과 확인으로 오인하지 않도록 새 정책과의 상호작용을 검증한다.
- project_itinerary가 미래 목적지까지 예측 경로를 추가할 수 있다. 실제 명령/실행 중인 구간과 예상 tail을 구분한다.

## 1. 한계 확대와 진단 로그

비용 여유 배수, 추가 대기 허용 비용, 탐색 노드 한계, solve 전체 시간 예산을 설정 가능하게 한다. 한계가 각 탐색 경로에 실제로 전달되는지 테스트한다. 일반 FindPath와 SimpleNegotiator/CentralizedNegotiation의 옵션 전달은 다를 수 있다.

로그에는 요청 ID, robot/participant ID, 원래 goal, 시작 보고 시각, schedule/plan version, greedy/compliant 선택, 실패 이유, 비용 추정/한계, saturation, 계산시간을 남긴다. 전역 옵션을 무조건 변경하여 모든 계획 호출에 영향을 주지 말고 적용 범위를 명시한다.

## 2. 새 solver를 만들기 전에 RMF 기존 공동 탐색을 시험

rmf_traffic::agv::CentralizedNegotiation을 우선 평가한다.

- Agent 입력은 participant ID, StartSet, Goal, Planner, SimpleNegotiator options다.
- viewer와 등록된 schedule 참가자가 필요하다. Result::proposal()은 participant ID → Plan 묶음이며 blockers도 제공한다.
- 내부는 SimpleNegotiator와 schedule::Negotiation을 사용한다. CBS/ECBS가 아니며 기존 협상의 제한을 자동으로 제거하지 않는다.
- optimal(true)가 완전 탐색/항상 해결을 보장한다고 해석하지 않는다. 해당 구현의 table version skip 등 중단 조건을 확인한다.
- solve()는 동기 호출이다. fleet worker에서 직접 실행하지 않는다. 전체 예산과 실제 취소 가능성을 확인한다. Future timeout만 걸고 계산 thread를 방치하지 않는다. shared bool의 무동기 동시 변경도 금지한다.
- SimpleNegotiator option과 Planner default option의 실제 전달을 추적한다. 비용 minimum threshold의 문서와 실제 수식 차이도 확인하고 비용 단위를 맞춘다.

실제 맵에서 최초에는 원래 목적지를 유지한 2대, 3대, 4대 공동 계획을 실행한다. 결과의 모든 pair와 외부 참가자에 대해 profile 및 시간 trajectory 기준으로 충돌을 검사한다. schedule horizon 끝에서 정지 로봇이 사라지는 모델을 사용하지 않는다.

성공하면 이 backend를 재사용한다. 실패하면 어떤 후보가 누락되었는지 로그로 설명한 후 베이 경유/양보 후보 생성을 최소 범위로 추가한다. 기존 helper가 실패하는데 이름만 바꿔 포장하지 않는다. 유효한 후보를 찾을 수 없다면 탐색 한계나 물리적 불가능성을 보고하고, 무조건 성공했다고 처리하지 않는다.

## 3. 공유 LocalTrafficCoordinator 통합

명칭은 제안이며 실제 코드 관례에 맞춰 변경 가능하다.

- 같은 fleet/process에서 공유하는 coordinator를 FleetUpdateHandle 수준에 두고 RobotContext를 통해 접근한다.
- GoToPlace::_find_plan()에서 goal 선택 후 기존 FindPath 호출 자리를 조건부 분기한다. 지정 통로 밖은 기존 동작을 유지한다.
- 각 활성 이동 event가 원래 goal과 요청 generation을 등록하고 완료/취소/중단 시 정리한다. queued task의 최종 상태를 현재 이동 goal로 오인하지 않는다.
- 위치/실행 상태는 fleet worker에서 일관된 snapshot으로 수집하고, solve는 별도 executor에서 수행한다. 결과는 worker로 되돌려 재검증한다.
- 계획 없는 idle 로봇의 현재 점유도 장애물/참여자로 고려한다. 같은 통로를 쓰는 다른 fleet를 발견하면 외부 제약으로 다루고, 공동 변경이 필요한 경우 현재 지원 범위와 확장 방법을 보고한다.
- request_plan, collect_snapshot, solve_group, validate_result, apply_group 같은 책임으로 나누되 함수 이름이나 개수에 맞추려고 설계를 왜곡하지 않는다.

## 4. 베이 후보 생성이 필요할 때의 정확한 조건

단순히 A→B→C 순서로 한 대씩 계획하는 방식은 완전하지 않다. 아직 계획하지 않은 로봇의 현재 점유와 필수 대기를 삭제하면 안 된다.

- 후보 예: A가 bay X로 이동하는 동안 B는 안전 위치에서 대기 → A의 통로 이탈 확인 뒤 B 통과 → B 이탈 확인 뒤 A가 원래 goal로 진행.
- 양쪽 양보 후보와 접근 가능한 베이들을 검토한다. 베이 진입·대기·이탈·원래 목적지까지의 경로 모두 검사한다.
- 베이 도착 자체를 원래 GoToPlace/task 완료로 취급하지 않는다.
- 시간표만 검증하지 말고 진행 확인 dependency의 방향과 대기 순환을 검사한다. A waits B, B waits A 같은 cycle을 승인하지 않는다.
- 움직이는 동안의 footprint, 회전 swept area, 대기 중 점유와 목적지 도착 후 점유를 포함한다.
- 중간 waypoint로 goal을 바꾸는 방식, Plan composition, staged event 실행 중 하나를 실제 API에 맞춰 선택하고 원래 event 완료/취소/replan 의미가 보존되는지 테스트한다.

## 5. 온라인 투입과 계획 적용 — 필수

- A/B 교행이 실행 가능하면 C 추가 시 우선 A/B의 필요한 공간을 보존하고 C의 안전 대기 후보를 탐색한다. 가능하지 않을 때만 관련 그룹의 변경 가능한 미래 구간을 다시 계획한다.
- 현재 점유, 멈추기 전 필수 이동 구간, 제동/명령 교체 제약을 보존한다. 전체 옛 미래 경로를 고정해서 이미 실패한 상태를 영구히 유지하지 않는다.
- 설정된 시간 예산과 로봇 상태/목적지/plan generation을 적용 직전에 확인한다. 단순히 schedule global version만 달라졌다고 무조건 폐기해 starvation을 만들지 말고 관련 변경을 판단한다.
- RMF에는 여러 참가자의 itinerary와 로봇 명령을 한 번에 바꾸는 원자적 commit이 있다고 가정하지 않는다. 실제 가능한 안전한 교체 지점, 적용 acknowledgement, 상태 전환을 설계한다. 중간 실패/취소도 처리한다.
- GoToPlace::_respond()의 개별 협상이 공동 결정과 충돌하지 않도록 연동한다. 그룹 외부 협상은 유지한다. 협상을 전역으로 끄거나 stale 제안을 무조건 승인하지 않는다.
- actual progress로 구간 release를 확인한다. 고정 sleep, 예상 도착 시각, deprecated dependency만으로 release하지 않는다. timeout 시 출발 허가가 아니라 재검증/복구로 이동한다.
- 실패 시 현재 상태에서 유효한 대기/정지 계획을 생성하거나 기존 검증된 실행을 유지한다. 정지도 안전하다고 자동 가정하지 않는다. 유효한 조치가 없으면 실패 상태를 명확히 보고한다.
- 모든 그룹 계획이 실패했다고 충돌 검사를 통과하지 않은 greedy 계획을 자동 실행하지 않는다. 복구는 별도 정책으로 구현하고 기록한다.
- 대기 시간 기반 공정성을 두어 한 방향/한 로봇의 영구 대기를 방지한다.

## 6. 실제 시뮬레이션 평가

planner-only 테스트와 실제 fleet adapter + schedule + robot command + simulator 실행을 구분한다. 최종 통과는 후자의 실행 결과로 판단한다. 실제 맵을 우선 사용하고 작은 재현용 graph도 유지한다.

필수 시나리오:

1. 로봇 1대 이동: 기존 동작 회귀 확인.
2. 양방향 1vs1: 서로 진입 전 계획, 한 대가 베이를 사용하는 완료.
3. A가 베이에 접근하는 중 C 신규 task 투입.
4. A가 베이에 들어간 뒤 B가 통과하는 중 C 투입.
5. C 대기 중 D 투입, 2vs2와 1vs3, 같은 방향 연속 투입.
6. 베이 점유/idle blocker, 출구 점유, 목적지가 베이 또는 통로 내부인 경우.
7. 로봇 속도 차이, 임의 지연, 일시 정지, 보고 지연과 통신 중단.
8. task 취소/goal 변경, replan, 실행 중 기존 협상 발생, 일부 계획 적용 실패.
9. park/charge 이동의 실제 호출 경로와 적용 범위 확인.
10. 외부 fleet 로봇과의 충돌 검사, 여러 통로 또는 공유 베이의 지원/제한 확인.
11. 물리적으로 해결 불가능한 경우: 올바른 실패/대기 상태, 허위 성공과 강제 통과가 없는지 확인.

시나리오 3~5는 투입 위치와 시점을 sweep하고, simulator가 지원하면 seed를 바꿔 반복한다. 기본 반복 목표는 각 핵심 시나리오 10회다. 실행 시간상 줄이면 횟수와 미검증 범위를 밝힌다. task 입력은 같게 유지하고 baseline/변경 버전을 비교한다.

기록할 지표:

- profile 기반 최소 이격/실제 충돌 수, 겹치는 반대 방향 구간 점유.
- 개별 원래 task 완료와 전체 완료, 최대 대기, 반복 replan/계획 교체, starvation.
- 분산 협상 발생 수, 중앙 solver 호출 수와 탐색량을 분리한다. 분산 협상만 줄었다고 전체 부하 감소로 결론내리지 않는다.
- solve p50/p95/max, timeout/saturation, CPU/RSS, stale 결과 폐기.
- 스케줄 plan ID와 실제 command generation/진행 상태의 대응, bay occupancy/release 로그.

안전 조건은 단순 schedule 충돌 0이 아니라 실제 simulator 위치·크기를 기준으로 확인한다. 완료 시간은 물리적 이동시간과 허용 대기를 반영해 시나리오별 상한을 먼저 정하고 60초 runner timeout만으로 교착을 단정하지 않는다. 측정과 임계값을 코드 변경 후 유리하게 바꾸지 않는다.

## 7. 산출물과 작업 지속

필요한 파일 조사·빌드·테스트·가역적인 소스 수정은 자율적으로 수행하고, 단계마다 사용자의 재승인을 기다리지 않는다. 다만 물리적으로 불가능한 상태를 성공으로 꾸미거나 실제 로봇에 배포하지 않는다. 이 작업의 실행 대상은 시뮬레이션이다.

- 커밋 가능한 변경과 feature flag/설정, 기본 off로 기존 동작 유지.
- 재현 스크립트와 task 투입 파일, 정확한 빌드/실행 명령.
- baseline 및 개선 로그, 궤적 replay/시각화, 반복 실행 결과 표.
- 환경/commit/binary 경로, 코드상 근거, 알려진 제한과 미검증 항목.
- 실패한 경우 실패 후보와 blocker/제한 원인을 남기고 원인에 맞는 최소 수정으로 재검증한다. 초기 설계를 반드시 성공시키려고 사실을 왜곡하지 않는다.
- 최종 보고는 “구현”, “빌드 통과”, “planner-only 통과”, “실제 동적 시뮬 통과”를 분리한다. 실행하지 못했다면 정확한 blocker와 필요한 환경을 보고한다.

## 참고 소스

- https://github.com/open-rmf/rmf_ros2/blob/02da9198b246980bbfd6d9fbc86685dfa2212bd6/rmf_fleet_adapter/src/rmf_fleet_adapter/events/GoToPlace.cpp
- https://github.com/open-rmf/rmf_ros2/blob/02da9198b246980bbfd6d9fbc86685dfa2212bd6/rmf_fleet_adapter/src/rmf_fleet_adapter/jobs/SearchForPath.cpp
- https://github.com/open-rmf/rmf_ros2/blob/02da9198b246980bbfd6d9fbc86685dfa2212bd6/rmf_fleet_adapter/src/rmf_fleet_adapter/services/detail/impl_FindPath.hpp
- https://github.com/open-rmf/rmf_traffic/blob/ab881a6e841b235a0b58b9acc87dae8549614676/rmf_traffic/include/rmf_traffic/agv/CentralizedNegotiation.hpp
- https://github.com/open-rmf/rmf_traffic/blob/ab881a6e841b235a0b58b9acc87dae8549614676/rmf_traffic/src/rmf_traffic/agv/CentralizedNegotiation.cpp
- https://github.com/open-rmf/rmf_traffic/blob/ab881a6e841b235a0b58b9acc87dae8549614676/rmf_traffic/src/rmf_traffic/agv/SimpleNegotiator.cpp
- https://github.com/open-rmf/rmf_traffic/blob/ab881a6e841b235a0b58b9acc87dae8549614676/rmf_traffic/test/unit/agv/test_Negotiation_edgecases.cpp

## 이 문서 작성 시 검증 범위

위 upstream API/소스를 재조회하여 확인했다. CentralizedNegotiation의 다중 Agent 입력, Plan 묶음 반환, SimpleNegotiator 사용, table version > 2 skip, 동기 solve와 private Plan 생성자를 확인했다. 기존 edgecase test에 solve/proposal 확인 코드가 있다. 이는 사용자 맵이나 동적 투입에서의 성공 증거가 아니다.

이 문서를 작성한 환경에는 ros2/colcon 및 /opt/ros가 없어 실제 RMF 빌드와 시뮬레이션을 실행하지 않았다. 구현된 패치나 통과 로그가 존재한다고 주장하지 않는다. 위 작업을 실제 개발/시뮬레이션 환경에서 수행해야 한다.
