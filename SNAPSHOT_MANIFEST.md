# Snapshot Manifest — 2026-09-16

이 저장소를 초기화할 때 ChatGPT 대화/Library에서 확인한 자료 목록입니다.

## 최신 통합 계열

- `rmf_simulation_workspace_direction_arbiter_2026-09-16.zip`
- `rmf_direction_arbiter_p4_afternoon_patch_2026-09-16.zip`
- `rmf_p4_home_runtime_patch_2026-09-16.zip`
- `rmf_simulation_transfer_checkpoint.zip`

최신 스냅샷은 첫 ZIP을 base로 사용하고 9/16 P4 patches를 순서대로 overlay하여 검증했습니다.

## VDA5050 복원/Recovery

- `vda5050_t2_reconstructed_2026-09-15.zip`
- `vda5050_recovered_sources_partial_clean_2026-09-15.zip`
- `vda5050_gui_recovered.py`
- `2026-09-15-vda5050-t2-reconstruction-design.md`

복원본은 실제 사내 원본과 동일함을 보장하지 않습니다.

## 설계 문서

- `directional_block_arbiter_implementation_spec.md`
- `direction_arbiter_segmented_corridor_implementation_spec.md`
- `corridor_manager_spec.md`

## 과거 Traffic Lab 변경 이력

- `rmf_lab_v23_20_cbs_roadmap_poc.patch`
- `rmf_lab_v23_20_to_v23_21_corridor_manager.patch`
- `rmf_lab_v23_21_3_to_v23_22_general_corridor_manager.patch`
- `rmf_lab_v23_22_to_v23_23_segmented_corridor_manager.patch`
- `rmf_lab_v23_23_to_v23_24_corridor_ux_and_spawn_fix.patch`

## 최신 스냅샷에서 확인된 핵심 파일

```text
traffic_control/
  models.py
  corridor_registry.py
  direction_arbiter.py
  robot_tracker.py
  task_gate.py

config/
  corridor_blocks.yaml
  corridor_blocks_segmented_example.yaml
  corridor_blocks_p4.yaml

scripts/
  run_direction_arbiter.sh
  t4_dispatch_via_arbiter.sh
  verify_workspace.py

rmf_dev_tool-main/vda5050_robot_simulator/
  run.py
  p4_scenario.yaml
  vda5050_simulator/*.py

rmf_platform-main/
  docker-compose.yml
  docker-compose.p4.yml
  src/rmf_vda5050_fleet_adapter/config/p4_edit_before.yaml
  src/rmf_vda5050_fleet_adapter/map/p4_edit_node_add.yaml

tests/
  Direction Arbiter / occupancy / dynamic insertion / fault / P4 config tests
```

## 검증

정리된 최신 스냅샷의 root tests:

```text
34 passed
```

## GitHub connector 제한

현재 ChatGPT GitHub connector는 repository file/blob 내용을 직접 작성할 수 있지만, 이 세션의 `/mnt/data` 로컬 파일이나 ZIP/TAR를 `git push`처럼 직접 전달하는 file-upload parameter를 제공하지 않습니다.

따라서 이 저장소에는 현재 README/manifest/history부터 직접 기록하고 있습니다. Library에 보존된 바이너리 checkpoint/ZIP 자체는 이 커넥터 경로로 그대로 업로드할 수 없습니다.

정확한 WSL working tree 전체를 GitHub에 반영하려면 마지막 단계에서 대상 PC의 workspace에서 일반 `git add/commit/push`를 실행하는 것이 source-of-truth입니다.
