ARG ROS_DISTRO=jazzy

# ============================================================================
# Stage 1: Build (rmf_fleet_adapter 제외한 나머지 — 캐시 레이어)
# ============================================================================
FROM ros:${ROS_DISTRO}-ros-base AS builder-base
# FROM osrf/ros:kilted-desktop-full AS builder-base
SHELL ["/bin/bash", "-c"]

ENV http_proxy=http://12.26.204.100:8080
ENV https_proxy=http://12.26.204.100:8080
RUN apt-get update
# ENV DEBIAN_FRONTEND=noninteractive

# 사내 프록시 CA 인증서 등록
COPY McAfee_Certificate.crt /usr/local/share/ca-certificates/McAfee_Certificate.crt
RUN update-ca-certificates
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt
ENV REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /rmf_ws

# ---------------------------------------------------------------------------
# package.xml만 먼저 복사 → rosdep 의존성 레이어 캐시 확보
# (소스 코드가 바뀌어도 package.xml이 동일하면 rosdep install 캐시 유지)
# ---------------------------------------------------------------------------
COPY src/rmf_core/ament_cmake_catch2/ament_cmake_catch2/package.xml          src/rmf/ament_cmake_catch2/ament_cmake_catch2/package.xml
COPY src/rmf_core/rmf_api_msgs/rmf_api_msgs/package.xml                     src/rmf/rmf_api_msgs/rmf_api_msgs/package.xml
COPY src/rmf_core/rmf_battery/rmf_battery/package.xml                       src/rmf/rmf_battery/rmf_battery/package.xml
COPY src/rmf_core/rmf_building_map_msgs/rmf_building_map_msgs/package.xml   src/rmf/rmf_building_map_msgs/rmf_building_map_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_charger_msgs/package.xml            src/rmf/rmf_internal_msgs/rmf_charger_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_dispenser_msgs/package.xml          src/rmf/rmf_internal_msgs/rmf_dispenser_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_door_msgs/package.xml               src/rmf/rmf_internal_msgs/rmf_door_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_fleet_msgs/package.xml              src/rmf/rmf_internal_msgs/rmf_fleet_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_ingestor_msgs/package.xml           src/rmf/rmf_internal_msgs/rmf_ingestor_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_lift_msgs/package.xml               src/rmf/rmf_internal_msgs/rmf_lift_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_obstacle_msgs/package.xml           src/rmf/rmf_internal_msgs/rmf_obstacle_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_reservation_msgs/package.xml        src/rmf/rmf_internal_msgs/rmf_reservation_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_scheduler_msgs/package.xml          src/rmf/rmf_internal_msgs/rmf_scheduler_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_site_map_msgs/package.xml           src/rmf/rmf_internal_msgs/rmf_site_map_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_task_msgs/package.xml               src/rmf/rmf_internal_msgs/rmf_task_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_traffic_msgs/package.xml            src/rmf/rmf_internal_msgs/rmf_traffic_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_workcell_msgs/package.xml           src/rmf/rmf_internal_msgs/rmf_workcell_msgs/package.xml
COPY src/rmf_core/rmf_ros2/rmf_charging_schedule/package.xml                src/rmf/rmf_ros2/rmf_charging_schedule/package.xml
COPY src/rmf_core/rmf_ros2/rmf_task_ros2/package.xml                        src/rmf/rmf_ros2/rmf_task_ros2/package.xml
COPY src/rmf_core/rmf_ros2/rmf_traffic_ros2/package.xml                     src/rmf/rmf_ros2/rmf_traffic_ros2/package.xml
COPY src/rmf_core/rmf_ros2/rmf_websocket/package.xml                        src/rmf/rmf_ros2/rmf_websocket/package.xml
COPY src/rmf_core/rmf_simulation/rmf_building_sim_gz_plugins/package.xml    src/rmf/rmf_simulation/rmf_building_sim_gz_plugins/package.xml
COPY src/rmf_core/rmf_simulation/rmf_robot_sim_common/package.xml           src/rmf/rmf_simulation/rmf_robot_sim_common/package.xml
COPY src/rmf_core/rmf_simulation/rmf_robot_sim_gz_plugins/package.xml       src/rmf/rmf_simulation/rmf_robot_sim_gz_plugins/package.xml
COPY src/rmf_core/rmf_task/rmf_task/package.xml                             src/rmf/rmf_task/rmf_task/package.xml
COPY src/rmf_core/rmf_task/rmf_task_sequence/package.xml                    src/rmf/rmf_task/rmf_task_sequence/package.xml
COPY src/rmf_core/rmf_traffic_editor/rmf_building_map_tools/package.xml     src/rmf/rmf_traffic_editor/rmf_building_map_tools/package.xml
COPY src/rmf_core/rmf_traffic_editor/rmf_traffic_editor_assets/package.xml  src/rmf/rmf_traffic_editor/rmf_traffic_editor_assets/package.xml
COPY src/rmf_core/rmf_traffic_editor/rmf_traffic_editor/package.xml         src/rmf/rmf_traffic_editor/rmf_traffic_editor/package.xml
COPY src/rmf_core/rmf_traffic_editor/rmf_traffic_editor_test_maps/package.xml src/rmf/rmf_traffic_editor/rmf_traffic_editor_test_maps/package.xml
COPY src/rmf_core/rmf_traffic/rmf_traffic_examples/package.xml              src/rmf/rmf_traffic/rmf_traffic_examples/package.xml
COPY src/rmf_core/rmf_traffic/rmf_traffic/package.xml                       src/rmf/rmf_traffic/rmf_traffic/package.xml
COPY src/rmf_core/rmf_utils/rmf_utils/package.xml                           src/rmf/rmf_utils/rmf_utils/package.xml
COPY src/rmf_core/rmf_visualization_msgs/rmf_visualization_msgs/package.xml src/rmf/rmf_visualization_msgs/rmf_visualization_msgs/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_building_systems/package.xml src/rmf/rmf_visualization/rmf_visualization_building_systems/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_fleet_states/package.xml     src/rmf/rmf_visualization/rmf_visualization_fleet_states/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_floorplans/package.xml       src/rmf/rmf_visualization/rmf_visualization_floorplans/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_navgraphs/package.xml        src/rmf/rmf_visualization/rmf_visualization_navgraphs/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_obstacles/package.xml        src/rmf/rmf_visualization/rmf_visualization_obstacles/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization/package.xml                  src/rmf/rmf_visualization/rmf_visualization/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_rviz2_plugins/package.xml    src/rmf/rmf_visualization/rmf_visualization_rviz2_plugins/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_schedule/package.xml         src/rmf/rmf_visualization/rmf_visualization_schedule/package.xml
COPY src/rmf_core/thirdparty/menge_vendor/package.xml                           src/thirdparty/menge_vendor/package.xml
COPY src/rmf_core/thirdparty/nlohmann_json_schema_validator_vendor/package.xml  src/thirdparty/nlohmann_json_schema_validator_vendor/package.xml

# ---------------------------------------------------------------------------
# rosdep: package.xml만 변경되지 않으면 이 레이어가 캐시됨
# ---------------------------------------------------------------------------
ARG ROS_DISTRO=jazzy
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -yr \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 전체 소스 복사 후 빌드 (fleet_adapter 제외)
# ---------------------------------------------------------------------------
COPY src/rmf_core/ament_cmake_catch2 src/rmf/ament_cmake_catch2
COPY src/rmf_core/rmf_api_msgs src/rmf/rmf_api_msgs
COPY src/rmf_core/rmf_battery src/rmf/rmf_battery
COPY src/rmf_core/rmf_building_map_msgs src/rmf/rmf_building_map_msgs
COPY src/rmf_core/rmf_internal_msgs src/rmf/rmf_internal_msgs
COPY src/rmf_core/rmf_simulation src/rmf/rmf_simulation
COPY src/rmf_core/rmf_task src/rmf/rmf_task
COPY src/rmf_core/rmf_traffic src/rmf/rmf_traffic
COPY src/rmf_core/rmf_traffic_editor src/rmf/rmf_traffic_editor
COPY src/rmf_core/rmf_utils src/rmf/rmf_utils
COPY src/rmf_core/rmf_visualization src/rmf/rmf_visualization
COPY src/rmf_core/rmf_visualization_msgs src/rmf/rmf_visualization_msgs
COPY src/rmf_core/rmf_ros2/rmf_charging_schedule src/rmf/rmf_ros2/rmf_charging_schedule
COPY src/rmf_core/rmf_ros2/rmf_task_ros2 src/rmf/rmf_ros2/rmf_task_ros2
COPY src/rmf_core/rmf_ros2/rmf_traffic_ros2 src/rmf/rmf_ros2/rmf_traffic_ros2
COPY src/rmf_core/rmf_ros2/rmf_websocket src/rmf/rmf_ros2/rmf_websocket
COPY src/rmf_core/thirdparty src/thirdparty

# Build (fleet_adapter 제외) — 변경 없으면 캐시됨
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        --event-handlers console_direct+

# ============================================================================
# Stage 2: Build rmf_fleet_adapter + python bindings (패치 변경 시 여기만 재빌드)
# ============================================================================
FROM builder-base AS builder

# package.xml만 먼저 복사 → rosdep 캐시 확보
COPY src/rmf_core/rmf_ros2/rmf_fleet_adapter/package.xml         src/rmf/rmf_ros2/rmf_fleet_adapter/package.xml
COPY src/rmf_core/rmf_ros2/rmf_fleet_adapter_python/package.xml  src/rmf/rmf_ros2/rmf_fleet_adapter_python/package.xml
COPY src/rmf_core/rmf_ros2/rmf_reservation_node/package.xml      src/rmf/rmf_ros2/rmf_reservation_node/package.xml

ARG ROS_DISTRO=jazzy
RUN apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -yr \
    && rm -rf /var/lib/apt/lists/*

# 전체 소스 복사
COPY src/rmf_core/rmf_ros2/rmf_fleet_adapter src/rmf/rmf_ros2/rmf_fleet_adapter
COPY src/rmf_core/rmf_ros2/rmf_fleet_adapter_python src/rmf/rmf_ros2/rmf_fleet_adapter_python
COPY src/rmf_core/rmf_ros2/rmf_reservation_node src/rmf/rmf_ros2/rmf_reservation_node

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . install/setup.sh \
    && colcon build \
        --packages-select rmf_fleet_adapter rmf_fleet_adapter_python rmf_reservation_node \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        --event-handlers console_direct+

# ============================================================================
# Stage 3: Runtime
# ============================================================================
FROM ros:${ROS_DISTRO}-ros-base AS runtime

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

ENV http_proxy=http://12.26.204.100:8080
ENV https_proxy=http://12.26.204.100:8080

# 사내 프록시 CA 인증서 등록
COPY McAfee_Certificate.crt /usr/local/share/ca-certificates/McAfee_Certificate.crt
RUN update-ca-certificates
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt
ENV REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt

# ---------------------------------------------------------------------------
# 고정 의존성 먼저 설치 (빌드 결과와 무관 → 캐시 최대 활용)
# ---------------------------------------------------------------------------
ARG ROS_DISTRO=jazzy

# CycloneDDS + rviz2 + X11 런타임 의존성
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-launch-xml \
        libx11-6 libxext6 libxrender1 libgl1 libglu1-mesa \
        python3-pip \
    && pip3 install --no-cache-dir --break-system-packages \
        "paho-mqtt>=2.0" pyyaml networkx nudged \
        fastapi "uvicorn[standard]" \
    && apt-get purge -y python3-pip \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# rosdep 런타임 의존성 (package.xml 기반)
# ---------------------------------------------------------------------------
WORKDIR /rmf_ws

ENV http_proxy=http://12.26.204.100:8080
ENV https_proxy=http://12.26.204.100:8080

# package.xml만 복사하여 rosdep 실행 (빌드 결과 변경과 무관하게 캐시)
COPY src/rmf_core/ament_cmake_catch2/ament_cmake_catch2/package.xml          src/rmf/ament_cmake_catch2/ament_cmake_catch2/package.xml
COPY src/rmf_core/rmf_api_msgs/rmf_api_msgs/package.xml                     src/rmf/rmf_api_msgs/rmf_api_msgs/package.xml
COPY src/rmf_core/rmf_battery/rmf_battery/package.xml                       src/rmf/rmf_battery/rmf_battery/package.xml
COPY src/rmf_core/rmf_building_map_msgs/rmf_building_map_msgs/package.xml   src/rmf/rmf_building_map_msgs/rmf_building_map_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_charger_msgs/package.xml            src/rmf/rmf_internal_msgs/rmf_charger_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_dispenser_msgs/package.xml          src/rmf/rmf_internal_msgs/rmf_dispenser_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_door_msgs/package.xml               src/rmf/rmf_internal_msgs/rmf_door_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_fleet_msgs/package.xml              src/rmf/rmf_internal_msgs/rmf_fleet_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_ingestor_msgs/package.xml           src/rmf/rmf_internal_msgs/rmf_ingestor_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_lift_msgs/package.xml               src/rmf/rmf_internal_msgs/rmf_lift_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_obstacle_msgs/package.xml           src/rmf/rmf_internal_msgs/rmf_obstacle_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_reservation_msgs/package.xml        src/rmf/rmf_internal_msgs/rmf_reservation_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_scheduler_msgs/package.xml          src/rmf/rmf_internal_msgs/rmf_scheduler_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_site_map_msgs/package.xml           src/rmf/rmf_internal_msgs/rmf_site_map_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_task_msgs/package.xml               src/rmf/rmf_internal_msgs/rmf_task_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_traffic_msgs/package.xml            src/rmf/rmf_internal_msgs/rmf_traffic_msgs/package.xml
COPY src/rmf_core/rmf_internal_msgs/rmf_workcell_msgs/package.xml           src/rmf/rmf_internal_msgs/rmf_workcell_msgs/package.xml
COPY src/rmf_core/rmf_ros2/rmf_charging_schedule/package.xml                src/rmf/rmf_ros2/rmf_charging_schedule/package.xml
COPY src/rmf_core/rmf_ros2/rmf_fleet_adapter/package.xml                    src/rmf/rmf_ros2/rmf_fleet_adapter/package.xml
COPY src/rmf_core/rmf_ros2/rmf_fleet_adapter_python/package.xml             src/rmf/rmf_ros2/rmf_fleet_adapter_python/package.xml
COPY src/rmf_core/rmf_ros2/rmf_reservation_node/package.xml                 src/rmf/rmf_ros2/rmf_reservation_node/package.xml
COPY src/rmf_core/rmf_ros2/rmf_task_ros2/package.xml                        src/rmf/rmf_ros2/rmf_task_ros2/package.xml
COPY src/rmf_core/rmf_ros2/rmf_traffic_ros2/package.xml                     src/rmf/rmf_ros2/rmf_traffic_ros2/package.xml
COPY src/rmf_core/rmf_ros2/rmf_websocket/package.xml                        src/rmf/rmf_ros2/rmf_websocket/package.xml
COPY src/rmf_core/rmf_simulation/rmf_building_sim_gz_plugins/package.xml    src/rmf/rmf_simulation/rmf_building_sim_gz_plugins/package.xml
COPY src/rmf_core/rmf_simulation/rmf_robot_sim_common/package.xml           src/rmf/rmf_simulation/rmf_robot_sim_common/package.xml
COPY src/rmf_core/rmf_simulation/rmf_robot_sim_gz_plugins/package.xml       src/rmf/rmf_simulation/rmf_robot_sim_gz_plugins/package.xml
COPY src/rmf_core/rmf_task/rmf_task/package.xml                             src/rmf/rmf_task/rmf_task/package.xml
COPY src/rmf_core/rmf_task/rmf_task_sequence/package.xml                    src/rmf/rmf_task/rmf_task_sequence/package.xml
COPY src/rmf_core/rmf_traffic_editor/rmf_building_map_tools/package.xml     src/rmf/rmf_traffic_editor/rmf_building_map_tools/package.xml
COPY src/rmf_core/rmf_traffic_editor/rmf_traffic_editor_assets/package.xml  src/rmf/rmf_traffic_editor/rmf_traffic_editor_assets/package.xml
COPY src/rmf_core/rmf_traffic_editor/rmf_traffic_editor/package.xml         src/rmf/rmf_traffic_editor/rmf_traffic_editor/package.xml
COPY src/rmf_core/rmf_traffic_editor/rmf_traffic_editor_test_maps/package.xml src/rmf/rmf_traffic_editor/rmf_traffic_editor_test_maps/package.xml
COPY src/rmf_core/rmf_traffic/rmf_traffic_examples/package.xml              src/rmf/rmf_traffic/rmf_traffic_examples/package.xml
COPY src/rmf_core/rmf_traffic/rmf_traffic/package.xml                       src/rmf/rmf_traffic/rmf_traffic/package.xml
COPY src/rmf_core/rmf_utils/rmf_utils/package.xml                           src/rmf/rmf_utils/rmf_utils/package.xml
COPY src/rmf_core/rmf_visualization_msgs/rmf_visualization_msgs/package.xml src/rmf/rmf_visualization_msgs/rmf_visualization_msgs/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_building_systems/package.xml src/rmf/rmf_visualization/rmf_visualization_building_systems/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_fleet_states/package.xml     src/rmf/rmf_visualization/rmf_visualization_fleet_states/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_floorplans/package.xml       src/rmf/rmf_visualization/rmf_visualization_floorplans/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_navgraphs/package.xml        src/rmf/rmf_visualization/rmf_visualization_navgraphs/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_obstacles/package.xml        src/rmf/rmf_visualization/rmf_visualization_obstacles/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization/package.xml                  src/rmf/rmf_visualization/rmf_visualization/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_rviz2_plugins/package.xml    src/rmf/rmf_visualization/rmf_visualization_rviz2_plugins/package.xml
COPY src/rmf_core/rmf_visualization/rmf_visualization_schedule/package.xml         src/rmf/rmf_visualization/rmf_visualization_schedule/package.xml
COPY src/rmf_core/thirdparty/menge_vendor/package.xml                           src/thirdparty/menge_vendor/package.xml
COPY src/rmf_core/thirdparty/nlohmann_json_schema_validator_vendor/package.xml  src/thirdparty/nlohmann_json_schema_validator_vendor/package.xml

RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -yr \
        --dependency-types exec \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf src

# ---------------------------------------------------------------------------
# 빌드 결과 복사
# ---------------------------------------------------------------------------
COPY --from=builder /rmf_ws/install install

# Source the workspace on entry
RUN sed -i '$isource "/rmf_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
