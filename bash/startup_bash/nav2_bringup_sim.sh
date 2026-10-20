#!/bin/bash

trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' INT

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
# Direct invocation must use the same simulation domain as launcher invocation.
source "${WORKSPACE_ROOT}/bash/activate_koko_sim_env.sh"
SAVED_MAPS_DIR="${WORKSPACE_ROOT}/maps_waypoints/maps"
BUILTIN_MAP="${WORKSPACE_ROOT}/src/robotbase_sim/maps/test_arena.yaml"
WORLD_HELPER="${WORKSPACE_ROOT}/other_programs/sirius_launcher/simulation_worlds.py"
WORLD_MAP="$(/usr/bin/python3 "$WORLD_HELPER" --current-map)" || exit 1
WORLD_POSE="$(/usr/bin/python3 "$WORLD_HELPER" --current-pose)" || exit 1

nav2_command=(ros2 launch robotbase_sim navigation.launch.py)
room_mode=false
if [ "${1:-}" = "--people" ] || [ "${1:-}" = "--people-room" ]; then
    [ "${1:-}" = "--people-room" ] && room_mode=true
    shift
    nav2_command=(ros2 launch external_camera_people_layer simulation.launch.py
        "params_file:=${ROBOTBASE_PARAMS_DIR:-${WORKSPACE_ROOT}/params}/sim/nav2.yaml")
fi

if [ "${1:-}" = "--world-current" ]; then
    if [ -z "$WORLD_MAP" ]; then
        echo "対応する地図がありません。koko_simで同梱ワールドを選んでください。" >&2
        exit 1
    fi
    cd -- "$WORKSPACE_ROOT" || exit 1
    source install/setup.bash || exit 1
    read -r pose_x pose_y pose_yaw <<< "$WORLD_POSE"
    exec "${nav2_command[@]}" "map:=${WORLD_MAP}" \
        "initial_pose_x:=${pose_x}" "initial_pose_y:=${pose_y}" "initial_pose_yaw:=${pose_yaw}" \
        "tf_prefix:=${ROBOTBASE_TF_PREFIX:-robot}"
fi

if [ "$room_mode" = true ]; then
    # Pair the room world with its exact map and the robot's centre spawn pose.
    cd -- "$WORKSPACE_ROOT" || exit 1
    source install/setup.bash
    echo "人位置Layer：横3.60 m × 縦2.70 mの部屋（外周壁のみ）"
    echo "Gazeboは koko_sim_room で起動してください。通常のNav2とは同時起動しないでください。"
    exec "${nav2_command[@]}" \
        "map:=${WORKSPACE_ROOT}/src/robotbase_sim/maps/room_360x270/room.yaml" \
        initial_pose_x:=1.8 initial_pose_y:=1.35 \
        tf_prefix:="${ROBOTBASE_TF_PREFIX:-robot}"
fi

map_paths=()
map_labels=()

build_map_list() {
    local map_file
    map_paths=()
    map_labels=()

    if [ -n "$WORLD_MAP" ] && [ -f "$WORLD_MAP" ]; then
        map_paths+=("$WORLD_MAP")
        map_labels+=("$(basename -- "$(dirname -- "$WORLD_MAP")") / $(basename -- "$WORLD_MAP") [選択ワールド対応・既定]")
    fi

    if [ -f "$BUILTIN_MAP" ] && [ "$BUILTIN_MAP" != "$WORLD_MAP" ]; then
        map_paths+=("$BUILTIN_MAP")
        map_labels+=("test_arena [同梱シミュレーション地図]")
    fi

    mkdir -p -- "$SAVED_MAPS_DIR"
    shopt -s nullglob
    for map_file in "$SAVED_MAPS_DIR"/*.yaml "$SAVED_MAPS_DIR"/*.yml; do
        map_paths+=("$map_file")
        map_labels+=("$(basename -- "$map_file" | sed -E 's/\.(yaml|yml)$//') [保存済み]")
    done
    shopt -u nullglob
}

print_map_list() {
    local index
    echo "========================================="
    echo "  シミュレーション用地図一覧"
    echo "========================================="
    for index in "${!map_paths[@]}"; do
        echo "  [$((index + 1))] ${map_labels[$index]}"
    done
    echo "========================================="
}

select_map() {
    local selection

    build_map_list
    if [ "${#map_paths[@]}" -eq 0 ]; then
        echo "エラー: 選択可能な地図がありません" >&2
        return 1
    fi

    print_map_list
    read -r -p "番号を入力してください (1-${#map_paths[@]}) [1]: " selection || return 1
    selection="${selection:-1}"
    if [[ ! "$selection" =~ ^[0-9]+$ ]] ||
       [ "$selection" -lt 1 ] || [ "$selection" -gt "${#map_paths[@]}" ]; then
        echo "エラー: 無効な選択です"
        return 1
    fi

    selected_map="${map_paths[$((selection - 1))]}"
    selected_label="${map_labels[$((selection - 1))]}"
    return 0
}

cd -- "$WORKSPACE_ROOT" || exit 1
# shellcheck disable=SC1091
source install/setup.bash

if [ "${1:-}" = "--list" ]; then
    build_map_list
    print_map_list
    exit 0
fi

while :; do
    echo ""
    read -r -p "Press [Enter] key to select a simulation map..." || break
    selected_map=""
    selected_label=""

    if ! select_map; then
        continue
    fi

    echo "選択された地図: $selected_label"
    echo "パス: $selected_map"
    echo ""
    echo "Gazebo（koko_sim）を先に起動しておいてください。"
    echo "通常のNav2と人位置Layer付きNav2は、どちらか一方だけ起動してください。"

    pose_args=()
    if [ "$selected_map" = "$WORLD_MAP" ]; then
        read -r pose_x pose_y pose_yaw <<< "$WORLD_POSE"
        pose_args=("initial_pose_x:=${pose_x}" "initial_pose_y:=${pose_y}" "initial_pose_yaw:=${pose_yaw}")
    fi
    "${nav2_command[@]}" "${pose_args[@]}" \
        map:="$selected_map" \
        tf_prefix:="${ROBOTBASE_TF_PREFIX:-robot}"
done

