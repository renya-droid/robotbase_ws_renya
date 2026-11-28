#!/bin/bash
set -u

mode="real"
navigation_mode="to_pose"
mode_selected=false
navigation_only=false
for argument in "$@"; do
    case "${argument}" in
        real|sim)
            if [ "${mode_selected}" = true ]; then
                echo "Usage: $0 [real|sim] [--navigation-only]"
                exit 2
            fi
            mode="${argument}"
            mode_selected=true
            ;;
        --navigation-only)
            navigation_only=true
            ;;
        *)
            echo "Usage: $0 [real|sim] [--navigation-only]"
            exit 2
            ;;
    esac
done
case "${mode}" in
    real) use_sim_time=false ;;
    sim) use_sim_time=true ;;
esac

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
waypoints_dir="${workspace_root}/maps_waypoints/waypoints"
mkdir -p "${waypoints_dir}"

mapfile -t waypoint_files < <(find "${waypoints_dir}" -maxdepth 1 -type f -name '*.yaml' | sort)
if [ "${#waypoint_files[@]}" -eq 0 ]; then
    echo "ウェイポイントがありません: ${waypoints_dir}"
    echo "先に koko_waypoint_save または koko_waypoint_save_sim で地点を保存してください。"
    exit 1
fi

echo "========================================="
echo " ココちゃん ウェイポイント選択 (${mode})"
echo "========================================="
for index in "${!waypoint_files[@]}"; do
    printf '  [%d] %s\n' "$((index + 1))" "$(basename "${waypoint_files[index]}")"
done
read -r -p "番号を入力 [1]: " selection
selection="${selection:-1}"
if ! [[ "${selection}" =~ ^[0-9]+$ ]] || \
   [ "${selection}" -lt 1 ] || [ "${selection}" -gt "${#waypoint_files[@]}" ]; then
    echo "無効な選択です: ${selection}"
    exit 1
fi
waypoint_file="${waypoint_files[$((selection - 1))]}"
arrival_args=()
arrival_map_file=""
arrival_waypoint_file=""
elevator_bank_file=""
case "$(basename "${waypoint_file}")" in
  # atc1 changes to atc4 inside the same follower process, so the arrival
  # parameters must be supplied before that map switch as well.
  atc1.yaml|atc4.yaml)
    arrival_map_file="${workspace_root}/maps_waypoints/maps/atc6.yaml"
    arrival_waypoint_file="${waypoints_dir}/atc6.yaml"
    if [ "${mode}" = real ]; then
        if [ -n "${ELEVATOR_PROFILE:-}" ] && [ "${ELEVATOR_PROFILE}" != nakanoshima ]; then
            echo "ATC4 → ATC6 には nakanoshima が必要です。制御GUIとランチャーを同じ環境にしてください" >&2
            exit 2
        fi
        export ELEVATOR_PROFILE=nakanoshima
    fi
    echo "操作 GUI の環境設定は nakanoshima を選択してください。退出完了後、atc6.yaml の1番から続行します。"
    ;;
  1018.yaml)
    arrival_map_file="${workspace_root}/maps_waypoints/maps/1121.yaml"
    arrival_waypoint_file="${waypoints_dir}/1121way.yaml"
    ;;
  1124ele3F.yaml)
    arrival_map_file="${workspace_root}/maps_waypoints/maps/1124ele.yaml"
    arrival_waypoint_file="${waypoints_dir}/1124ele.yaml"
    echo "操作 GUI の環境設定は 1124ele3F_to_1124ele を選択してください。"
    ;;
esac
# A new 1018_2 route uses the existing 1018 map. Its destination route is
# selected only from a validated bank profile, never from the legacy 1018 rule.
if [ "$(basename "${waypoint_file}")" = 1018_2.yaml ] && [ "${mode}" = real ]; then
    # This route is dedicated to the 5-cabin 1018 experiment. A standalone
    # launcher has no selected profile; choose the matching one for this route.
    if [ -z "${ELEVATOR_PROFILE:-}" ]; then
        export ELEVATOR_PROFILE=1018_bank_template
        echo "1018_2 用の環境設定: ${ELEVATOR_PROFILE}（制御GUIも同じ環境を選択してください）"
    elif [ "${ELEVATOR_PROFILE}" = current ]; then
        echo "1018_2 には 1018_bank_template が必要です。制御GUIとランチャーを同じ環境にして起動してください" >&2
        exit 2
    fi
    kuwa_root="$(dirname "${workspace_root}")/kuwa_ws2"
    if [ -f "${kuwa_root}/tools/configure.py" ]; then
        bank_config="$(KUWA_ROOT="${kuwa_root}" SELECTED_WAYPOINT="${waypoint_file}" ROBOT_WS="${workspace_root}" python3 - <<'BANK_CONFIG'
import os, sys
from pathlib import Path
root=Path(os.environ['KUWA_ROOT'])
sys.path.insert(0,str(root/'tools'))
from configure import profile_paths, resolve_profile_file, validate
import yaml
site=yaml.safe_load(profile_paths(root)['site.yaml'].read_text())
if site.get('elevator_mode') == 'bank':
    checked_site,cfg=validate(root)
    entry=cfg['elevator_entry_map_diff']['ros__parameters']
    if Path(checked_site['departure_waypoint_file']).resolve() != Path(os.environ['SELECTED_WAYPOINT']).resolve():
        raise ValueError('Selected waypoint does not match the validated bank departure route')
    expected_map=Path(os.environ['ROBOT_WS'])/'maps_waypoints/maps/1018.yaml'
    if Path(entry['map_yaml_path']).resolve() != expected_map.resolve():
        raise ValueError('1018_2 requires the 1018 map')
    print(entry['exit_map_yaml_path'])
    print(entry['elevator_bank_config'])
BANK_CONFIG
)" || exit 2
        if [ -n "${bank_config}" ]; then
            mapfile -t bank_values <<< "${bank_config}"
            arrival_map_file="${bank_values[0]}"
            elevator_bank_file="${bank_values[1]}"
            arrival_args=(-p "elevator_arrival_map_file:=${arrival_map_file}"
                          -p "elevator_bank_config:=${elevator_bank_file}")
            echo "13F到着後は選択号機の初期位置から 1126.yaml の経路へ進みます"
        else
            echo "1018_2 は検証済みの複数基プロファイルが必要です" >&2
            exit 2
        fi
    fi
fi
if [ -n "${arrival_map_file}" ] && [ "${#arrival_args[@]}" -eq 0 ]; then
    if [ ! -f "${arrival_map_file}" ] || [ ! -f "${arrival_waypoint_file}" ]; then
        echo "エレベーター到着側の地図または経路がありません: ${arrival_map_file} / ${arrival_waypoint_file}" >&2
        exit 1
    fi
    arrival_args=(-p "elevator_arrival_map_file:=${arrival_map_file}"
                  -p "elevator_arrival_waypoint_file:=${arrival_waypoint_file}")
fi

read -r -p "開始番号 [1]: " start_index
start_index="${start_index:-1}"
if ! [[ "${start_index}" =~ ^[0-9]+$ ]] || [ "${start_index}" -lt 1 ]; then
    echo "開始番号には1以上の整数を指定してください: ${start_index}" >&2
    exit 1
fi
echo "走行方式を選択："
echo "  1: NavigateToPose（従来方式）"
echo "  2: NavigateThroughPoses（連続走行）"
read -r -p "番号 [1]: " navigation_selection
case "${navigation_selection:-1}" in
    1) navigation_mode="to_pose" ;;
    2) navigation_mode="through_poses" ;;
    *)
        echo "無効な走行方式です: ${navigation_selection}" >&2
        exit 1
        ;;
esac
threshold="1.0"
threshold_args=()
if [ "$navigation_mode" = to_pose ]; then
    read -r -p "中間点の切替距離[m] [1.0]: " threshold
    threshold="${threshold:-1.0}"
    threshold_parameter="default_threshold"
else
    read -r -p "通常区間の切替距離[m] [6.0]: " threshold
    threshold="${threshold:-6.0}"
    threshold_parameter="section_switch_distance"
fi
if ! [[ "${threshold}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || \
   ! awk -v value="${threshold}" 'BEGIN { exit !(value > 0) }'; then
    echo "切替距離には0より大きい数値を指定してください: ${threshold}" >&2
    exit 1
fi
# ROS 2 infers `2` as INTEGER, but this parameter is declared as DOUBLE.
if [[ "${threshold}" =~ ^[0-9]+$ ]]; then
    threshold="${threshold}.0"
fi
threshold_args=(-p "${threshold_parameter}:=${threshold}")
read -r -p "無限ループしますか [y/N]: " loop_answer
loop_value=false
if [[ "${loop_answer:-}" =~ ^[yY]$ ]]; then
    loop_value=true
fi

echo "ウェイポイント: ${waypoint_file}"
if [ "${#arrival_args[@]}" -gt 0 ]; then
    echo "エレベーター退室後: ${arrival_map_file} / ${arrival_waypoint_file:-号機別bank経路}"
fi
echo "開始番号: ${start_index}, loop: ${loop_value}"
if [ "$navigation_mode" = through_poses ]; then
    echo "走行方式: NavigateThroughPoses（区間上限20 m、通常区間の切替距離: ${threshold} m）"
else
    echo "走行方式: NavigateToPose（中間点の切替距離: ${threshold} m）"
fi
if [ "${navigation_only}" = true ]; then
    echo "人物探索連携: OFF (Navigation-only)"
else
    echo "人物探索連携: ON"
fi

# The 1018_2 route must never drive towards the bank when the matching
# controller is absent. It also must stop at the first Nav2 failure instead
# of skipping several points and failing at the elevator waypoint.
failure_args=()
if [ -n "${elevator_bank_file}" ]; then
    if ! /usr/bin/python3 "${kuwa_root}/tools/check_live_controller.py" \
        "${ELEVATOR_PROFILE}" "${elevator_bank_file}"; then
        exit 2
    fi
    failure_args=(-p 'stop_on_failure:=true')
    echo 'エレベーター経路: Nav2失敗時は次の地点へ飛ばさず停止します'
fi

# setup.cfg installs console scripts under <package-prefix>/lib/robotbase_waypoint.
# Keep the sourced launcher/alias environment, but select this workspace's
# executable directly: ros2 run would introduce another parent process.
install_layout=isolated
if [ -f "${workspace_root}/install/.colcon_install_layout" ]; then
    IFS= read -r install_layout < "${workspace_root}/install/.colcon_install_layout" || true
fi
case "${install_layout}" in
    isolated) install_prefix="${workspace_root}/install/robotbase_waypoint" ;;
    merged) install_prefix="${workspace_root}/install" ;;
    *)
        echo "未対応の install layout: ${install_layout}" >&2
        exit 126
        ;;
esac
waypoint_executable="${install_prefix}/lib/robotbase_waypoint/waypoint_follow"
if [ ! -x "${waypoint_executable}" ]; then
    echo "waypoint_follow 実行ファイルがありません: ${waypoint_executable}" >&2
    echo "この workspace の robotbase_waypoint をビルドしてください。" >&2
    exit 127
fi

# Preserve this startup PID across the handoff so TERM/INT reach the follower
# itself and the launcher's existing PID snapshot remains valid after exec.
exec "${waypoint_executable}" --ros-args \
    -p navigation_mode:="${navigation_mode}" \
    -p use_sim_time:="${use_sim_time}" \
    -p waypoint_file:="${waypoint_file}" \
    "${arrival_args[@]}" \
    -p start_index:="${start_index}" \
    "${threshold_args[@]}" \
    -p loop:="${loop_value}" \
    -p navigation_only:="${navigation_only}" \
    "${failure_args[@]}" \
    -p base_frame:="${ROBOTBASE_TF_PREFIX:-robot}/base_footprint"
