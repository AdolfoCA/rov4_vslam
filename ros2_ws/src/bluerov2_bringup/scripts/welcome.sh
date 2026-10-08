# BlueROV2 mission panel: printed when an interactive shell opens in the container.
#
# Sourced from ~/.bashrc (see the Dockerfile). It lives in the bind-mounted source, so
# edits on the host show up in the next shell without rebuilding anything.
#   rov_help              print it again
#   ROV_WELCOME=0 bash    a shell without it
#
# The status line is live: each vehicle address gets one ping (1 s timeout, all in
# parallel), so opening a shell with the tether unplugged costs about one second.

rov_help() {
    local rov_ip=${ROV_IP:-192.168.2.2} dvl_ip=${DVL_IP:-192.168.2.128} cam_ip=10.42.0.5

    local r=$'\e[0m' b=$'\e[1m' d=$'\e[2m'
    local cmd=$'\e[1;38;5;159m' hd=$'\e[1;38;5;39m' warn=$'\e[38;5;221m'
    local ok=$'\e[38;5;48m' bad=$'\e[38;5;203m'
    local sep="  \e[38;5;24m-------------------------------------------------------------------${r}\n"
    local grad=(26 25 20 19 19 18)  # deep to navy blue, one per banner row

    # --- Live check: pings in parallel ---------------------------------------------------
    local tmp ip; tmp=$(mktemp -d)
    for ip in "$rov_ip" "$dvl_ip" "$cam_ip"; do
        ( ping -c1 -W1 "$ip" >/dev/null 2>&1 && touch "$tmp/$ip" ) &
    done
    wait
    _rov_dot() { [ -e "$tmp/$1" ] && printf '%s●%s' "$ok" "$r" || printf '%s○%s' "$bad" "$r"; }

    # --- Banner ----------------------------------------------------------------------------
    local banner=(
        "██████╗ ██╗     ██╗   ██╗███████╗██████╗  ██████╗ ██╗   ██╗██████╗"
        "██╔══██╗██║     ██║   ██║██╔════╝██╔══██╗██╔═══██╗██║   ██║╚════██╗"
        "██████╔╝██║     ██║   ██║█████╗  ██████╔╝██║   ██║██║   ██║ █████╔╝"
        "██╔══██╗██║     ██║   ██║██╔══╝  ██╔══██╗██║   ██║╚██╗ ██╔╝██╔═══╝"
        "██████╔╝███████╗╚██████╔╝███████╗██║  ██║╚██████╔╝ ╚████╔╝ ███████╗"
        "╚═════╝ ╚══════╝ ╚═════╝ ╚══════╝╚═╝  ╚═╝ ╚═════╝   ╚═══╝  ╚══════╝"
    )
    local i
    echo
    for i in "${!banner[@]}"; do
        printf '  \e[38;5;%sm%s%s\n' "${grad[i]}" "${banner[i]}" "$r"
    done
    printf '  \e[38;5;19m≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈%s  %sM I S S I O N   P A N E L%s  \e[38;5;18m≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈%s\n' \
        "$r" "${b}"$'\e[38;5;26m' "$r" "$r"
    printf "$sep"
    printf '  %sROS 2 sensor drivers for the BlueROV2 Heavy (IMU, DVL, cameras). No vehicle control.%s\n' "$d" "$r"
    printf '  %s ROV   %s DVL   %s Multi-camera\n' \
        "$(_rov_dot "$rov_ip")" "$(_rov_dot "$dvl_ip")" "$(_rov_dot "$cam_ip")"

    # --- Mission steps -------------------------------------------------------------------
    printf "$sep"
    local S="  ${hd}%s${r} ${b}%-12s${r} ${cmd}%s${r}\n"
    local N="                 ${d}%s${r}\n"
    printf "$S" 1 "Bring up" "ros2 launch bluerov2_bringup session_start.launch.py"
    printf "$N" "wait for SESSION READY (~2.5 min), keep this shell open"
    printf "$sep"
    printf "$S" 2 "Foxglove" "ros2 launch bluerov2_bringup foxglove.launch.py"
    printf "$N" "open connection → ws://localhost:8765"
    printf "$N" "first time: Layout → Import from file → ros2_ws/src/foxglove/rov_layout.json"
    printf "$sep"
    printf "$S" 3 "DVL calib" "ros2 run bluerov2_dvl dvl_calibrate --water-temp 12 --salinity 0"
    printf "                 ${warn}%s${r}\n" "at the surface, disarmed, still · sea water: --salinity 35"
    printf "$sep"
    # Where it lands: data/ (= ./data on the host), or the first external drive under the
    # host's /media, mounted at the same path (docker-compose.yaml).
    local drive out=/home/rosdev/data/dive01
    for drive in /media/*/*/; do
        [ -d "$drive" ] && { out=${drive%/}/dive01; break; }
    done
    printf "$S" 4 "Record" "ros2 launch bluerov2_bringup record.launch.py output_dir:=${out}"
    printf "$N" "output_dir = where the bag lands · Ctrl-C stops · or Foxglove → Record tab"
    printf "$sep"
    printf "$S" 5 "Stop" "ros2 launch bluerov2_bringup session_stop.launch.py"
    printf "                 ${warn}%s${r}\n" "unplug only after \"safe to unplug\""
    printf "$sep"
    printf '  %srov_help%s show this again · %sREADME.md%s for details\n\n' "$cmd" "$r" "$cmd" "$r"

    rm -rf "$tmp"
    unset -f _rov_dot
}

# Interactive shells only (scp, `docker compose exec bluerov2 <cmd>` stay quiet).
if [[ $- == *i* && "${ROV_WELCOME:-1}" != 0 ]]; then
    rov_help
fi
