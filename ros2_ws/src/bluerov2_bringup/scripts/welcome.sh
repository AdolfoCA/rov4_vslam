# BlueROV2 mission panel: printed when an interactive shell opens in the container.
#
# Sourced from ~/.bashrc (see the Dockerfile). It lives in the bind-mounted source, so
# edits on the host show up in the next shell without rebuilding anything.
#   rov_help              print it again
#   help_me               per sensor: check it, fix it, start it alone (rov_start)
#   ROV_WELCOME=0 bash    a shell without it
#
# The status line is live: each vehicle address gets one ping (1 s timeout, all in
# parallel), so opening a shell with the tether unplugged costs about one second.

# _rov_blue TEXT ROW ROWS [COL0 WIDTH]: TEXT coloured character by character, light cyan
# on the left to deep blue on the right, a little darker on each lower row. Truecolor
# (24-bit) escapes; COL0/WIDTH place a piece of text inside a wider gradient.
_rov_blue() {
    local text=$1 row=$2 rows=$3 col0=${4:-0} width=${5:-${#1}} out="" piece k t dim
    (( width < 2 )) && width=2
    dim=$(( 100 - 35 * row / (rows > 1 ? rows - 1 : 1) ))   # 100% top row -> 65% bottom
    for (( k = 0; k < ${#text}; k++ )); do
        t=$(( 1000 * (col0 + k) / (width - 1) ))             # 0..1000 across the width
        (( t > 1000 )) && t=1000
        # (90, 225, 255) light cyan -> (20, 60, 210) deep blue
        printf -v piece '\e[38;2;%d;%d;%dm%s' \
            $(( (90 + (20 - 90) * t / 1000) * dim / 100 )) \
            $(( (225 + (60 - 225) * t / 1000) * dim / 100 )) \
            $(( (255 + (210 - 255) * t / 1000) * dim / 100 )) "${text:k:1}"
        out+=$piece
    done
    printf '%s' "$out"
}

rov_help() {
    local rov_ip=${ROV_IP:-192.168.2.2} dvl_ip=${DVL_IP:-192.168.2.128} cam_ip=10.42.0.5

    local r=$'\e[0m' b=$'\e[1m' d=$'\e[2m'
    local cmd=$'\e[1;38;5;159m' hd=$'\e[1;38;5;39m' warn=$'\e[38;5;221m'
    local ok=$'\e[38;5;48m' bad=$'\e[38;5;203m'
    local sep="  \e[38;5;24m-------------------------------------------------------------------${r}\n"

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
        printf '  %s%s\n' "$(_rov_blue "${banner[i]}" "$i" "${#banner[@]}")" "$r"
    done
    printf '  %s%s  %sM I S S I O N   P A N E L%s  %s%s\n' \
        "$(_rov_blue '≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈' 6 7 0 67)" "$r" "${b}"$'\e[38;5;39m' "$r" \
        "$(_rov_blue '≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈' 6 7 50 67)" "$r"
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
    printf '  %srov_help%s show this again · %shelp_me%s a sensor did not start · %sREADME.md%s for details\n\n' \
        "$cmd" "$r" "$cmd" "$r" "$cmd" "$r"

    rm -rf "$tmp"
    unset -f _rov_dot
}

# --- help_me: what to do when one sensor did not come up ---------------------------------
# Two different failures, two different fixes:
#   - the node runs but publishes nothing  -> the device is not sending (fix upstream)
#   - the node is not running at all       -> rov_start <sensor>, in a new shell
rov_help_me() {
    local r=$'\e[0m' b=$'\e[1m' d=$'\e[2m'
    local cmd=$'\e[1;38;5;159m' hd=$'\e[1;38;5;39m' warn=$'\e[38;5;221m'
    local sep="  \e[38;5;24m-------------------------------------------------------------------${r}\n"
    local rov_ip=${ROV_IP:-192.168.2.2} dvl_ip=${DVL_IP:-192.168.2.128}

    # Sensor header, then: check / node runs but no data / node not running.
    local H="  ${hd}%s${r}  ${d}%s${r}\n"
    local C="    ${b}check  ${r} ${cmd}%s${r}\n"
    local D="    ${b}no data${r} ${cmd}%s${r}\n"
    local M="    ${b}no node${r} ${cmd}%s${r}\n"
    local N="            ${d}%s${r}\n"

    echo
    printf '  %sA sensor did not start%s\n' "$b" "$r"
    printf "$N" "1. check: is the topic publishing?  2. is the node running?  pgrep -af __node:="
    printf "$N" "   node there, no data → 'no data' line;  node missing → 'no node' line"
    printf "$N" "run rov_start in a NEW shell; it keeps running (Ctrl-C stops that sensor only)"
    printf "$sep"
    printf "$H" "IMU" "node /bluerov2/imu_node · ≥50 Hz"
    printf "$C" "ros2 topic hz /bluerov2/imu/data_raw"
    printf "$D" "ping $rov_ip"
    printf "$N" "BlueOS → MAVLink endpoints: a udpout to 192.168.2.1:14551 must exist"
    printf "$M" "rov_start imu"
    printf "$sep"
    printf "$H" "DVL" "node /bluerov2/dvl_node · ≥1 Hz"
    printf "$C" "ros2 topic hz /bluerov2/dvl/report"
    printf "$D" "ping $dvl_ip && nc -vz $dvl_ip 16171"
    printf "$N" "dvl/report alive but no velocity = no bottom lock (out of range, soft mud)"
    printf "$M" "rov_start dvl"
    printf "$sep"
    printf "$H" "Nose camera" "node /bluerov2/camera_node + nose_video_relay · ≥10 Hz"
    printf "$C" "ros2 topic hz /bluerov2/camera/camera_info"
    printf "$D" "ping $rov_ip · close QGroundControl if it was started before the drivers"
    printf "$M" "rov_start camera"
    printf "$sep"
    printf "$H" "Multi-cameras" "node /bluerov2/multicam/<camera> · ≥10 Hz each"
    printf "$N" "log says 'No frames for 5.0 s' = the camera computer is not streaming it"
    printf "$D" "multicam status        (which cameras are in the 'streaming' list)"
    printf "$N" "service not found → ping -c3 10.42.0.5 (ttl=64), camera computer still booting"
    local cam port
    for cam in aux_left:5700 aux_right:5701 stereo_bottom:5702 bottom_most:5703; do
        port=${cam#*:} cam=${cam%%:*}
        printf "    ${hd}%-13s${r} ${d}UDP %s${r}\n" "$cam" "$port"
        printf "      ${b}check  ${r} ${cmd}%s${r}\n" "ros2 topic hz /bluerov2/multicam/$cam/camera_info"
        printf "      ${b}no data${r} ${cmd}%s${r}\n" "multicam start $cam"
        printf "      ${b}no node${r} ${cmd}%s${r}\n" "rov_start $cam   then   multicam start $cam"
    done
    printf "$N" "top pair at once: multicam start · all four: multicam start all"
    printf "$sep"
    printf "$H" "Lights" "multi-camera light controller"
    printf "    ${b}set    ${r} ${cmd}%s${r}\n" "multicam lights 50        (0-100; activates the controller if needed)"
    printf "$sep"
    printf "$H" "Static TF" "node /static_tf_node · sensor extrinsics on /tf_static"
    printf "$C" "ros2 topic echo --once /tf_static"
    printf "$M" "rov_start tf"
    printf "$sep"
    printf "$H" "Recording control" "node /bluerov2/recording_manager · Foxglove Record tab"
    printf "$M" "rov_start recording"
    printf "$sep"
    printf "  %swarn:%s rov_start refuses a sensor whose node is already running: a second copy\n" "$warn" "$r"
    printf "  %s     would fight the first for its UDP port. Fix the 'no data' cause instead.%s\n\n" "$warn" "$r"
}
alias help_me=rov_help_me

# rov_start SENSOR: bluerov2.launch.py with only that sensor switched on.
rov_start() {
    local sensor=$1 node
    # Every switch of bluerov2.launch.py that defaults to true, set false; then the
    # chosen one(s) back on.
    local off=(imu:=false dvl:=false dead_reckoning:=false dvl_health:=false dvl_calibration:=false
               camera:=false recording_control:=false static_tf:=false
               aux_left:=false aux_right:=false stereo_bottom:=false bottom_most:=false)
    local on
    case $sensor in
        imu)        node=imu_node;           on=(imu:=true) ;;
        dvl)        node=dvl_node;           on=(dvl:=true dvl_health:=true dvl_calibration:=true) ;;
        camera)     node=camera_node;        on=(camera:=true) ;;
        aux_left|aux_right|stereo_bottom|bottom_most)
                    node=$sensor;            on=("$sensor:=true") ;;
        tf)         node=static_tf_node;     on=(static_tf:=true) ;;
        recording)  node=recording_manager;  on=(recording_control:=true) ;;
        *)
            echo "usage: rov_start imu|dvl|camera|aux_left|aux_right|stereo_bottom|bottom_most|tf|recording"
            echo "help_me explains when to use it."
            return 2 ;;
    esac
    # Look for the process, not the ROS graph: `ros2 node list` can miss a running node
    # or list one that has stopped. Every driver runs in this container, and launch
    # names each one with `-r __node:=<name>`.
    if pgrep -f -- "__node:=$node( |$)" >/dev/null; then
        echo "$node is already running - not starting a second copy."
        echo "If it has no data, the device is not sending: see help_me."
        return 1
    fi
    echo "+ ros2 launch bluerov2_bringup bluerov2.launch.py ${off[*]} ${on[*]}"
    ros2 launch bluerov2_bringup bluerov2.launch.py "${off[@]}" "${on[@]}"
}

# Interactive shells only (scp, `docker compose exec bluerov2 <cmd>` stay quiet).
if [[ $- == *i* && "${ROV_WELCOME:-1}" != 0 ]]; then
    rov_help
fi
