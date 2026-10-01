#!/usr/bin/env bash
# The console's data as ROS 2 topics (bluewave/ros_bridge.py) + rosbridge on :9090, inside Hiwonder's container
# `MentorPi` on the STOCK card.  DEV ONLY -- never before a race (race mode has no container, and no radio).
#
#     ssh <user>@<robot> 'bash -s' -- start < mentorpi/os/ros_bridge.sh        (from the laptop; or stop | status | check)
#     bash ros_bridge.sh start | stop | status | check                           (a copy on the robot)
#
# start   copies the bridge into the container and starts it (read-only, passive: the link watchdog never counts
#         it) and rosbridge_websocket.  NEVER Hiwonder's bringup: it opens the serial ports the agent owns.
#         Then: Foxglove Studio on the laptop -> Open connection -> Rosbridge -> ws://<robot ip>:9090, add a 3D panel
#         (fixed frame "map": /map, /scan, /bluewave/pillars, /plan, the TF tree).  Or RViz in the container on the
#         stock card's VNC desktop (Fixed Frame "map").
# check   the one [DAY1] question: are rosbridge_server / foxglove_bridge in the container?
# Note    `bw sys restart_camera` runs ~/.stop_ros.sh, which kills EVERY process whose name holds "ros" -- the bridge
#         and rosbridge included: start them again after it.
set -euo pipefail
C=MentorPi
SRC="$HOME/bluewave/bluewave/ros_bridge.py"
RUN="docker exec -u ubuntu -w /home/ubuntu $C /bin/zsh -c"
say() { printf '== %s\n' "$*"; }
status() {
    for p in bw_ros_bridge rosbridge_websocket; do
        if docker exec "$C" pgrep -f "$p" >/dev/null 2>&1; then echo "$p  running"; else echo "$p  not running"; fi
    done
    docker exec "$C" tail -n 3 /tmp/bw_ros_bridge.log 2>/dev/null || true
}

case "${1:-status}" in
check)
    $RUN "source ~/.zshrc >/dev/null 2>&1; ros2 pkg list 2>/dev/null | grep -E '^(rosbridge_server|nav_msgs|tf2_msgs|visualization_msgs|foxglove_bridge)$'" \
        || echo "none of them listed: is the container running (docker ps)?"
    ;;
start)
    [ -f "$SRC" ] || { echo "no $SRC: bw deploy first"; exit 1; }
    docker cp "$SRC" "$C:/tmp/bw_ros_bridge.py"
    docker exec "$C" pkill -f bw_ros_bridge >/dev/null 2>&1 || true
    docker exec -d -u ubuntu -w /home/ubuntu "$C" /bin/zsh -c \
        "source ~/.zshrc; exec python3 /tmp/bw_ros_bridge.py --agent http://127.0.0.1:8000 > /tmp/bw_ros_bridge.log 2>&1"
    if ! docker exec "$C" pgrep -f rosbridge_websocket >/dev/null 2>&1; then
        docker exec -d -u ubuntu -w /home/ubuntu "$C" /bin/zsh -c \
            "source ~/.zshrc; exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/bw_rosbridge.log 2>&1"
    fi
    sleep 3
    status
    ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    say "Foxglove: Rosbridge -> ws://${ip:-<robot>}:9090   (logs: docker exec $C tail /tmp/bw_ros_bridge.log)"
    ;;
stop)
    docker exec "$C" pkill -f bw_ros_bridge >/dev/null 2>&1 || true
    docker exec "$C" pkill -f rosbridge_websocket >/dev/null 2>&1 || true
    docker exec "$C" pkill -f rosapi_node >/dev/null 2>&1 || true
    say "stopped"
    ;;
status)
    status
    ;;
*)
    echo "usage: ros_bridge.sh start | stop | status | check"; exit 2
    ;;
esac
