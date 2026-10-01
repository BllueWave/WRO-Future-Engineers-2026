#!/usr/bin/env bash
# BlueWave on the STOCK Hiwonder card -- the quick route: no new card, no flashing.  Run it from the laptop over SSH:
#     ssh pi@<robot> 'bash -s' < mentorpi/os/install_on_stock.sh
#
# Measured on this robot (2026-09-22): Hiwonder's start_node.service runs `ros2 launch bringup` inside the Docker
# container MentorPi (net=host, privileged, restart=always).  That launch holds the RRC board (/dev/ttyACM0) and the
# LD19 lidar (/dev/ttyUSB0), and it is the ONLY way the Angstrong depth camera (3482:6723) is read: it has no
# /dev/video node, only its vendor driver (ascamera) in the container.  So:
#   * start_node.service is disabled -> the board and the lidar are free for the agent;
#   * bluewave-camera.service runs ONLY the camera + web_video_server in the container: the agent reads the colour
#     image as MJPEG from http://127.0.0.1:8080 (net=host), no new driver; and bluewave/depth_bridge.py (copied in
#     from ~/bluewave by `bw deploy`) serves the raw depth on 127.0.0.1:8091 -- it subscribes only while the agent is
#     connected (params depth.on = 1), so with depth off it costs nothing.  The bridge file is copied into the
#     container by bluewave-camera's ExecStartPre (from ~/bluewave, where `bw deploy` puts it): a changed
#     depth_bridge.py needs `bw sys restart_camera` (bw deploy says so when it changed);
#   * bluewave-agent.service runs the agent + console on :8000, and after it stops -- whatever stopped it -- zeroes
#     the WLtoys motor's PWM (ExecStopPost; a no-op on the rrc drive).
# Everything it changes is appended to ~/bluewave/REVERT.sh: `bash ~/bluewave/REVERT.sh` = the car as Hiwonder shipped it.
set -euo pipefail
U=$(whoami); H=$HOME
mkdir -p "$H/bluewave/runs" "$H/bluewave/programs"
say() { printf '\n== %s\n' "$*"; }
REV="$H/bluewave/REVERT.sh"
[ -f "$REV" ] || printf '#!/bin/sh\n# puts back what BlueWave changed on this card\n' > "$REV"
rev() { grep -qxF "$1" "$REV" || echo "$1" >> "$REV"; }

say "Hiwonder's stack off (the board and the lidar become ours)"
if systemctl is-enabled start_node.service >/dev/null 2>&1; then
    sudo systemctl disable --now start_node.service || true
fi
rev "sudo systemctl disable --now bluewave-agent bluewave-camera 2>/dev/null; sudo systemctl enable --now start_node.service"
docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "~/.stop_ros.sh" >/dev/null 2>&1 || true
sleep 2
echo "serial ports still held:"; sudo fuser -v /dev/ttyACM0 /dev/ttyUSB0 2>&1 | tail -n +2 || echo "  none -- free"

say "camera only: ascamera + web_video_server in the container"
sudo tee /etc/systemd/system/bluewave-camera.service >/dev/null <<EOF
[Unit]
Description=BlueWave camera: Angstrong ascamera + web_video_server (MJPEG on :8080) + the depth bridge (:8091) in Hiwonder's container
Requires=docker.service
After=docker.service

[Service]
User=$U
# "-": ~/.stop_ros.sh kills every process matching "ros", ITSELF included, so it always exits 137.  The depth bridge's
# name does not match "ros": it is stopped by name.  A missing bridge file only costs the depth, never the colour.
ExecStartPre=-/usr/bin/docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "~/.stop_ros.sh"
ExecStartPre=-/usr/bin/docker exec MentorPi pkill -f bw_depth_bridge
ExecStartPre=-/usr/bin/docker cp $H/bluewave/bluewave/depth_bridge.py MentorPi:/tmp/bw_depth_bridge.py
ExecStart=/usr/bin/docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "source ~/.zshrc; ros2 launch peripherals depth_camera.launch.py & ros2 run web_video_server web_video_server & python3 /tmp/bw_depth_bridge.py; wait"
ExecStop=-/usr/bin/docker exec MentorPi pkill -f bw_depth_bridge
ExecStop=-/usr/bin/docker exec -u ubuntu -w /home/ubuntu MentorPi /bin/zsh -c "~/.stop_ros.sh"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

say "agent venv (numpy / opencv / serial from the system, fastapi / uvicorn from pip)"
[ -d "$H/bw-venv" ] || python3 -m venv --system-site-packages "$H/bw-venv"
"$H/bw-venv/bin/pip" install -q --disable-pip-version-check fastapi "uvicorn[standard]" python-multipart
"$H/bw-venv/bin/python" -c "import fastapi, uvicorn, numpy, cv2, serial; print('fastapi', fastapi.__version__, '| numpy', numpy.__version__, '| cv2', cv2.__version__)"

say "agent service (port 8000)"
sudo tee /etc/systemd/system/bluewave-agent.service >/dev/null <<EOF
[Unit]
Description=BlueWave dev agent (console + API on :8000)
After=network-online.target bluewave-camera.service
Wants=network-online.target

[Service]
User=$U
WorkingDirectory=$H/bluewave
ExecStart=$H/bw-venv/bin/uvicorn bluewave.agent:app --host 0.0.0.0 --port 8000 --log-level warning
# the WLtoys build: a hardware PWM keeps driving after its program dies -- duty 0 whatever happened (a hang, SIGKILL,
# bw deploy's restart; no-op on the rrc drive).  The guardian process is in the same cgroup and dies with the agent
ExecStopPost=$H/bw-venv/bin/python -m bluewave.motor --off
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
SUD=$(mktemp)                          # the console's system actions too (os/console_sudoers.sh); checked first
echo "$U ALL=(root) NOPASSWD: /usr/bin/systemctl restart bluewave-agent, /usr/bin/systemctl start bluewave-agent, /usr/bin/systemctl stop bluewave-agent, /usr/bin/systemctl restart bluewave-camera, /usr/sbin/reboot, /usr/sbin/poweroff, /usr/bin/date -u -s @*" > "$SUD"
sudo visudo -cf "$SUD" && sudo install -m 440 "$SUD" /etc/sudoers.d/bluewave-console    # never setup's `bluewave`
rm -f "$SUD"
sudo systemctl daemon-reload
sudo systemctl enable --now bluewave-camera >/dev/null 2>&1
sudo systemctl enable bluewave-agent >/dev/null 2>&1
chmod +x "$REV"
echo; echo "BW_INSTALL_OK"
