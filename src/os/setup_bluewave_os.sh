#!/usr/bin/env bash
# BlueWave OS -- turns a FRESH Raspberry Pi OS Lite (64-bit, no desktop) card into the MentorPi's race system.
#
#   The stock Hiwonder card is NEVER touched: keep it as the reference / rescue card.  This goes on a SECOND card.
#   Flash it with Raspberry Pi Imager: "Raspberry Pi OS Lite (64-bit)", and in the Imager's settings set
#     hostname  bluewave        user  bluewave        SSH  on, public-key only (paste the laptop's ~/.ssh/id_ed25519.pub)
#     Wi-Fi     the TEAM network (the router / phone hotspot the laptop also joins -- see mentorpi/README.md)
#   Boot it on the robot, then from the laptop:   bw deploy-os   (or copy this folder and run it with sudo)
#
#   sudo bash setup_bluewave_os.sh            idempotent: running it twice changes nothing the second time
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
U=${BW_USER:-bluewave}
H=/home/$U
HERE=$(cd "$(dirname "$0")" && pwd)
say() { printf '\n== %s\n' "$*"; }

say "packages (numpy / opencv / serial from apt: prebuilt for arm64, no compile on the Pi)"
apt-get update -qq
apt-get install -y -qq python3-venv python3-pip python3-numpy python3-opencv python3-serial python3-lgpio \
    rfkill v4l-utils usbutils i2c-tools htop git
say "python venv for the agent (sees the apt numpy / opencv)"
sudo -u "$U" python3 -m venv --system-site-packages "$H/venv"
sudo -u "$U" "$H/venv/bin/pip" install -q fastapi "uvicorn[standard]" python-multipart
sudo -u "$U" mkdir -p "$H/bluewave/runs" "$H/bluewave/programs"

say "udev: /dev/rrc = the RRC Lite (CH9102 1a86:55d4), kept away from ModemManager"
install -m 644 "$HERE/99-bluewave.rules" /etc/udev/rules.d/99-bluewave.rules
usermod -aG dialout,video,gpio "$U" || true
udevadm control --reload-rules && udevadm trigger

say "services this car never needs (each one is boot time, CPU, or a port grabbed from us)"
for s in ModemManager bluetooth hciuart triggerhappy apt-daily.timer apt-daily-upgrade.timer man-db.timer \
         NetworkManager-wait-online; do
    systemctl disable --now "$s" 2>/dev/null || true
done

say "boot config: Pi 5 USB current for lidar + camera, Bluetooth off at the firmware, no splash"
CFG=/boot/firmware/config.txt
add() { grep -qxF "$1" "$CFG" || echo "$1" >> "$CFG"; }
add "# --- BlueWave ---"
add "usb_max_current_enable=1"
add "dtoverlay=disable-bt"
add "disable_splash=1"

say "CPU governor performance (a control loop wants no frequency ramp-up), journal in RAM (fewer SD writes)"
install -m 644 "$HERE/bluewave-perf.service" /etc/systemd/system/
mkdir -p /etc/systemd/journald.conf.d
printf '[Journal]\nStorage=volatile\nRuntimeMaxUse=24M\n' > /etc/systemd/journald.conf.d/bluewave.conf

say "the two modes: dev (Wi-Fi on, agent + console on :8000) and race (radios OFF, start button)"
install -m 644 "$HERE/bluewave-agent.service" "$HERE/bluewave-race.service" "$HERE/bluewave-mode.service" \
    /etc/systemd/system/
install -m 755 "$HERE/bluewave-mode.sh" /usr/local/bin/bluewave-mode.sh
[ -f /boot/firmware/bluewave.mode ] || echo dev > /boot/firmware/bluewave.mode
echo "$U ALL=(root) NOPASSWD: /usr/bin/systemctl restart bluewave-agent, /usr/bin/systemctl stop bluewave-agent, \
/usr/bin/systemctl start bluewave-agent, /usr/bin/tee /boot/firmware/bluewave.mode, /usr/sbin/rfkill, /usr/sbin/reboot" \
    > /etc/sudoers.d/bluewave && chmod 440 /etc/sudoers.d/bluewave
systemctl daemon-reload
systemctl enable bluewave-perf bluewave-mode
systemctl disable bluewave-agent bluewave-race 2>/dev/null || true     # bluewave-mode starts the right one

say "what is plugged in -- put the lidar's by-id path into params.lidar.device"
lsusb
ls -l /dev/serial/by-id/ 2>/dev/null || echo "(no /dev/serial/by-id yet)"
v4l2-ctl --list-devices 2>/dev/null || true
echo
echo "done.  reboot, then from the laptop:  bw deploy && bw status"
