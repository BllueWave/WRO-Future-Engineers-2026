#!/usr/bin/env bash
# The console's system actions (bluewave/sysinfo.py): restart the agent / camera, reboot, power off, set the clock.
# Needed on the BlueWave OS card only (the stock Hiwonder card's user already has passwordless sudo).  From Git Bash
# or cmd.exe on the laptop -- NOT PowerShell (it rejects `<`, and Get-Content piping adds CRLF, which breaks bash):
#     py -3 tools/bw.py ssh "sudo bash -s" < os/console_sudoers.sh
# Idempotent.  Its own file, /etc/sudoers.d/bluewave-console: /etc/sudoers.d/bluewave belongs to
# os/setup_bluewave_os.sh and holds the tee + rfkill rules race mode needs (overwriting it broke KEY2 back to dev).
# The file is checked with visudo BEFORE it is installed: a broken sudoers file locks sudo.
set -euo pipefail
U=${SUDO_USER:-$(whoami)}
T=$(mktemp)
echo "$U ALL=(root) NOPASSWD: /usr/bin/systemctl restart bluewave-agent, /usr/bin/systemctl start bluewave-agent, \
/usr/bin/systemctl stop bluewave-agent, /usr/bin/systemctl restart bluewave-camera, /usr/sbin/reboot, /usr/sbin/poweroff, \
/usr/bin/date -u -s @*" > "$T"
visudo -cf "$T" && install -m 440 "$T" /etc/sudoers.d/bluewave-console && echo BW_SUDOERS_OK
rm -f "$T"
