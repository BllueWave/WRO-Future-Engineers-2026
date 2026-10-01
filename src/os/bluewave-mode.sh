#!/bin/sh
# /boot/firmware/bluewave.mode holds "dev" or "race" (set from the laptop with `bw mode race`, or by editing the file
# on the card from any PC).  race: every radio is blocked BEFORE the race program starts (WRO FE 11.10).
MODE=$(tr -d '[:space:]' < /boot/firmware/bluewave.mode 2>/dev/null || echo dev)
if [ "$MODE" = race ]; then
    rfkill block all
    systemctl start bluewave-race
else
    rfkill unblock wifi
    systemctl start bluewave-agent
fi
