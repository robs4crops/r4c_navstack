#!/bin/bash

# Reference:
# https://linuxconfig.org/how-to-run-script-on-startup-on-ubuntu-20-04-focal-fossa-server-desktop

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" 1>/dev/null 2>&1 && pwd)"

[ -e /etc/systemd/system/livox_ptp_time_sync.service ] && sudo rm -f /etc/systemd/system/livox_ptp_time_sync.service
[ -e /usr/local/bin/livox_ptp_time_sync.sh ] && sudo rm -f /usr/local/bin/livox_ptp_time_sync.sh

sudo cp "${script_dir}/livox_ptp_time_sync.service" /etc/systemd/system
sudo cp "${script_dir}/livox_ptp_time_sync.sh" /usr/local/bin
sudo chmod 664 /etc/systemd/system/livox_ptp_time_sync.service
sudo chmod 744 /usr/local/bin/livox_ptp_time_sync.sh
sudo systemctl daemon-reload
sudo systemctl enable livox_ptp_time_sync.service
