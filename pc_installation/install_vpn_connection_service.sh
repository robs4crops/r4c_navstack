#!/bin/bash

# Reference:
# https://linuxconfig.org/how-to-run-script-on-startup-on-ubuntu-20-04-focal-fossa-server-desktop

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" 1>/dev/null 2>&1 && pwd)"

[ -e /etc/systemd/system/vpn_connection.service ] && sudo rm -f /etc/systemd/system/vpn_connection.service
[ -e /usr/local/bin/vpn_connection.sh ] && sudo rm -f /usr/local/bin/vpn_connection.sh

sudo cp "${script_dir}/vpn_connection.service" /etc/systemd/system
sudo cp "${script_dir}/vpn_connection.sh" /usr/local/bin
sudo chmod 664 /etc/systemd/system/vpn_connection.service
sudo chmod 744 /usr/local/bin/vpn_connection.sh
sudo systemctl daemon-reload
sudo systemctl enable vpn_connection.service
