#!/bin/bash

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" 1>/dev/null 2>&1 && pwd)"

"${script_dir}/install_packages.sh"
"${script_dir}/setup_network.sh"
"${script_dir}/install_vpn_connection_service.sh"
"${script_dir}/install_can_board_driver.sh"
mv "${script_dir}/dotbash_aliases.sh" "${HOME}/.bash_aliases"
