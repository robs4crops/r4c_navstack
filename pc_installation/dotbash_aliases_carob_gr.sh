#!/usr/bin/env bash

# Place this file in ${HOME} with name '.bash_aliases'

alias sync_livox='${HOME}/workspaces/r4c_ws/src/robs4crops/pc_installation/livox_ptp_time_sync.sh'

alias r4c-open='docker compose -f ${HOME}/workspaces/r4c_ws/src/robs4crops/docker/docker_compose_carob_gr.yml up -d'

alias r4c-stop='docker compose -f ${HOME}/workspaces/r4c_ws/src/robs4crops/docker/docker_compose_carob_gr.yml down'

alias r4c_exec='docker exec -it docker-robs4crops_humble-1 bash'
