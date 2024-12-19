#!/bin/bash
sudo ip route add default via 192.168.1.1 dev eno1

NETWORK_ID="52b337794f4302e5"

sudo zerotier-cli join "${NETWORK_ID}"
