#!/bin/bash

# Check the script setup_network.sh to get the list of network devices associated with hardware component.
sudo ptpd --masteronly --interface enp2s0
# -C or --foreground: Don't run in background
