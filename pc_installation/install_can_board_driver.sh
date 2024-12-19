#!/bin/bash

target_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/can_board/EMUC-B202" 1>/dev/null 2>&1 && pwd)"

# The driver has to be compiled with the same version of gcc that was used to compile the kernel for Ubuntu 20.04, i.e, gcc-9.
# Make sure we have gcc-9 installed and can-utils.
sudo apt-get update
sudo apt-get -o Dpkg::Options::="--force-all" dist-upgrade --yes
sudo apt-get install --yes gcc-9 can-utils

# Activate temporaly gcc-9 (by assigning it a huge priority)
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 1000

# Compile the driver.
cd "${target_dir}"
sudo make

# Activate to the gcc version that was active previously.
# 1. Assign gcc-9 a low priority so it is not the preferred option any longer.
# 2. Activate the automatic option, i.e., the gcc compiler with the highest priority.
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 1
sudo update-alternatives --auto gcc

# Configure the pc to launch the can driver for the embedded can board at boot up.
# We need a baudrate of 250 kbps in the Robs4crops project, so we need to set the proper option (6) in the bootexec/run_emucd file.
baudrate_option="$(awk -F' ' '/baudrate=/ {print $1}' "${target_dir}/bootexec/run_emucd" | cut -d= -f2)"
sed -i s/baudrate=${baudrate_option}/baudrate=6/g "${target_dir}/bootexec/run_emucd"
chmod +x "${target_dir}/bootexec/run_emucd"
chmod +x "${target_dir}/bootexec/add_2_boot.sh"
cd "${target_dir}/bootexec"
./add_2_boot.sh
