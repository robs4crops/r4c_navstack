#!/usr/bin/env bash

# Create a backup of the file /etc/hosts, with original content.
[ ! -e /etc/hosts.bak ] && sudo cp /etc/hosts /etc/hosts.bak

# Restore the backup of the file /etc/hosts before modifying it.
sudo cp /etc/hosts.bak /etc/hosts

# Delete all network connections present in the machine.

# Obtain network connection's names and types
# Each result <conn_name>:<conn_type> is separated by new lines.
NETWORK_CONNECTIONS="$(nmcli -terse -fields name,type con)"

# While loop + IFS + read command to read over a list of values separated by new lines.
while IFS= read -r CONNECTION; do
    # Print the current connection being processed
    # echo "Connection: ${CONNECTION}"
    # Process the result <conn_name>:<conn_type> and get the <conn_type>
    CONNECTION_TYPE="$(echo "${CONNECTION}" | cut -d':' -f2)"
    # Remove ethernet connections.
    if [ "${CONNECTION_TYPE}" = "802-3-ethernet" ]
    then
        # Get the connection's name
        CONNECTION_NAME="$(echo "${CONNECTION}" | cut -d':' -f1)"
        echo "Remove connection ${CONNECTION_NAME}"
        sudo nmcli connection delete "${CONNECTION_NAME}"
    fi
done <<< "${NETWORK_CONNECTIONS}"

# *************************************

# ROUTER_CONN_NAME=router
# ROUTER_NET_DEV=enp1s0 # Poe 1
# ROUTER_NET_DEV_MAC=d0:4c:c1:04:b9:62
# ROUTER_CONN_IP=192.168.1.20/24
# ROUTER_CONN_GW=192.168.1.1
# ROUTER_CONN_DNS=8.8.8.8

# sudo nmcli connection add type ethernet con-name ${ROUTER_CONN_NAME} ifname ${ROUTER_NET_DEV} mac ${ROUTER_NET_DEV_MAC} ip4 ${ROUTER_CONN_IP} gw4 ${ROUTER_CONN_GW} autoconnect yes save yes -- ipv4.dns ${ROUTER_CONN_DNS}

echo "192.168.1.1 ROUTER" | sudo tee -a /etc/hosts

# *************************************

# LIVOX_LIDAR_CONN_NAME=livox_lidar
# LIVOX_LIDAR_NET_DEV=enp2s0 # Poe 2
# LIVOX_LIDAR_NET_DEV_MAC=d0:4c:c1:04:b9:63
# LIVOX_LIDAR_CONN_IP=192.168.2.20/24

# sudo nmcli connection add type ethernet con-name ${LIVOX_LIDAR_CONN_NAME} ifname ${LIVOX_LIDAR_NET_DEV} mac ${LIVOX_LIDAR_NET_DEV_MAC} ip4 ${LIVOX_LIDAR_CONN_IP} autoconnect yes save yes

# echo "192.168.2.54 LIVOX_LIDAR"  | sudo tee -a /etc/hosts

# *************************************

# RS_LIDAR_CONN_NAME=rs_lidar
# RS_LIDAR_NET_DEV=enp3s0 # Poe 3
# RS_LIDAR_NET_DEV_MAC=d0:4c:c1:04:b9:64
# RS_LIDAR_CONN_IP=192.168.3.20/24

# sudo nmcli connection add type ethernet con-name ${RS_LIDAR_CONN_NAME} ifname ${RS_LIDAR_NET_DEV} mac ${RS_LIDAR_NET_DEV_MAC} ip4 ${RS_LIDAR_CONN_IP} autoconnect yes save yes

echo "192.168.1.20 RS_LIDAR"  | sudo tee -a /etc/hosts

# *************************************

# LUXONIS_CAMERA_CONN_NAME=luxonis_camera
# LUXONIS_CAMERA_NET_DEV=enp4s0 # Poe 4
# LUXONIS_CAMERA_NET_DEV_MAC=d0:4c:c1:04:b9:65
# LUXONIS_CAMERA_CONN_IP=192.168.4.20/24

# sudo nmcli connection add type ethernet con-name ${LUXONIS_CAMERA_CONN_NAME} ifname ${LUXONIS_CAMERA_NET_DEV} mac ${LUXONIS_CAMERA_NET_DEV_MAC} ip4 ${LUXONIS_CAMERA_CONN_IP} autoconnect yes save yes

# echo "192.168.4.54 LUXONIS_CAMERA"  | sudo tee -a /etc/hosts

# *************************************

printf "==> Restart NetworkManager.\n"

sudo systemctl restart network-manager

printf "==> Check network connections.\n"

# Show network devices along with their associated ip addresses.
ip address show

sleep_time=2

printf "==> Sleep ${sleep_time} until the network connections get ready\n"
sleep ${sleep_time}

# Check which network connections are active.
nmcli connection show

# *************************************

# Create ssh key

# ed25519 key is more secured than rsa, and also is shorter.
# -q silence ssh-keygen
# -t type
# -C comment
# -N pass void passphrase
# -f filename
# <<< $'\ny' answer yes by default

# ref: https://stackoverflow.com/a/43235320
ssh-keygen -q -t ed25519 -C "${USER}@eurecat.org" -N '' -f "${HOME}/.ssh/id_ed25519" <<< $'\ny' >/dev/null 2>&1
# Alternative way to do the same as above.
# yes | ssh-keygen -q -t ed25519 -C "${USER}@eurecat.org" -f "${HOME}/.ssh/id_ed25519" -P ""
# *************************************

# Add IPS from the ZERO TIER VPN into the file /etc/hosts
echo "192.168.1.100 CL-FFALLICA" | sudo tee -a /etc/hosts
echo "192.168.1.101 CL-JRASCON"  | sudo tee -a /etc/hosts
echo "192.168.1.102 CL-MSANCHES" | sudo tee -a /etc/hosts
echo "192.168.1.103 CL-PREVERTE" | sudo tee -a /etc/hosts
echo "192.168.1.104 CL-XRUIZ"    | sudo tee -a /etc/hosts

echo "10.147.17.60  EUT-FFALLICA" | sudo tee -a /etc/hosts
echo "10.147.17.34  EUT-JRASCON"  | sudo tee -a /etc/hosts
echo "10.147.17.175 EUT-MSANCHES" | sudo tee -a /etc/hosts
echo "10.147.17.87  EUT-PREVERTE" | sudo tee -a /etc/hosts
echo "10.147.17.224 EUT-XRUIZ"    | sudo tee -a /etc/hosts
echo "10.147.17.52 FC"            | sudo tee -a /etc/hosts
