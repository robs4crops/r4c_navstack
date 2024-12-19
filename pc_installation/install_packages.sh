#!/bin/bash

# Copy a default .bashrc file into the ${HOME} folder.
#[ -e "${HOME}/.bashrc" ] && rm -f "${HOME}/.bashrc"
#cp -v /etc/skel/.bashrc "${HOME}"

echo "*********************************"
echo "* Install basic Ubuntu packages *"
echo "*********************************"

sudo apt-get update
# Install the apt-utils package first, to avoid warnings when installing packages if this package
# is not installed previously.
sudo apt-get install --yes --quiet --no-install-recommends apt-utils
sudo apt-get -o Dpkg::Options::="--force-all" dist-upgrade --yes
# Install the package that allow us to add repositories.
apt-get install --yes --quiet --no-install-recommends software-properties-common
# Now add-apt-repository is available
sudo add-apt-repository --yes universe
sudo apt-get update
sudo apt-get -o Dpkg::Options::="--force-all" dist-upgrade --yes
# 
sudo apt-get install --yes --quiet --no-install-recommends apt-rdepends \
apt-transport-https \
arp-scan \
automake \
bash-completion \
bindfs \
build-essential \
ca-certificates \
coreutils \
curl \
dirmngr \
gawk \
git \
gnupg \
gnupg2 \
gpg \
htop \
iproute2 \
iputils-ping \
less \
libcppunit-dev \
libpython3-dev \
libtool \
libtool-bin \
lsb-release \
man \
man-db \
manpages \
manpages-dev \
manpages-posix \
manpages-posix-dev \
mlocate \
nano \
net-tools \
network-manager \
p7zip-full \
p7zip-rar \
ppa-purge \
ptpd \
python3 \
python3-dev \
python3-pip \
python3-argcomplete \
python3-flake8 \
python3-mock \
python3-numpy \
python3-pygraphviz \
python3-requests \
python3-setuptools \
python3-pytest \
python3-pytest-mock \
python3-wheel \
rsync \
sed \
ssh \
sshfs \
sudo \
tmux \
udev \
wget \
xclip \
xsel \
unzip \
zip

echo "Etc/UTC" > sudo tee /etc/timezone
sudo ln -sf "/usr/share/zoneinfo/Etc/UTC" /etc/localtime
sudo apt-get install --yes --quiet --no-install-recommends tzdata

sudo apt-get install --yes --quiet --no-install-recommends locales
sudo sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && sudo locale-gen

git config --global user.name "eutrob"
git config --global user.email "eutrob@eurecat.org"

# For modern ubuntu oses, i.e., from 18.04 onwards, we can consider python3 command as the default
# python command.
update-alternatives --install /usr/bin/python python /usr/bin/python3 100

echo "*********************************"
echo "* Install extra Ubuntu packages *"
echo "*********************************"

sudo apt-get install --yes --quiet --no-install-recommends exfat-fuse \
geany \
ghex \
gparted \
meld \
synaptic \
terminator \

echo "********************************************"
echo "* Install project-specific Ubuntu packages *"
echo "********************************************"

sudo apt-get install --yes --quiet --no-install-recommends can-utils

echo "******************"
echo "* Install NeoVim *"
echo "******************"

# neovim instead of vim, but basically they are the same for most purposes.
add-apt-repository ppa:neovim-ppa/stable --yes
apt-get update
apt-get -o Dpkg::Options::="--force-all" dist-upgrade --yes
apt-get install --yes --quiet --no-install-recommends neovim neovim-runtime python3-neovim
update-alternatives --install /usr/bin/vi     vi     /usr/bin/nvim 100
update-alternatives --install /usr/bin/vim    vim    /usr/bin/nvim 100
update-alternatives --install /usr/bin/editor editor /usr/bin/nvim 100

echo "*********************************************"
echo "* Install bat, exa and ripgrep applications *"
echo "*********************************************"

# The command bat is a better version of the cat command.
# Install bat from the package manager if available, otherwise download it from github and install it.
bat_in_package_manager="$(apt-cache search --names-only '^bat$')"

if [ -n "${bat_in_package_manager}" ]
then
  apt-get install --yes --quiet --no-install-recommends bat
  # If you install bat this way, the executable will be installed as batcat instead of bat, because
  # there is already another command in the package manager, older, that when installed has the name
  # bat. So, we use a symbolic link to have the command bat.
  ln -sv /usr/bin/batcat /usr/bin/bat
else
  bat_version=$(curl -s "https://api.github.com/repos/sharkdp/bat/releases/latest" | grep -Po '"tag_name": "v\K[0-9.]+') # we get something like num1.num2.num3

  if [ -n "${bat_version}" ]
  then
    echo "Install bat version: ${bat_version}"
    wget -O /tmp/bat.deb "https://github.com/sharkdp/bat/releases/latest/download/bat-musl_${bat_version}_amd64.deb"
    chmod +x /tmp/bat.deb
    dpkg -i /tmp/bat.deb
    rm -f /tmp/bat.deb
  fi
fi

# The command exa is a better version of ls command.
# Install exa from the package manager if available, otherwise download it from github and install it.
exa_in_package_manager="$(apt-cache search --names-only '^exa$')"

if [ -n "${exa_in_package_manager}" ]
then
  apt-get install --yes --quiet --no-install-recommends exa
else
  exa_version=$(curl -s "https://api.github.com/repos/ogham/exa/releases/latest" | grep -Po '"tag_name": "v\K[0-9.]+')

  if [ -n "${exa_version}" ]
  then
    echo "Install exa version: ${exa_version}"
    wget -O /tmp/exa.zip "https://github.com/ogham/exa/releases/latest/download/exa-linux-x86_64-musl-v${exa_version}.zip"
    unzip -q /tmp/exa.zip bin/exa -d /usr
    rm -f /tmp/exa.zip
  fi
fi

# The command ripgrep is a better version of grep command.
# Install ripgrep from the package manager if available, otherwise download it from github and install it.
ripgrep_in_package_manager="$(apt-cache search --names-only '^ripgrep$')"

if [ -n "${ripgrep_in_package_manager}" ]
then
  apt-get install --yes --quiet --no-install-recommends ripgrep
else
  ripgrep_version=$(curl -s "https://api.github.com/repos/BurntSushi/ripgrep/releases/latest" | grep -Po '"tag_name": "\K[0-9.]+')

  if [ -n "${ripgrep_version}" ]
  then
    echo "Install exa version: ${ripgrep_version}"
    wget -O /tmp/ripgrep.deb "https://github.com/BurntSushi/ripgrep/releases/latest/download/ripgrep_${ripgrep_version}_amd64.deb"
    chmod +x /tmp/ripgrep.deb
    dpkg -i /tmp/ripgrep.deb
    rm -f /tmp/ripgrep.deb
  fi
fi

echo "******************"
echo "* Install Docker *"
echo "******************"

# Ref: https://docs.docker.com/desktop/install/ubuntu/
# Ref: https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user

sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose docker-compose-plugin
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker # re-evaluate group membership (linux only), this way no logout and login is required

echo "******************"
echo "* Install vscode *"
echo "******************"

wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository -y "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt-get update && sudo apt-get install --yes code

echo "************************************"
echo "* Install vpn client: zerotier-cli *"
echo "************************************"

curl -s https://install.zerotier.com | sudo bash

echo "**********************************"
echo "* Cleaning installation residues *"
echo "**********************************"

sudo apt-get clean autoclean
sudo apt-get autoremove --yes
sudo rm -rf /var/lib/apt/lists/*
