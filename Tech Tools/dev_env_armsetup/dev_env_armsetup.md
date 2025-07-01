- [install vscode on arm based linux](#install-vscode-on-arm-based-linux)
- [install ros2 iron (align with gz-garden)](#install-ros2-iron-align-with-gz-garden)
- [install gazebo on arm linux](#install-gazebo-on-arm-linux)


## install gazebo on arm linux
```bash
sudo apt update
sudo apt install lsb-release wget gnupg
wget https://packages.osrfoundation.org/gazebo.gpg -O - | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt update
sudo apt install gz-garden
# add this to .bashrc for mac-linux: alias gzsim="LIBGL_ALWAYS_SOFTWARE=1 gz sim"
```


## install ros2 iron (align with gz-garden)
```bash
# clean the space
sudo apt remove ~nros-humble-* && sudo apt autoremove
sudo apt remove ros2-apt-source
sudo apt update
# delete from .bashrc as well source /opt/ros/humble/setup.bash
# install iron
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
# add repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# install iron
sudo apt update
sudo apt install ros-iron-desktop
# source enviroment
source /opt/ros/iron/setup.bash # add to .bashrc
source ~/.bashrc
# install gazebo garden

# install ros_gz
sudo apt update
sudo apt install ros-iron-ros-gz
```

## install ros2_control from source
```bash
https://control.ros.org/humble/doc/getting_started/getting_started.html # humble - gazebo-Fortress
```

## install vscode on arm based linux
```bash
# Update Packages
sudo apt update
sudo apt install software-properties-common apt-transport-https wget
# Updating keys
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
sudo rm microsoft.gpg
# Adding repository
sudo add-apt-repository "deb [arch=arm64] https://packages.microsoft.com/repos/vscode stable main"
# Install VS Code
sudo apt install code
```