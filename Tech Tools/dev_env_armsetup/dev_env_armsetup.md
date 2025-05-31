[install vscode on arm based linux](#install-vscode-on-arm-based-linux)
[install gazebo on arm linux](#install-gazebo-on-arm-linux)

## install gazebo on arm linux
```bash
sudo apt update
sudo apt install lsb-release wget gnupg
wget https://packages.osrfoundation.org/gazebo.gpg -O - | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt update
sudo apt install gz-garden
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