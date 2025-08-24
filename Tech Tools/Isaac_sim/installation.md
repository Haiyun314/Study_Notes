## Install Isaac Sim on Tencent Cloud

### Install and Configure NVIDIA Container Toolkit

- **Verify NVIDIA driver**
```bash
nvidia-smi
```
- Uninstall any previous NVIDIA Docker packages
```bash
sudo apt-get purge -y nvidia-docker*
```
- Update NVIDIA container toolkit repositories
```bash
# Add the package repository GPG key
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Add repository to APT sources list
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update package index
sudo apt-get update
```

- Install NVIDIA container toolkit
```bash 
sudo apt-get install -y nvidia-container-toolkit
```
```bash
sudo nvidia-ctk runtime configure --runtime=docker
```
- Restart Docker
```bash
sudo systemctl restart docker
```
- Verification
```bash
sudo docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu20.04 nvidia-smi
```
If the output matches nvidia-smi on the host, the NVIDIA Container Toolkit has been successfully installed and configured.

### Install and Run Isaac Sim
Log in to NVIDIA NGC
```bash
docker login nvcr.io
#username: $oauthtoken
#Password: <你的 NGC API 密钥>
```
Pull Isaac Sim image
```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
```

Option A: Run with display (GUI)
```bash
docker run --name isaac-sim \
  --gpus all -e DISPLAY=$DISPLAY \
  --network host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/vulkan/icd.d:/etc/vulkan/icd.d \
  -it nvcr.io/nvidia/isaac-sim:2023.1.1-hotfix.1-full-ubuntu20.04
```
Option B: Run headless (no GUI, for WebRTC streaming)
```bash
docker run --name isaac-sim-headless --gpus all --network host --entrypoint bash -it nvcr.io/nvidia/isaac-sim:2023.1.0
```

Check your server’s public IP
```bash
 curl ifconfig.me
```

Now Isaac Sim is installed and running on Tencent Cloud.