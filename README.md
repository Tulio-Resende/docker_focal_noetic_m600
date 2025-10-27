## Getting Started

### 1. Install Docker

Follow the [Docker Engine installation guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/):

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | sudo tee /etc/apt/sources.list.d/docker.list
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

---

### 2. Run Docker Without `sudo`

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
sudo reboot
newgrp docker
```

---

### 3. (Optional) Enable NVIDIA GPU Support

To enable GPU acceleration, install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html):

**Install prerequisites:**
```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends curl gnupg2
```

**Add the NVIDIA repository:**
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

**Enable experimental packages and install:**
```bash
sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.0-1
sudo apt-get install -y   nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION}   nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION}   libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION}   libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```

**Configure Docker runtime:**
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

### 4. Build and Launch the Simulator

1. **Clone the repository:**
   ```bash
   git@github.com:Tulio-Resende/docker_focal_noetic_m600.git
   cd docker_focal_noetic_m600
   ```

2. **Enable BuildKit:**
   ```bash
   export DOCKER_BUILDKIT=1
   ```

3. **Start the SSH agent:**
   ```bash
   eval "$(ssh-agent -s)"
   ```

4. **Build the Docker image: (Notice that you need to enable ssh and have access to LEAD's Gitlab env in order to clone perception packages)**
   ```bash
   docker build --ssh default -t ariel-image .
   ```

5. **Launch the container:**
   ```bash
   docker start ariel-image
   docker exec -it <CONTAINER_NAME> /bin/bash
   ```

6. **More detailed info available on documentation folder**
---