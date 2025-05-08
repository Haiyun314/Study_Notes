# Docker Basic

| Concept        | Description                                                             |
| -------------- | ----------------------------------------------------------------------- |
| **Image**      | A snapshot of an environment (like a recipe) used to create containers. |
| **Container**  | A running instance of an image (like a dish made from a recipe).        |
| **Dockerfile** | Script used to automate image creation.                                 |
| **Volume**     | Persistent data storage for containers.                                 |
| **Registry**   | A remote store for Docker images (e.g., Docker Hub).                    |

## Graphic interface
install **XQuartz** then enable **allow connections from network clients** from **Preferences -> Security**
```bash
xhost + 127.0.0.1

docker run -it --name ros2_gui_container -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix ros:humble
```

## image command
```bash
# Pull an image from Docker Hub
docker pull <image-name>
# List all images on your machine
docker images
# Remove an image
docker rmi <image-id>
```

## Container command
```bash
# Run a container from an image, this will create a new container
docker run -it <image-name>
# Run a container with a custom name and auto-remove on exit
docker run -it --rm --name <container-name> <image-name>
# Start an existing container
docker start <container-id or name>
# Attach to a running container
docker attach <container-id or name>
# Execute a command in a running container
docker exec -it <container-name> bash
# Stop a running container
docker stop <container-id or name>
# List running containers
docker ps
# List all containers (including stopped ones)
docker ps -a
# Remove a stopped container
docker rm <container-id or name>
```

## Volumes and Bind Mounts
```bash
# create a volume on local machine, the location is distributed by docker
docker volume create my_volume
# Run container with volume
docker run -v my_volume:/container_path <image-name>
# Run container with bind mount (host to container)
docker run -v /host/path:/container_path <image-name>
# The -v flag allows you to share data between the container and your host by mounting specific folders. For example, mounting ros2_ws allows you to edit the workspace locally and use it inside the container.
docker run -it --name ros2_container -v /path/to/ros2_ws:/ros2_ws ros:humble
```


## Dockerfile Basics
<details>
<summary> Dockerfile example</summary>

```bash
# Use an official ROS 2 Humble base image
FROM ros:humble

# Set the maintainer label
LABEL maintainer="your_name@domain.com"

# Set environment variables for non-interactive installation and working directory
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/ros2_ws

# Install common dependencies for building ROS 2 and additional tools
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    python3-venv \
    python3-setuptools \
    git \
    wget \
    curl \
    sudo \
    vim \
    less \
    && apt-get clean

# Install ROS 2 Python dependencies
RUN pip3 install -U \
    setuptools \
    colcon-common-extensions

# Create ROS workspace directories
RUN mkdir -p $ROS_WS/src

# Set up the workspace inside the container, bind the source code
COPY ./ros2_ws/ $ROS_WS/

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Source ROS 2 setup script when the container starts
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

# Set the working directory to the ROS workspace
WORKDIR $ROS_WS

# Build the workspace by default when the container starts
RUN colcon build

# Expose any necessary ports (ROS 2 uses default port 11311 for ROS master)
EXPOSE 11311

# Entry point to run ROS 2 nodes
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && source $ROS_WS/install/setup.bash && exec bash"]
```
</details>

## Build and Run
```bash
# Build an image from Dockerfile
docker build -t my-image-name .
# Run a container from the built image
docker run -it my-image-name
```

## Docker regiestry
```bash
# Download an image from a registry (e.g., Docker Hub).
docker pull <image_name>
# Upload an image to a registry.
docker push <image_name>
# Tag an image for pushing to a registry.
docker tag <image_name> <username>/<repo>:<tag>
# Login to Docker Hub (or another registry).
docker login
# Search for Docker images on Docker Hub.
docker search <image_name>
```