# Traversability Analysis

## Usage

### Docker
To run the repository in Docker, follow these steps:

1. **Build the Docker Image**
   Navigate to the folder where the repository is cloned and run the following command:
   ```bash
   docker build -t traversability-humble-jammy .
   ```

2. **Create and Run a Container**
   Use the following command to create and start a container:
   ```bash
   docker run --init -it -d \
       --name traversability-humble-jammy-container \
       -v /etc/localtime:/etc/localtime:ro \
       -v /etc/timezone:/etc/timezone:ro \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $(pwd)/src:/root/ros2_ws/src \
       -v $(pwd)/Utilities/launchproject:/root/launchproject \
       -e DISPLAY=$DISPLAY \
       --runtime=nvidia \
       --gpus all \
       traversability-humble-jammy bash
   ```

### TODO

- [Done]    Design Dockerfile build.
- [Working] Fix the map fusion problem.
- [ ]       Resolve the GPU issues.
