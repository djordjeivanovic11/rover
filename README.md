### Docker setup
To quickly open the repo in a ROS2 docker image with a similar environment to the rover's jetson, run:
```bash
docker run --rm -it $(docker build -q .) /bin/bash
```
**Be careful**, any changes you make to the repo from inside the docker container will not be reflected on your host filesystem and any changes made to the container will be lost as soon as you exit the container.

You can also build and tag the image to create a more customized container that persists or mounts the host filesystem if needed, but I think it's safer to treat the container as something you reset each time you want to test new changes, not a permanent development environment (unless you know what you're doing).