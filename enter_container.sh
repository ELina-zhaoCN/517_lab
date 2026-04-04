#!/bin/bash
if docker ps -a --format '{{.Names}}' | grep -q '^techin517_dev$'; then
  docker start -ai techin517_dev
else
  sudo docker run -it \
    --name techin517_dev \
    --gpus all \
    --privileged \
    --network=host \
    -v ~/TECHIN517:/home/ubuntu/techin517 \
    -v /dev:/dev \
    techin517:setup \
    /bin/bash
fi
