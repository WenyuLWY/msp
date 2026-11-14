#!/bin/bash

source config.env
# docker exec -it $CONTAINER_NAME /bin/bash
docker exec -it "$CONTAINER_NAME" /bin/bash -ic 'roslaunch tare_planner explore.launch; exec bash'