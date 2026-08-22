#!/bin/bash
source config.env
docker build -t ${IMAGE_NAME} -f Dockerfile.gpu .
# docker build -t ${IMAGE_NAME}-cpu -f Dockerfile.cpu .