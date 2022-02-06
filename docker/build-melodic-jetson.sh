#!/bin/bash
RUN_DIR=$(dirname $(readlink -f $0))
KERNEL_VERSION="r32.5.0"
TORCH_VERSION="1.7"
SRC_IMAGE="nvcr.io/nvidia/l4t-pytorch:${KERNEL_VERSION}-pth${TORCH_VERSION}-py3"

function usage_exit {
  cat <<_EOS_ 1>&2
  Usage: build-docker.sh [OPTIONS...]
  OPTIONS:
    -h, --help                          Show this help
    -i, --base-image DOCKER_IMAGE[:TAG] Specify the base Docker image
_EOS_
  exit 1
}

while (( $# > 0 )); do
  if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
    usage_exit
  elif [[ $1 == "-i" ]] || [[ $1 == "--base-image" ]]; then
    if [[ $2 == -* ]]; then
      echo "Invalid parameter"
      usage_exit
    else
      SRC_IMAGE=$2
    fi
    shift 2
  else
    echo "Invalid parameter: $1"
    usage_exit
  fi
done

if [[ -z ${SRC_IMAGE} ]]; then
  echo "Specify the base Docker image."
  usage_exit
fi

docker build \
  --build-arg SRC_IMAGE=${SRC_IMAGE} \
  -t shikishimatasakilab/pmod-ros1:jetson-${KERNEL_VERSION}-torch${TORCH_VERSION} \
  -f ${RUN_DIR}/src/Dockerfile.melodic.jetson \
  ${RUN_DIR}/src
