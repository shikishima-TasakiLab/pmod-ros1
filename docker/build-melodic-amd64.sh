#!/bin/bash
RUN_DIR=$(dirname $(readlink -f $0))
CUDA_VERSION="11.0"
CUDA_SHORT="cu110"
CUDNN_VERSION="8"
TORCH_VERSION="1.7.0"
TORCH_SHORT="1.7"
SRC_IMAGE="nvidia/cuda:${CUDA_VERSION}-cudnn${CUDNN_VERSION}-devel-ubuntu18.04"

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
  --build-arg CUDA_VERSION=${CUDA_SHORT} \
  --build-arg TORCH_VERSION=${TORCH_VERSION} \
  -t shikishimatasakilab/pmod-ros1:amd64-torch${TORCH_SHORT} \
  -f ${RUN_DIR}/src/Dockerfile.melodic.amd64 \
  ${RUN_DIR}/src
