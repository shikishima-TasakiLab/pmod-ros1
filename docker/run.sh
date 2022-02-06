#!/bin/bash
PROG_NAME=$(basename $0)
RUN_DIR=$(dirname $(readlink -f $0))
PKG_DIR=$(dirname ${RUN_DIR})

DOCKER_IMAGE="shikishimatasakilab/pmod-ros1:amd64-torch1.7"
# DOCKER_IMAGE="shikishimatasakilab/pmod-ros1:jetson-r32.5.0-torch1.7"

DOCKER_VOLUME="${DOCKER_VOLUME} -v ${PKG_DIR}:/workspace/src/pmod_ros:rw"

ip_list=($(hostname -I))
if [[ ${#ip_list[@]} -eq 1 ]]; then
    ROS_IP="${ip_list[0]}"
else
    echo -e "番号\tIPアドレス"
    cnt=0
    for ip in "${ip_list[@]}"; do
        echo -e "${cnt}:\t${ip}"
        cnt=$((${cnt}+1))
    done
    isnum=3
    ip_num=-1
    while [[ ${isnum} -ge 2 ]] || [[ ${ip_num} -ge ${cnt} ]] || [[ ${ip_num} -lt 0 ]]; do
        read -p "使用するIPアドレスの番号を入力してください: " ip_num
        expr ${ip_num} + 1 > /dev/null 2>&1
        isnum=$?
    done
    ROS_IP="${ip_list[${ip_num}]}"
fi

DOCKER_ENV="${DOCKER_ENV} -e ROS_IP=${ROS_IP}"

docker run \
    -it \
    --rm \
    --gpus all \
    --net host \
    ${DOCKER_VOLUME} \
    ${DOCKER_ENV} \
    --name "pmod-ros" \
    ${DOCKER_IMAGE} \
    bash
