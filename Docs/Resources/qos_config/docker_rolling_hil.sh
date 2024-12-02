#!/bin/bash

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

function docker_id() {
    local ID="$(docker container ls -f name="rollipd-docker_hil" | tail -n +2 | cut -d ' ' -f 1)"
    if [ -z "$ID" ]; then
        ID="$(docker container ls -af name="rollipd-docker_hil" | tail -n +2 | cut -d ' ' -f 1)"
        if [ -n "$ID" ]; then
            echo "Starting container $ID ..." >&2
            docker start "$ID" >/dev/null
        fi
    fi
    if [ -z "$ID" ]; then
        echo "Creating container ..." >&2
        ID="$(
            docker run -d -i --privileged \
            -v /etc/shadow:/etc/shadow:ro \
            -v /etc/passwd:/etc/passwd:ro \
            -v /etc/group:/etc/group:ro \
            -v /etc/sudoers:/etc/sudoers:ro \
            -v /usr/share/bash-completion:/usr/share/bash-completion \
            -v /etc/cyclonedds_config.xml.hil:/etc/cyclonedds_config.xml:ro \
            -v ${HOME}:${HOME} \
            -v "${SELF}:${SELF}" \
            -e HOME=${HOME} \
            -e USER=${USER} \
            -e TZ=Asia/Shanghai \
            -u $(id -u):$(id -g) \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -e DISPLAY=$DISPLAY \
            -e GDK_SCALE \
            -e GDK_DPI_SCALE \
            -w "$HOME" \
            --net=host \
            -h "rollipd-docker_hil" \
            --name "rollipd-docker_hil" \
            10.133.122.84:80/build_env_local/public/ubuntu-20.04-ros2-rolling:latest \
            /bin/bash \
            )"
    fi
    echo "$ID"
}

function docker_exec() {
    docker exec --privileged -it "$(docker_id)" "$@"
}

function main() {
    for i in "$@"; do
        case "$i" in
        --container)
            # cat ~/.passwd | sudo -S ip link add name lxc-bridge type bridge 2>/dev/null
            # cat ~/.passwd | sudo -S ip link set eth0 master lxc-bridge
            # cat ~/.passwd | sudo -S ifconfig lxc-bridge 192.168.10.200/24
            # if [[ $(ifconfig) =~ "lxc-bridge.131" ]]; then
            #   cat ~/.passwd | sudo -S ip link del lxc-bridge.131 
            # else
            #   echo "no vlan"
            # fi
            # cat ~/.passwd | sudo -S ip link add link lxc-bridge name lxc-bridge.131 type vlan id 131
            # cat ~/.passwd | sudo -S ifconfig lxc-bridge.131 172.31.131.100/24
            # cat ~/.passwd | sudo -S ip link set lxc-bridge.131 up
            # cat ~/.passwd | sudo -S sed -i "s/>lxc-bridge</>lxc-bridge.131</g" /etc/cyclonedds_config.xml
            # cat ~/.passwd | sudo -S sed -i "s/>lxc-bridge.131</>lxc-bridge</g" /etc/cyclonedds_config.xml
            return
            ;;
        *)
            ;;
        esac
    done

    cat ~/.passwd | sudo -S ifconfig docker0 192.168.10.201/24
    docker_exec "${SELF}" --container

    SELF_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    SELF_PATH="$(cd "$(dirname "$SELF_PATH/../../../../")"; pwd)"
    echo "Project Path: $SELF_PATH"
    if [ "$1" = "runlxc" ]; then
      echo "launch lxc"
      docker_exec /bin/bash -c "cd $SELF_PATH;source build/envsetup.sh; lunch zpd_platform_dj5_s32g_lxc-eng; runemu -p -s; runemu -p;"
    elif [ "$1" = "stoplxc" ]; then
      echo "stop lxc"
      docker_exec /bin/bash -c "cd $SELF_PATH;source build/envsetup.sh; lunch zpd_platform_dj5_s32g_lxc-eng; runemu -p -s;"
    elif [ "$1" = "drivingrviz" ]; then
      echo "launch rviz2 $SELF_PATH"
      docker_exec /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;pkill rviz2;cd $SELF_PATH/packages;source ./install/setup.bash;rviz2 -d asf_nodes/script/driving.rviz"
    elif [ "$1" = "parkingrviz" ]; then
      echo "launch rviz2 $SELF_PATH"
      docker_exec /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;pkill rviz2;cd $SELF_PATH/packages;source ./install/setup.bash;rviz2 -d asf_nodes/script/parking.rviz"
    elif [ "$1" = "rvizproxy" ]; then
      echo "launch rvizproxy"
      docker_exec /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;pkill rviz_proxy_node;cd $SELF_PATH/packages;source ./install/setup.bash;ros2 run rviz_proxy_node rviz_proxy_node"
    elif [ "$1" = "parkingrvizproxy" ]; then
      echo "launch parkingrvizproxy"
      docker_exec /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;pkill parking_rviz_node;cd $SELF_PATH/packages;source ./install/setup.bash;ros2 run parking_rviz_node parking_rviz_node"
    elif [ "$1" = "plotjuggler" ]; then
      echo "launch plotjuggler"
      docker_exec /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;pkill plotjuggler;cd $SELF_PATH/packages;source ./install/setup.bash;ros2 run plotjuggler plotjuggler"
    elif [ "$1" = "vtdproxy" ]; then
      echo "launch vtdproxy $SELF_PATH"
      docker_exec /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;pkill sil_node;cd $SELF_PATH/packages;source ./install/setup.bash;ros2 launch ./asf_nodes/simulation/sil_pkg/launch/driving.py"
    elif [ "$1" = "parkingvtdproxy" ]; then
      echo "launch vtdproxy $SELF_PATH"
      docker_exec /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;pkill sil_node;cd $SELF_PATH/packages;source ./install/setup.bash;ros2 launch ./asf_nodes/simulation/sil_pkg/launch/parking.py"
   elif [ "$1" = "ros2" ]; then
      docker_exec /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=/etc/cyclonedds_config.xml;cd $SELF_PATH/packages;source ./install/setup.bash;bash"
    else
      docker_exec /bin/bash 
    fi
}

main "$@"
