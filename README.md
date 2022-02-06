# PMOD-ROS1

## Requirement

- NVIDIA-Driver `>=418.81.07`
- Docker `>=19.03`
- NVIDIA-Docker2

## Preparation

1. 
    ```bash
    git clone https://github.com/shikishima-TasakiLab/pmod-ros1.git
    ```
1. Place a trained Torch Script model in `./model`.

## Docker Image

- pull
    ```bash
    docker pull shikishimatasakilab/pmod-ros1:amd64-torch1.7
    ```

- build
    ```bash
    ./docker/build-melodic-amd64.sh
    ```

## Start a Docker Container

1. Start a Docker container with the following command. 
    ```bash
    ./docker/run.sh
    ```

1. Select the IP address to be used as the "ROS_IP".

1. Build source code.
    ```bash
    catkin build
    source /workspace/devel/setup.bash
    ```

## Launch

1. Launch the ROS nodes with the following command.
    ```bash
    roslaunch pmod_ros pmod.launch
    ```

### ROS Launch

```xml
<launch>
    <node name="pmod" pkg="pmod_ros" type="pmod" output="screen">
        <rosparam command="load" file="$(find pmod_ros)/config/5class.yaml"/>
    </node>
</launch>
```

### ROS Param

```yaml
# Trained Torch Script model
checkpoint: /workspace/src/pmod_ros/model/00169_PMOD.pt

hz: 5.0                     # Frequency of Timer Callback [1/s]

pub_seg_id: False           # If publishing semantic maps, True.
pub_seg_color: True         # If publishing color semantic maps, True.
pub_depth: False            # If publishing depth maps, True.
pub_points_ground: False    # If publishing point clouds of the ground, True.
pub_points_noground: False  # If publishing point clouds that are not ground, True.
pub_points_static: False    # If publishing static point clouds, True.
pub_points_dynamic: True    # If publishing dynamic point clouds, True

use_optical_frame: False    # If publishing point clouds in the camera coordinate system, True.

seg_labels:                 # Configuring Semantic Label
  - id: 0                   #   ID          : Unique Integer
    tag: Void               #   Tag         : Tag of label
    is_ground: False        #   Is Ground   : True if label is ground.
    is_dynamic: False       #   Is Dynamic  : True if label is dynamic obstacle.
    color:                  #   Color       :
      r: 0                  #       R       : Red   [0-255]
      g: 0                  #       G       : Green [0-255]
      b: 0                  #       B       : Blue  [0-255]
  - id: 1
    tag: Ground
    is_ground: True
    is_dynamic: False
    color:
      r: 70
      g: 130
      b: 180
  - id: 2
    tag: Obstacle
    is_ground: False
    is_dynamic: False
    color:
      r: 70
      g: 70
      b: 70
  - id: 3
    tag: Vehicle
    is_ground: False
    is_dynamic: True
    color:
      r: 0
      g: 0
      b: 142
  - id: 4
    tag: Person
    is_ground: False
    is_dynamic: True
    color:
      r: 220
      g: 20
      b: 60

height: 256                 # Height of the input/output image[px]
width: 512                  # Width of the input/output image[px]

sub_queue_size: 10          # Queue size of Subscriber
pub_queue_size: 2           # Queue size of Publisher
```
