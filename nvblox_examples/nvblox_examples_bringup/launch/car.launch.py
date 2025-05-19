# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera, NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME

EMITTER_FLASHING_CONFIG_FILE_PATH = lu.get_path('nvblox_examples_bringup', 'config/sensors/realsense_emitter_flashing.yaml')
EMITTER_ON_CONFIG_FILE_PATH = lu.get_path('nvblox_examples_bringup', 'config/sensors/realsense_emitter_on.yaml')

def start(args: lu.ArgumentContainer) -> List[Action]:
    actions = []
    # Single or Multi-realsense
    camera_mode = str(NvbloxCamera.multi_realsense)
    
    # Only up to 4 Realsenses is supported.
    actions.append(
        lu.assert_condition(
            'Up to 4 cameras have been tested! num_cameras must be less than 5.',
            IfCondition(PythonExpression(['int("', str(args.num_cameras), '") > 4']))),
    )

    run_splitter_list = [False] * args.num_cameras
    run_splitter_list[0] = True
    
    for idx in range(args.num_cameras):
        run_splitter = run_splitter_list[idx]
        nodes = []
        camera_name = f'car/camera{idx}'
        # Config file
        if run_splitter:
            config_file_path = EMITTER_FLASHING_CONFIG_FILE_PATH
        else:
            config_file_path = EMITTER_ON_CONFIG_FILE_PATH
        # Realsense
        log_message = lu.log_info(f'Starting realsense with name: {camera_name}, running splitter: {run_splitter}')
        parameters = []
        parameters.append(config_file_path)
        parameters.append({'camera_name': camera_name})
        nodes.append(
             ComposableNode(
        	namespace=camera_name,
        	package='realsense2_camera',
        	plugin='realsense2_camera::RealSenseNodeFactory',
        	parameters=parameters)
        )
        # Splitter
        if run_splitter:
            nodes.append(
                ComposableNode(
                    namespace=camera_name,
                    name='realsense_splitter_node',
                    package='realsense_splitter',
                    plugin='nvblox::RealsenseSplitterNode',
                    parameters=[{
                        'input_qos': 'SENSOR_DATA',
                        'output_qos': 'SENSOR_DATA'
                    }],
                    remappings=[
                        ('input/infra_1', f'/{camera_name}/infra1/image_rect_raw'),
                        ('input/infra_1_metadata', f'/{camera_name}/infra1/metadata'),
                        ('input/infra_2', f'/{camera_name}/infra2/image_rect_raw'),
                        ('input/infra_2_metadata', f'/{camera_name}/infra2/metadata'),
                        ('input/depth', f'/{camera_name}/depth/image_rect_raw'),
                        ('input/depth_metadata', f'/{camera_name}/depth/metadata'),
                        ('input/pointcloud', f'/{camera_name}/depth/color/points'),
                        ('input/pointcloud_metadata', f'/{camera_name}/depth/metadata'),
                ])
            )
        # Note(xinjieyao: 2024/08/24): Multi-rs launch use RealSenseNodeFactory could be unstable
        # Camera node may fail to launch without any ERROR or app crashes
        # Adding delay for cameras after the first camera bringup (including splitter) as temp fix
        actions.append(
            TimerAction(
                period=idx * 10.0, actions=[lu.load_composable_nodes(args.container_name, nodes)]))
        actions.append(log_message)

    camera = NvbloxCamera.multi_realsense
    # NOTE(alexmillane, 19.08.2024): At the moment in nvblox_examples we only support a single
    # camera running cuVSLAM, even in the multi-camera case: we run *nvblox* on multiple
    # cameras, but cuVSLAM on camera0 only.
    realsense_remappings = [
        ('visual_slam/camera_info_0', '/car/camera0/infra1/camera_info'),
        ('visual_slam/camera_info_1', '/car/camera0/infra2/camera_info'),
        ('visual_slam/image_0', '/car/camera0/realsense_splitter_node/output/infra_1'),
        ('visual_slam/image_1', '/car/camera0/realsense_splitter_node/output/infra_2'),
        ('visual_slam/imu', 'car/camera0/imu'),
    ]

    # Base frame: 
    # - camera0_link for single realsense,
    # - base_link for everything else (multi realsense)
    if camera is NvbloxCamera.realsense:
        base_frame = 'car/camera0_link'
    else:
        base_frame = 'car/base_link'

    actions.append(lu.log_info(f'Starting cuVSLAM with base_frame: {base_frame}'))

    base_parameters = {
        'num_cameras': 2,
        'min_num_images': 2,
        'enable_localization_n_mapping': False,
        'gyro_noise_density': 0.000244,
        'gyro_random_walk': 0.000019393,
        'accel_noise_density': 0.001862,
        'accel_random_walk': 0.003,
        'calibration_frequency': 200.0,
        'rig_frame': 'car/base_link',
        'imu_frame': 'car/front_stereo_camera_imu',
        'enable_slam_visualization': True,
        'enable_landmarks_view': True,
        'enable_observations_view': True,
        'path_max_size': 200,
        'verbosity': 5,
        'enable_debug_mode': False,
        'debug_dump_path': '/tmp/cuvslam',
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': base_frame,
    }
    realsense_parameters = {
        'enable_rectified_pose': True,
        'enable_image_denoising': False,
        'rectified_images': True,
        'imu_frame': 'car/camera0_gyro_optical_frame',
        'camera_optical_frames': [
            'car/camera0_infra1_optical_frame',
            'car/camera0_infra2_optical_frame',
        ],
    }

    if camera is NvbloxCamera.realsense or NvbloxCamera.multi_realsense:
        remappings = realsense_remappings
        camera_parameters = realsense_parameters
    else:
        raise Exception(f'Camera {camera} not implemented for vslam.')

    parameters = []
    parameters.append(base_parameters)
    parameters.append(camera_parameters)
    parameters.append(
        {'enable_ground_constraint_in_odometry': args.enable_ground_constraint_in_odometry})
    parameters.append({'enable_imu_fusion': args.enable_imu_fusion})

    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=remappings,
        parameters=parameters)
    actions.append(lu.load_composable_nodes(args.container_name, [vslam_node]))

    # People detection for multi-RS
    #camera_namespaces = ['camera0', 'camera1', 'camera2', 'camera3']
    camera_namespaces = [i+'/'+j for i in ['car', 'uav1', 'uav2', 'uav3', 'uav4'] for j in ['camera0', 'camera1', 'camera2', 'camera3']]

    num_cameras = int(args.num_cameras)
#    use_lidar = lu.is_true(args.lidar)
    base_config = lu.get_path('nvblox_examples_bringup', 'config/nvblox/car_nvblox_base.yaml')
    multi_realsense_config = lu.get_path(
        'nvblox_examples_bringup', 'config/nvblox/specializations/car_nvblox_multi_realsense.yaml')
    mode_config = {}
    remappings = []
    for i in range(0, args.num_cameras):
        if i == 0:
            # Only cam0 (i == 0) runs splitter.
            remappings.append(
                (f'car/camera_{i}/depth/image', f'/car/camera{i}/realsense_splitter_node/output/depth'))
            remappings.append((f'car/camera_{i}/depth/camera_info', f'/car/camera{i}/depth/camera_info'))
        else:
            remappings.append((f'car/camera_{i}/depth/image', f'/car/camera{i}/depth/image_rect_raw'))
            remappings.append((f'car/camera_{i}/depth/camera_info', f'/car/camera{i}/depth/camera_info'))
        remappings.append((f'car/camera_{i}/color/image', f'/car/camera{i}/color/image_raw'))
        remappings.append((f'car/camera_{i}/color/camera_info', f'/car/camera{i}/color/camera_info'))
        
    for i in range(0, args.num_uavs):
        remappings.append(
            (f'uav{i}/camera_0/depth/image', f'/uav{i}/camera0/realsense_splitter_node/output/depth'))
        remappings.append((f'uav{i}/camera_0/depth/camera_info', f'/uav{i}/camera0/depth/camera_info'))
        remappings.append((f'uav{i}/camera_0/color/image', f'/uav{i}/camera0/color/image_raw'))
        remappings.append((f'uav{i}/camera_0/color/camera_info', f'/uav{i}/camera0/color/camera_info'))

    camera_config = multi_realsense_config
    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(camera_config)
    parameters.append({'num_cameras': args.num_uavs+num_cameras})
#    parameters.append({'use_lidar': use_lidar})

    # Add the nvblox node.
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions.append(
        lu.log_info(
            ["Starting nvblox in static mode."]))
             
    # TF transforms for multi-realsense
    actions.append(
        lu.add_robot_description(robot_calibration_path=args.multicam_urdf_path)
    )
         
    # Visualization
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/visualization/visualization.launch.py',
            launch_arguments={
                'mode': NvbloxMode.static,
                'camera': camera_mode,
                'use_foxglove_whitelist': args.use_foxglove_whitelist,
            }))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('log_level', 'info', choices=[
                 'debug', 'info', 'warn'], cli=True)
    args.add_arg('num_cameras', 1,
                 description='How many cameras to use.', cli=True)
    args.add_arg('num_uavs', 1,
                 description='How many uavs to use.',
                 cli=True)
    args.add_arg(
        'multicam_urdf_path',
        lu.get_path('nvblox_examples_bringup',
                    'config/urdf/4_realsense_carter_example_calibration.urdf.xacro'),
        description='Path to a URDF file describing the camera rig extrinsics. Only used in multicam.',
        cli=True)
    args.add_arg(
        'attach_to_container',
        'False',
        description='Add components to an existing component container.',
        cli=True)
    args.add_arg(
        'container_name',
        NVBLOX_CONTAINER_NAME,
        description='Name of the component container.')
    args.add_arg(
        'run_realsense',
        'True',
        description='Launch Realsense drivers')
    args.add_arg(
        'use_foxglove_whitelist',
        True,
        description='Disable visualization of bandwidth-heavy topics',
        cli=True)
        
    args.add_opaque_function(start)
    actions = args.get_launch_actions()

    # Container
    # NOTE: By default (attach_to_container:=False) we launch a container which all nodes are
    # added to, however, we expose the option to not launch a container, and instead attach to
    # an already running container. The reason for this is that when running live on multiple
    # realsenses we have experienced unreliability in the bringup of multiple realsense drivers.
    # To (partially) mitigate this issue the suggested workflow for multi-realsenses is to:
    # 1. Launch RS (cameras & splitter) and start a component_container
    # 2. Launch nvblox + cuvslam and attached to the above running component container

    actions.append(
        lu.component_container(
            NVBLOX_CONTAINER_NAME, condition=UnlessCondition(args.attach_to_container),
            log_level=args.log_level))

    return LaunchDescription(actions)
