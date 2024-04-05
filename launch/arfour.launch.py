import os
import pathlib
import yaml
import launch
from launch import LaunchIntrospector

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action

import lifecycle_msgs.msg

#from launch import LaunchDescription
#from launch_ros.actions import Node

def generate_launch_description():

    config_qualisys = os.path.join(get_package_share_directory('ros2_qualisys_driver'),
                                   'params', 'params.yaml')

    ld = LaunchDescription()

    # qualisys_node = LifecycleNode(
    #     package='ros2_qualisys_driver',              # must match name in config -> YAML
    #     executable='qualisys_driver_exe',
    #     # must match node name in config -> YAML
    #     name='ros2_qualisys_driver',
    #     namespace='qualisys_driver',
    #     emulate_tty=True,
    #     output='screen',
    #     parameters=[config_qualisys]
    # )    

    # configure_event = EmitEvent(
    #     event=ChangeState(
    #         lifecycle_node_matcher=matches_action(qualisys_node),
    #         transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    #     )
    # )

    # # Make the node take the 'activate' transition
    # activate_event = RegisterEventHandler(
    #     OnStateTransition(
    #         target_lifecycle_node=qualisys_node, goal_state='inactive',
    #         entities=[
    #             LogInfo(
    #                 msg="[LifecycleLaunch] ros2_qualisys_driver node is activating."),
    #             EmitEvent(event=ChangeState(
    #                 lifecycle_node_matcher=matches_action(qualisys_node),
    #                 transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
    #             )),
    #         ],
    #     )
    # )    
    qualisys_node =Node(
        package='ros2_qualisys_driver',              # must match name in config -> YAML
        executable='qualisys_driver_exe',
        # must match node name in config -> YAML
        name='ros2_qualisys_driver',
        namespace='qualisys_driver',
        emulate_tty=True,
        output='screen',
        parameters=[config_qualisys]
    )     
    joy_driver = Node(
            package='joy',            
            executable='joy_node',
            name='joy_stick_driver'
    )

    mocap_odom = Node(
            package='artemis_tools_ros2',
            executable='pose_listener_qualisys',
            name='qualisys_odom_streamer'
    )   
    
    ld.add_action(mocap_odom)
    ld.add_action(joy_driver)
    ld.add_action(qualisys_node)
   # ld.add_action(activate_event)
   # ld.add_action(configure_event)  
    #ld.add_action(mocap_odom)
    
    return ld  

