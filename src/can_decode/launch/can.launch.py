from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnProcessStart
import lifecycle_msgs.msg
import launch
import launch_ros
from launch.actions import TimerAction, EmitEvent
def generate_launch_description():
    # 创建 LifecycleNode
    receiver_node = LifecycleNode(
        name='socket_can_receiver_node',
        namespace='ugv',
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',  # 注意：这里应该是 receiver
        parameters=[{'interface': 'can0'}],
        output='screen',
    )
    
    sender_node = LifecycleNode(
        name='socket_can_sender_node',
        namespace='ugv',
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        parameters=[{'interface': 'can0'}],
        output='screen',
    )
    
    # 配置事件
    configure_receiver_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(receiver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # 配置后延迟再激活
    activate_receiver_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(receiver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )
    
    configure_sender_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(sender_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    activate_sender_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(sender_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )
    
    return LaunchDescription([
        receiver_node,
        sender_node,
        
        # 先配置
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=receiver_node,
                on_start=[configure_receiver_event],
            )
        ),
        
        # 延迟后再激活（确保配置完成）
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=receiver_node,
                on_start=[
                    TimerAction(
                        period=2.0,
                        actions=[activate_receiver_event]
                    )
                ],
            )
        ),
        
        # 同样处理 sender
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=sender_node,
                on_start=[configure_sender_event],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=sender_node,
                on_start=[
                    TimerAction(
                        period=2.0,
                        actions=[activate_sender_event]
                    )
                ],
            )
        ),
        launch_ros.actions.Node(
            namespace='ugv',
            package='can_decode',
            executable='can_decode',
            output='screen'),

    ])