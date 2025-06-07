import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch arguments
    text_arg = launch.actions.DeclareLaunchArgument(
        'text',
        default_value='The quick brown fox jumps over the lazy dog.',
        description='String to be splitter'
    )

    # Create action server node
    server_node = Node(
        package='vazquez_ejer2_pkg',
        executable='republisher_server',
        name='republisher_server',
        output='screen',
        parameters=[ ],
    )

    # Create action client node
    client_node = Node(
        package='vazquez_ejer2_pkg',
        executable='republisher_client',
        name='republisher_client',
        output='screen',
        arguments=[LaunchConfiguration('text')]
    )

    # Create launch description and add nodes and arguments
    ld = launch.LaunchDescription()
    ld.add_action(text_arg)
    ld.add_action(actionServer_node)
    ld.add_action(actionClient_node)
    
    return ld