from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch_ros.actions import Node
import ament_index_python.packages
import os
import launch
import launch.launch_service
from launch.substitutions import LaunchConfiguration

# sudo chmod 777 /dev/ttyACM0
# sudo chmod 777 /dev/ttyACM1

def generate_launch_description():
    #General settings
    output = LaunchConfiguration('output', default='screen')
    respawn = LaunchConfiguration('respawn', default='true')
    respawn_delay = LaunchConfiguration('respawn_delay', default='30')
    clear_params = LaunchConfiguration('clear_params', default='true')

    # u-blox general
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_utils'),
        'config')
    params = os.path.join(config_directory, 'zed_f9p.yaml')

    # u-blox position receiver
    device_position_receiver = LaunchConfiguration('device_position_receiver', default='/dev/ttyACM0')
    frame_id_position_receiver = LaunchConfiguration('frame_id_position_receiver', default='gnss')

    m8n_device_position_receiver = LaunchConfiguration('device_position_receiver', default='/dev/ttyACM0')
    m8n_params = os.path.join(config_directory, 'm8n.yaml') #c94_m8t_rover
    
    # u-blox moving baseline receiver
    use_moving_baseline = LaunchConfiguration('use_moving_baseline', default='false')
    device_moving_baseline_receiver = LaunchConfiguration('device_moving_baseline_receiver', default='/dev/ttyACM1')
    frame_id_moving_baseline_receiver = LaunchConfiguration('frame_id_moving_baseline_receiver', default='position_receiver')

    
    # NTRIP
    use_ntrip = LaunchConfiguration('use_ntrip', default='true')
    use_ntrip_arg = DeclareLaunchArgument(
        'use_ntrip',
        default_value="true",
        description="Set to 'true' if you want to use NTRIP. Remember to set the credentials."
    )
    navpvt_topic = LaunchConfiguration('navpvt_topic', default='/ublox_position_receiver/navpvt')
    nmea_topic = LaunchConfiguration('nmea_topic', default='/ntrip_client/nmea')

    ntrip_host = LaunchConfiguration('ntrip_host', default="")
    ntrip_port = LaunchConfiguration('ntrip_port', default='2101')
    ntrip_mountpoint = LaunchConfiguration('ntrip_mountpoint', default='')
    ntrip_version = LaunchConfiguration('ntrip_version', default='')
    ntrip_authentificate = LaunchConfiguration('ntrip_authentificate', default='true')
    ntrip_username = LaunchConfiguration('ntrip_username', default="")
    ntrip_password = LaunchConfiguration('ntrip_password', default="")

    # The ublox position receiver
    ublox_position_receiver = launch_ros.actions.Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output=output,
        parameters=[params,
        #{"package_name": "ublox_utils"},
        #{"launch_file_name": "zed_f9p.yaml"},
        {"device": device_position_receiver},
        {"frame_id":frame_id_position_receiver}
        #,{"rate": 1.5}
        ],
        #respawn=True,
        #respawn_delay=4,
        #clear_params=clear_params,
        #respawn=respawn_param,
        #respawn_delay=respawn_delay_param,
        remappings=[
            ("/rtcm","/ublox_position_receiver/rtcm")
        ]
    )

    # The second optional ublox moving baseline receiver
    if(use_moving_baseline):
        ublox_moving_baseline_receiver = launch_ros.actions.Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            parameters=[
                params,
                {"publish/nav/heading":"true"},
                {"device": device_moving_baseline_receiver}
                ],
            output=output,  
            #clear_params=clear_params,
            #respawn=respawn_param,
            #respawn_delay=respawn_delay_param,
            remappings=[
                ("/rtcm","/ublox_moving_baseline_receiver/rtcm")
            ]
        )

    if(use_ntrip):
        print("NTRIP is being used")
        #This node creates NMEA $GPGGA messages from u-blox navpvt to send to the NTRIP caster
        ublox2nmea = Node(
            package="ublox_utils",
            executable="ublox2nmea",
            output=output,
            remappings=[
                ("/navpvt",navpvt_topic),
                ("/nmea",nmea_topic)
            ]
        )
        
        #This node transforms mavros/rtcm to rtcm/Message
        rtcm_transform = Node(
            package="topic_tools",
            executable="transform",
            output=output,
            arguments=[
            '/ntrip_client/rtcm',
            '/ublox_position_receiver/rtcm',
            'rtcm_msgs/Message',
            "rtcm_msgs.msg.Message(header=m.header, message=m.data)",
            '--import','rtcm_msgs',
            '--wait-for-start'
            #"""ros2 run topic_tools transform /ntrip_client/rtcm /ublox_position_receiver/rtcm mavros_msgs/RTCM "mavros_msgs.msg.RTCM(header=m.header, message=m.data)" --import rtcm_msgs --wait-for-start"""
            ],
        )

        # #This node relays the current NMEA $GPGGA position to the NTRIP caster and returns the RTCM corrections
        ntrip_client = Node(
            package="ntrip_client",
            executable="ntrip_ros.py",
            output=output,
            parameters=[
                {"host":ntrip_host},
                {"port":ntrip_port},
                {"mountpoint":ntrip_mountpoint},
                {"ntrip_version":ntrip_version},
                {"authenticate":ntrip_authentificate},
                {"username":ntrip_username},
                {"password":ntrip_password},
                {"rtcm_frame_id":frame_id_position_receiver}
            ],
            remappings=[
                ("/rtcm", "/ntrip_client/rtcm"),
                ("/nmea", "/ntrip_client/nmea")
            ]
        )
    # The ublox position receiver
    m8n_ublox_position_receiver = launch_ros.actions.Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output=output,
        parameters=[m8n_params,
        {"device": m8n_device_position_receiver}
        #,{"rate": 1.5}
        ],
        #namespace="m8n",
        #respawn=True,
        #respawn_delay=4,
        #clear_params=clear_params,
        #respawn=respawn_param,
        #respawn_delay=respawn_delay_param,
        remappings=[
            ("/rtcm","/ublox_position_receiver/rtcm")
        ]
    )

    return LaunchDescription([ublox_position_receiver,
                                ublox2nmea,ntrip_client,rtcm_transform,
                                #m8n_ublox_position_receiver,
                                    use_ntrip_arg,
    #                                  launch.actions.RegisterEventHandler(
    #                                      event_handler=launch.event_handlers.OnProcessExit(
    #                                          target_action=ublox_position_receiver,
    #                                          on_exit=[launch.actions.EmitEvent(
    #                                              event=launch.events.Shutdown())
    #                                              ],
    # )),
    ])
