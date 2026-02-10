# ONLY the important part shown (exactly what I changed)

gz_bridge_node = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
        "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"
    ],
    remappings=[
        ('/cmd_vel', '/safe_cmd_vel'),
    ],
    output="screen",
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
)
