# Bridge node (ROS <-> Gazebo)
# NOTE:
# - Safety node publishes SAFE Twist on /cmd_vel
# - ros_gz_bridge must therefore subscribe to /cmd_vel

gz_bridge_node = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
        "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
    ],
    output="screen",
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
)
