from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # ========================================================================
    # 参数配置
    # ========================================================================
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='nav',
        description='运行模式: "nav" (定位+导航) 或 "mapping" (仅建图)'
    )

    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='faster_lio',
        description='定位后端: fast_lio, faster_lio, point_lio'
    )

    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd',
        default_value=PathJoinSubstitution([FindPackageShare("rm_bringup"), "PCD", "siyuan", "home.pcd"]),
        description='3D 点云地图路径 (用于定位)'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution([FindPackageShare("rm_bringup"), "PCD", "siyuan", "home.yaml"]),
        description='2D 栅格地图路径 (用于导航)'
    )

    pcd_save_en_arg = DeclareLaunchArgument(
        'pcd_save_en',
        default_value='true',
        description='是否保存定位/建图点云 PCD'
    )
    pcd_save_interval_arg = DeclareLaunchArgument(
        'pcd_save_interval',
        default_value='-1',
        description='PCD 保存间隔；-1 表示不按固定间隔分段'
    )

    initial_x_arg = DeclareLaunchArgument(
        'initial_x',
        default_value='0.0',
        description='初始 X 位置(map坐标系)'
    )
    initial_y_arg = DeclareLaunchArgument(
        'initial_y',
        default_value='0.0',
        description='初始 Y 位置(map坐标系)'
    )
    initial_z_arg = DeclareLaunchArgument(
        'initial_z',
        default_value='0.0',
        description='初始 Z 位置(map坐标系)'
    )
    initial_yaw_arg = DeclareLaunchArgument(
        'initial_yaw',
        default_value='0.0',
        description='初始航向角，单位 rad'
    )
    use_initial_pose_arg = DeclareLaunchArgument(
        'use_initial_pose',
        default_value='false',
        description='定位是否使用 initial_x/initial_y/initial_z/initial_yaw 作为初始位姿'
    )

    # 子系统开关
    driver_arg = DeclareLaunchArgument(
        'driver', default_value='true', description='启动雷达驱动'
    )
    comm_arg = DeclareLaunchArgument(
        'comm', default_value='false', description='启动通信节点'
    )
    decision_arg = DeclareLaunchArgument(
        'decision', default_value='false', description='启动决策节点'
    )
    terrain_arg = DeclareLaunchArgument(
        'terrain', default_value='true', description='启动地形分析'
    )
    region_detector_arg = DeclareLaunchArgument(
        'region_detector', default_value='true', description='启动区域检测节点'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='启动 RViz'
    )

    # ========================================================================
    # 1. 驱动 (Livox)
    # ========================================================================
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 
                'launch_ROS2', 
                'msg_MID360_launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('driver'))
    )

    # ========================================================================
    # 2. 定位与建图
    # ========================================================================
    # 模式: nav -> slam_and_localize (定位 + 里程计)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rm_bringup'), 'launch', 'slam_and_localize.launch.py'])
        ),
        launch_arguments={
            'backend': LaunchConfiguration('backend'),
            'map': LaunchConfiguration('map_pcd'),
            'rviz': LaunchConfiguration('rviz'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_z': LaunchConfiguration('initial_z'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'use_initial_pose': LaunchConfiguration('use_initial_pose'),
            'pcd_save_en': LaunchConfiguration('pcd_save_en'),
            'pcd_save_interval': LaunchConfiguration('pcd_save_interval'),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"])
        )
    )
    
    # 模式: mapping -> slam_mapping_only (建图 + 里程计)
    mapping_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rm_bringup'), 'launch', 'slam_mapping_only.launch.py'])
        ),
        launch_arguments={
            'backend': LaunchConfiguration('backend'),
            'rviz': LaunchConfiguration('rviz')
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"])
        )
    )

    # ========================================================================
    # 3. 地形分析 (仅在 nav 模式下)
    # ========================================================================
    terrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rm_terrain_analysis'), 
                'launch', 
                'traversability_pointcloud.launch.py'
            ])
        ),
        launch_arguments={
            'input_topic': '/cloud_registered_body',
            'rviz': 'false'
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'nav' and '",
                LaunchConfiguration('terrain'), "' == 'true'"
            ])
        )
    )

    # ========================================================================
    # 4. 区域检测 (仅在 nav 模式下，用于检测颠簸区域等)
    # ========================================================================
    region_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rm_terrain_analysis'), 
                'launch', 
                'region_detector.launch.py'
            ])
        ),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'nav' and '",
                LaunchConfiguration('region_detector'), "' == 'true'"
            ])
        )
    )

    # ========================================================================
    # 5. Nav2 导航栈 (仅在 nav 模式下)
    # ========================================================================
    # 等待地图加载？通常 Nav2 栈会处理自己的生命周期。
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_client_cpp'), 
                'launch', 
                'nav2_stack_with_gvc.launch.py'
            ])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_yaml')
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"])
        )
    )

    # ========================================================================
    # 6. 决策模块 (行为树)
    # ========================================================================
    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rm_decision'), 
                'launch', 
                'bt.launch.py'
            ])
        ),
        # Do not open web viewer by default to save resources? 
        # launch_arguments={'use_web_viewer': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('decision'))
    )

    # ========================================================================
    # 7. 通信模块
    # ========================================================================
    comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rm_communication'), 
                'launch', 
                'communication_bringup.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('comm'))
    )

    # ========================================================================
    # 全局可视化
    # ========================================================================
    # 注意: RViz 的启动逻辑已下放到 slam_and_localize 和 slam_mapping_only 中。
    # 它们会根据传入的 'rviz' 参数决定是否启动 RViz。
    # 这样可以复用各模块自带的比较完善的 RViz 配置。

    return LaunchDescription([
        mode_arg,
        backend_arg,
        map_pcd_arg,
        map_yaml_arg,
        pcd_save_en_arg,
        pcd_save_interval_arg,
        initial_x_arg,
        initial_y_arg,
        initial_z_arg,
        initial_yaw_arg,
        use_initial_pose_arg,
        driver_arg,
        comm_arg,
        decision_arg,
        terrain_arg,
        region_detector_arg,
        rviz_arg,

        GroupAction([
            livox_driver,
            localization_launch,
            mapping_launch,
            terrain_launch,
            region_detector_launch,
            nav_launch,
            decision_launch,
            comm_launch,
        ])
    ])
