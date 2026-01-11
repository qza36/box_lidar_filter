# Box Lidar Filter
Use `pcl::CropBox` to crop lidar pointclouds
## Why
- 由于`stvl`无法显式设置最小障碍物半径，所以开发此包裁切源点云以避免将机器人自身注册成障碍物
- Since stvl does not allow explicitly setting a minimum obstacle radius, this package was developed to crop the source point cloud, preventing the robot itself from being registered as an obstacle.
## How to use 
```bash
Node(
    package='box_lidar_filter',
    executable='lidar_filter_node',
    name='box_lidar_filter',
    output='screen',
    parameters=[{
        'input_topic': '/livox/lidar',              
        'output_topic': '/livox/lidar_filtered',
        'min_x': -0.2, 'max_x': 0.2,                #坐标系符合 REP 103，右手坐标系，原点在base_link
        'min_y': -0.2, 'max_y': 0.4,
        'min_z': -0.1, 'max_z': 0.2,
        'negative': True,                           # 挖掉车身
        #'leaf_size': 0.05                           # 降采样 ，降采样未启用 
    }]
),
```