import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pose_graph_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
                (os.path.join('share', package_name, 'rviz2'), glob(os.path.join('rviz2', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bao',
    maintainer_email='23020783@vnu.edu.vn',
    description='Implementation Of Pose-Graph SLAM With EKF Sensor Fusion Odometry From Create3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_graph_slam_node = pose_graph_slam.pose_graph_slam_node:main',
            'occupancy_grid_map_node = pose_graph_slam.occupancy_grid_map_node:main',
        ],
    },
)
