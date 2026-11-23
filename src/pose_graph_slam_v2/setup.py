from glob import glob
from setuptools import find_packages, setup

package_name = 'pose_graph_slam_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bao',
    maintainer_email='23020783@vnu.edu.vn',
    description='Implementation Of Pose-Graph SLAM V2 (Scan-To-Map ICP And Huber Loss Optimization)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_graph_slam_v2_node = pose_graph_slam_v2.pose_graph_slam_v2_node:main',
            'occupancy_grid_map_v2_node = pose_graph_slam_v2.occupancy_grid_map_v2_node:main',
        ],
    },
)
