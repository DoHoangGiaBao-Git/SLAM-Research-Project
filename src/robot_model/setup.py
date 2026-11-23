from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'meshes'), glob('urdf/meshes/*.stl')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bao',
    maintainer_email='23020783@vnu.edu.vn',
    description='Robot Model Definition Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_base_node = robot_model.robot_base_node:main',
            'robot_feedback_node = robot_model.robot_feedback_node:main',
        ],
    },
)
