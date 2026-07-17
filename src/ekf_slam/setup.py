from setuptools import find_packages, setup

package_name = 'ekf_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vatcraft',
    maintainer_email='23020783@vnu.edu.vn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry = ekf_slam.odometry:main',
            'landmark_extraction = ekf_slam.landmark_extraction:main',
            'ekf_slam = ekf_slam.ekf_slam:main'
        ],
    },
)
