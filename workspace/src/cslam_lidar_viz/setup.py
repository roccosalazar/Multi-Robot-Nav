from glob import glob
from setuptools import find_packages, setup


package_name = 'cslam_lidar_viz'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='rf.salazarbuttiglione@gmail.com',
    description='RViz2 visualization bridge for Swarm-SLAM/CSLAM LiDAR pose graph and keyframe clouds.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'cslam_lidar_visualizer = cslam_lidar_viz.cslam_lidar_visualizer:main',
        ],
    },
)
