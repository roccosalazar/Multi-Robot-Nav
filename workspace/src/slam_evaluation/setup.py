from setuptools import find_packages, setup
from glob import glob

package_name = 'slam_evaluation'

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
    maintainer='ubuntu',
    maintainer_email='rf.salazarbuttiglione@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ground_truth_pose_extractor = slam_evaluation.ground_truth_pose_extractor:main',
            'cslam_odom_tf_bridge = slam_evaluation.cslam_odom_tf_bridge:main',
            'cslam_keyframe_cloud_viewer = slam_evaluation.cslam_keyframe_cloud_viewer:main',
            'slam_pose_publisher = slam_evaluation.slam_pose_publisher:main',
        ],
    },
)
