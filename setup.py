from setuptools import setup, find_packages
import os
import glob



package_name = 'riact_camera_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohamed',
    maintainer_email='ma@riact.eu',
    description='TODO: Package description',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_publisher = riact_camera_realsense.pointcloud_publisher:main', 
            'pointcloud_subscriber = riact_camera_realsense.pointcloud_subscriber:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*_launch.py'))
    ],
)
