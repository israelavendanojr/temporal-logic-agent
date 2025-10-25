from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'uav_ltl_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Include models directory (but not the actual model files)
        (os.path.join('share', package_name, 'models'), 
 glob('models/translation_model.gguf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='UAV Task Planner with LTL Translation Agent for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor = uav_ltl_planner.ros2_nodes.mission_executor_node:main',
            'state_monitor = uav_ltl_planner.ros2_nodes.state_monitor_node:main',
        ],
    },
)
