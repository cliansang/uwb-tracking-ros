from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'uwb_tracking_ros2'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    py_modules=[
        'uwb_tracking_ros2.dwm1001_apiCommands'
    ],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index        
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),              
        # (os.path.join('share', package_name, 'resource'), glob('resource/*')),          
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'cfg'), glob(os.path.join('cfg', '*.yaml'))),
        (os.path.join('share', package_name, 'msg'), glob(os.path.join('msg', '*.msg'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Lauritz Keysberg, Cung Lian Sang',
    author_email='lkeysberg@techfak.uni-bielefeld.de, csang@techfak.uni-bielefeld.de',
    maintainer='ROS 2 Developer',
    maintainer_email='ros2@ros.com',
    description='UWB Tracking in ROS2.',
    license='MIT',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'uwb_tracking_dwm1001 = uwb_tracking_ros2.uwb_tracking_dwm1001:main',
            'viz_dwm1001 = uwb_tracking_ros2.viz_dwm1001:main',
            # 'uwb_tracking_trek1000 = uwb_tracking_ros.uwb_tracking_trek1000:main',
            # Example: 'node_name = package.module:function'
        ],
    },
)