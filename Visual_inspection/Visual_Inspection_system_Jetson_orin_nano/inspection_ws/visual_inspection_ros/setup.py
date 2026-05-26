from setuptools import setup
from glob import glob

package_name = 'visual_inspection_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.bt_nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dinethra',
    description='Visual inspection IBVS pipeline as ROS2 nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Add only nodes that exist — add more as files are created
            'camera_node         = visual_inspection_ros.camera_node:main',
            'servo_node          = visual_inspection_ros.servo_node:main',
            'ibvs_action_server  = visual_inspection_ros.ibvs_action_server:main',
            'run_inspection_bt   = visual_inspection_ros.bt_nodes.inspection_bt_nodes:main',
        ],
    },
)
