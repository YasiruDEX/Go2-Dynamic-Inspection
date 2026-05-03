from setuptools import find_packages, setup

package_name = 'mola_mqtt_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt>=1.6.1',
    ],
    zip_safe=True,
    maintainer='Yasiru',
    maintainer_email='yasiru@example.com',
    description='ROS2 ↔ MQTT bridge for MOLA LiDAR-Odometry telemetry',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mola_mqtt_publisher = mola_mqtt_bridge.mola_mqtt_publisher:main',
        ],
    },
)
