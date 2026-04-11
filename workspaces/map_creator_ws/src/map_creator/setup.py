from setuptools import setup

package_name = 'map_creator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Map creator python node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_creator_node = map_creator.map_creator_node:main'
        ],
    },
)
