from setuptools import find_packages, setup

package_name = 'insta360_tools'

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
    maintainer='dinethra',
    maintainer_email='rajapakshaipdd.21@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    	'console_scripts': [
        	'insta360_cropper = insta360_tools.insta360_cropper:main',
    	],
    },
)
