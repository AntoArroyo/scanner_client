from setuptools import find_packages, setup

package_name = 'robot_wifi_client'

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
    maintainer='AntoArroyo',
    maintainer_email='arroyo.antom@gmail.com',
    description='TODO: Package description',
    license='GNU GENERAL PUBLIC LICENSE Version 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_client = robot_wifi_client.client:main',
        ],
    },
)
