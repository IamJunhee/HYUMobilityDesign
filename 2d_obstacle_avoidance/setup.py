from setuptools import find_packages, setup

package_name = '2d_obstacle_avoidance'

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
    maintainer='mobility',
    maintainer_email='junhee0110@hanyang.ac.kr',
    description='Automatic flight with 2D obstacle avoidance using LiDAR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "automatic_flight_controller = 2d_obstacle_avoidance.automatic_flight_controller:main"
        ],
    },
)
