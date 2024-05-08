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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple_algorithm = 2d_obstacle_avoidance.simple_algorithm:main"
        ],
    },
)
