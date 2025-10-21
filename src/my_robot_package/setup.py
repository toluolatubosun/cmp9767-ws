from setuptools import find_packages, setup

package_name = 'my_robot_package'

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
    maintainer='ros',
    maintainer_email='toluolatubosun@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_circle = my_robot_package.move_circle_publisher:main',
            'move_square = my_robot_package.move_square_publisher:main',
            'closest_laser = my_robot_package.closest_laser_point_publisher:main',
            'custom_transform_listener = my_robot_package.custom_transform_listener:main',
        ],
    },
)
