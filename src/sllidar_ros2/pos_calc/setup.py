from setuptools import find_packages, setup

package_name = 'pos_calc'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'publish_from_arduino = pos_calc.publish_from_arduino:main',
		'pitch2groundlength = pos_calc.pitch2groundlength:main',
		'position_calculation = pos_calc.position_calculation:main',
		'are_we_moving = pos_calc.are_we_moving:main',
		'lidar_subscriber = pos_calc.lidar_subscriber:main',
        ],
    },
)
