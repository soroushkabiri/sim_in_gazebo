from setuptools import find_packages, setup

package_name = 'comp_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_yaw.launch.py']),
        #('share/' + package_name + '/launch', ['launch/combined.launch.py']),
        ('share/' + package_name + '/launch', ['launch/velocity_publishing.launch.py']),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soroush',
    maintainer_email='soroush.kabiri.92@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'imu_yaw_node = comp_pkg.imu_yaw_node:main',
                    'initial_align_node = comp_pkg.initial_allignment_node:main',
                    'test_node = comp_pkg.test_node:main',
                    'state_consensus_node = comp_pkg.state_consensus_node:main',
                    'pub_des_vel_node = comp_pkg.pub_des_vel_node:main',
                    
                    
                    'imu_to_odom = comp_pkg.imu_to_odom:main',
                    'gazebo_to_odom = comp_pkg.gazebo_to_odom:main',
                    'laser_relay = comp_pkg.laser_relay:main',





        ],
    },
)
