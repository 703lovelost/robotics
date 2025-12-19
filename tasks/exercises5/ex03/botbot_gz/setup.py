from setuptools import setup

package_name = 'botbot_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/botbot_gz.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot_gz.urdf']),
        ('share/' + package_name + '/rviz', ['rviz/urdf_config.rviz']),
        ('share/' + package_name + '/worlds', ['worlds/empty.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Botbot Gazebo Sim bringup with diff drive and ROS-Gazebo bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)