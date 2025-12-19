from setuptools import setup

package_name = 'botbot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_display.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
        ('share/' + package_name + '/rviz', ['rviz/urdf_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='URDF display package for botbot',
    license='Apache-2.0',
)