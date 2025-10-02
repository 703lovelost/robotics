from setuptools import find_packages, setup

package_name = 'text_to_cmd_vel'

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
    maintainer='a.spirkin',
    maintainer_email='a.spirkin@g.nsu.ru',
    description='Subscriber to /turtle1/cmd_vel to easily control your stupid ahh turtle.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
    		'text_to_cmd_vel = text_to_cmd_vel.text_to_cmd_vel_sub:main',
        ],
    },
)
