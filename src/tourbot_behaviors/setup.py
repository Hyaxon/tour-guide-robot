from setuptools import find_packages, setup

package_name = 'tourbot_behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/door_behavior.launch.py']),
        ('share/' + package_name + '/config', ['config/door_behavior.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyde',
    maintainer_email='108911977+Hyaxon@users.noreply.github.com',
    description='Behavior nodes for the tourbot.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'door_behavior_server = tourbot_behaviors.door_behavior_server:main',
            'manual_door_override_node = tourbot_behaviors.manual_door_override_node:main',
        ],
    },
)
