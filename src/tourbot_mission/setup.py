from setuptools import find_packages, setup

package_name = 'tourbot_mission'

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
    maintainer='Hyde',
    maintainer_email='108911977+Hyaxon@users.noreply.github.com',
    description='Mission-level control for the TurtleBot4 tour guide robot.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tour_deliberation_node = tourbot_mission.tour_deliberation_node:main',
        ],
    },
)
