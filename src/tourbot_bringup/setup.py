from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tourbot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        #(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        #(os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyde',
    maintainer_email='108911977+Hyaxon@users.noreply.github.com',
    description='Bringup package for the TurtleBot4 tour guide robot.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
