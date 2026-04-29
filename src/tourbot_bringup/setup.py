from setuptools import find_packages, setup
from pathlib import Path
import os

package_name = 'tourbot_bringup'
here = Path(__file__).resolve().parent

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (os.path.join('share', package_name), ['package.xml']),
]

# Launch files
launch_root = here / 'launch'
if launch_root.exists():
    launch_files = [
        str(p.relative_to(here))
        for p in launch_root.glob('*.py')
    ]
    if launch_files:
        data_files.append(
            (
                os.path.join('share', package_name, 'launch'),
                launch_files,
            )
        )

# Config files, preserving subdirectories
config_root = here / 'config'
if config_root.exists():
    for path in config_root.rglob('*'):
        if path.is_file():
            rel_parent = path.parent.relative_to(config_root)
            install_dir = os.path.join(
                'share',
                package_name,
                'config',
                str(rel_parent),
            )
            data_files.append((install_dir, [str(path.relative_to(here))]))

# Map files, preserving subdirectories
maps_root = here / 'maps'
if maps_root.exists():
    for path in maps_root.rglob('*'):
        if path.is_file():
            rel_parent = path.parent.relative_to(maps_root)
            install_dir = os.path.join(
                'share',
                package_name,
                'maps',
                str(rel_parent),
            )
            data_files.append((install_dir, [str(path.relative_to(here))]))

# World files, preserving subdirectories
worlds_root = here / 'worlds'
if worlds_root.exists():
    for path in worlds_root.rglob('*'):
        if path.is_file():
            rel_parent = path.parent.relative_to(worlds_root)
            install_dir = os.path.join(
                'share',
                package_name,
                'worlds',
                str(rel_parent),
            )
            data_files.append((install_dir, [str(path.relative_to(here))]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyde',
    maintainer_email='108911977+Hyaxon@users.noreply.github.com',
    description='Bringup package for the TurtleBot4 tour guide robot.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)