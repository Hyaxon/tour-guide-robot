from setuptools import find_packages, setup
from pathlib import Path
import os

package_name = 'tourbot_landmarks'
here = Path(__file__).resolve().parent
config_root = here / 'config'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (os.path.join('share', package_name), ['package.xml']),
]

if config_root.exists():
    for map_dir in config_root.iterdir():
        if map_dir.is_dir():
            yaml_files = [
                str(p.relative_to(here))
                for p in map_dir.glob('*.yaml')
            ]
            if yaml_files:
                data_files.append(
                    (
                        os.path.join('share', package_name, 'config', map_dir.name),
                        yaml_files,
                    )
                )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyde',
    maintainer_email='108911977+Hyaxon@users.noreply.github.com',
    description='Landmark location loader for the TurtleBot4 tour guide robot.',
    license='MIT',
)