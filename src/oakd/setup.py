from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'oakd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'oakd'), glob(os.path.join('oakd', 'rewritten_yaml.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mayerfel',
    maintainer_email='felix-max-jakob.mayer@student.uni-tuebingen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
