import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'map_change'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kuri-tadaoki',
    maintainer_email='kuri-tadaoki@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'change = map_change.map_change:main',
            'initial_pose_pub = map_change.initial_pose_publisher:main',
        ],
    },
)
