import glob
import os

from setuptools import find_packages, setup

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob.glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'),
            glob.glob('maps/*')),
        (os.path.join('share', package_name, 'models', 'actor', 'meshes'),
            glob.glob('models/actor/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f',
    maintainer_email='f@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	    'color_detector = robot_perception.color_detector:main',
	    'target_navigator = robot_perception.target_navigator:main',
	    'yolo_detector = robot_perception.yolo_detector:main',
        ],
    }
)
