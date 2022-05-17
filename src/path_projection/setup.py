from setuptools import setup
from glob import glob
import os


package_name = 'path_projection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.model')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.pgm')),    
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),    
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PathProjection = path_projection.path_projection_node:main',
        ],
    },
)
