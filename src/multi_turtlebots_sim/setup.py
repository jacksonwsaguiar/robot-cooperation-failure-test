from setuptools import setup
import os
from glob import glob

package_name = 'multi_turtlebots_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    # This section is crucial for simulation packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install the package.xml
        ('share/' + package_name, ['package.xml']),
        # Install all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install all world files from the 'worlds' directory
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Install all robot description files (xacro, urdf) from the 'urdf' directory
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Install all 3D mesh files from the 'meshes' directory
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pallav',
    maintainer_email='jackson.aguiar@email.com',
    description='Package for simulating multiple TurtleBots in Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any Python nodes here if you have them
            # 'my_sim_node = multi_turtlebots_sim.my_script:main',
        ],
    },
)