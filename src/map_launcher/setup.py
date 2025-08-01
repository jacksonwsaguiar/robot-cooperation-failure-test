from setuptools import setup
import os
from glob import glob

package_name = 'map_launcher'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instala todos os arquivos da pasta 'launch' (se você criar uma)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Instala todos os arquivos da pasta 'map'
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        # Instala todos os arquivos da pasta 'rviz'
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu_email@exemplo.com',
    description='Pacote para lançar mapas e configurações do Rviz.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)