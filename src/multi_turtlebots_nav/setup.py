from setuptools import setup
import os
from glob import glob

package_name = 'multi_turtlebots_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Instala o package.xml
        ('share/' + package_name, ['package.xml']),
        # Instala os arquivos de lançamento (launch files)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Instala os arquivos de configuração (se houver)
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jackson Aguiar',
    maintainer_email='jackson.aguiar@email.com',
    description='Pacote para navegação de múltiplos TurtleBots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Aqui você define seus nós executáveis
            # Formato: 'nome_do_executavel = nome_do_pacote.nome_do_script:main'
            # Exemplo: 'meu_no_nav = multi_turtlebots_nav.script_de_nav:main',
        ],
    },
)