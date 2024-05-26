from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'brazo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andres',
    maintainer_email='andres.cotrino@uao.edu.co',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["banda = brazo.nodo_banda:main",
                            "camara = brazo.cam_sub:main",
                            "deteccion = brazo.deteccion:main",
                            "control = brazo.control_brazo:main",
                            "prueba = brazo.prueba:main"
        ],
    },
)
