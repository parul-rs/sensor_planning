from setuptools import setup
import os
from glob import glob

package_name = 'nbv_space'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install meshes, urdf, launch, etc.
        (os.path.join('share', package_name, 'meshes'),
        [f for f in glob('meshes/**/*', recursive=True) if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'worlds'),
        [f for f in glob('worlds/**/*', recursive=True) if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Parul Singh',
    maintainer_email='prsingh@utexas.edu',
    description='TODO',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_dynamics.py = nbv_space.target_dynamics:main',
        ],
    },
)
