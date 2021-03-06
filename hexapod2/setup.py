from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hexapod2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    #packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='je',
    maintainer_email='jeickmann@googlemail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardwarenode = hexapod2.hardwarenode:main',
            'controller = hexapod2.controller.tree:main',
        ],
    },
)
