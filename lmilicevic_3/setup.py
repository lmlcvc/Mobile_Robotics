import os
from setuptools import setup
from glob import glob

package_name = 'lmilicevic_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lana Milicevic',
    maintainer_email='lana.milicevic10@gmail.com',
    description='Z03 iz Mobilne robotike',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotator = lmilicevic_3.rotator:main'
        ],
    },
)