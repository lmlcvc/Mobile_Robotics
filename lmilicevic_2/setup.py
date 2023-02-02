import os
from setuptools import setup
from glob import glob

package_name = 'lmilicevic_2'

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
    description='Z02 iz Mobilne robotike',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = lmilicevic_2.navigator:main'
        ],
    },
)
