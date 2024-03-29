import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'signal_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arturomurra',
    maintainer_email='arturo.murra@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_generator = signal_challenge.signal_generator:main',
            'signal_processor = signal_challenge.signal_processor:main'
        ],
    },
)
