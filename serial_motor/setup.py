from setuptools import find_packages, setup
from glob import glob

package_name = 'serial_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dmitri Manajev',
    maintainer_email='dmitri.manajev@protonmail.com',
    description='Serial Motor Differential Drive',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_driver = serial_motor.motor_driver:main',
        ],
    },
)
