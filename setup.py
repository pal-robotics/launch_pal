from setuptools import find_packages
from setuptools import setup

package_name = 'launch_pal'

setup(
    name=package_name,
    version='0.0.9',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=['Jordan Palacios', 'Noel Jimenez'],
    maintainer_email=['jordan.palacios@pal-robotics.com',
                      'noel.jimenez@pal-robotics.com'],
    description='Launch utilities needed by PAL Robotics software',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
