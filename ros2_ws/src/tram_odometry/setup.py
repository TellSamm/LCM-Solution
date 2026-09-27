from setuptools import setup
from glob import glob
package_name = 'tram_odometry'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/assets', glob('assets/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='LCM Solution', maintainer_email='chernikov.semen21@gmail.com',
    description='Model-based backup odometry for an autonomous tram',
    license='MIT',
    entry_points={'console_scripts': ['odometry_node = tram_odometry.node:main']},
)
