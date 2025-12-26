from setuptools import find_packages, setup
from  glob import glob
package_name = 'obstacle_sim'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),         
        ('share/' + package_name + '/config', glob('config/*.rviz')),         
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='prajwalthakur98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_cylinder =  obstacle_sim.spawn_cylinder:main',
            'obstacle_sim = obstacle_sim.obstacle_sim:main'
        ],
    },
)
