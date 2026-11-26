from glob import glob
from setuptools import setup

package = 'tron2_bringup'
setup(
    name=package,
    version='0.1.0',
    packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml']),
        ('share/' + package + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='aa',
    maintainer_email='freshmea@gmail.com',
    description='TRON2 simulation bringup and SDK gateway.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mujoco_sim = tron2_bringup.mujoco_sim:main',
    ]},
)
