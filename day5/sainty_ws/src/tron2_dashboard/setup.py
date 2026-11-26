from glob import glob
from setuptools import setup

package = 'tron2_dashboard'
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
    description='TRON2 operator joint panel, RViz and graph.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'joint_panel = tron2_dashboard.joint_panel:main',
    ]},
)
