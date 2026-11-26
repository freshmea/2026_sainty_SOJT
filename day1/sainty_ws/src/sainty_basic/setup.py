from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sainty_basic'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (share_dir + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='freshmea@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # "명령어=패키지이름.파일이름:함수이름"
            "simple_pub = sainty_basic.simple_pub:main",
            "simple_class_pub = sainty_basic.simple_class_pub:main",
            "move_turtle = sainty_basic.move_turtle:main",
        ],
    },
)
