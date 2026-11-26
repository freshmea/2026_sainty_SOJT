from setuptools import setup

setup(
    name="sainty_knee",
    version="0.1.0",
    packages=["sainty_knee"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sainty_knee"]),
        ("share/sainty_knee", ["package.xml"]),
        ("share/sainty_knee/launch", ["launch/knee.launch.py"]),
        ("share/sainty_knee/urdf", ["urdf/leg.urdf"]),
    ],
    entry_points={"console_scripts": [
        "knee_source = sainty_knee.source:main",
        "knee_watch = sainty_knee.watch:main",
    ]},
)
