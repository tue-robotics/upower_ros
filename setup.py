from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['upower_ros'],
    package_dir={'': 'src'},
    scripts=['scripts/battery_node']
)

setup(**d)
