from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_disutils_setup(
    packages=['taurob_explorer'],
    package_dir=['':'src' 'cfg'],
)

setup(**setup_args)