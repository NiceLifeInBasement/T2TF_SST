## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
# TODO if you rename the package, change T2TF_SST to the new name (lowercase?!)
setup_args = generate_distutils_setup(
    packages=['T2TF_SST'],
    package_dir={'': 'src'},
)

setup(**setup_args)