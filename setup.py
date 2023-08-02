from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['pepper_bt','util'],
    package_dir={'': 'src'},
    scripts=['scripts/main'],
 #   requires=['genpy', 'numpy', 'rosgraph', 'roslib', 'rospkg']
)

setup(**setup_args)