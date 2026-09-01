from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # packages=['drone_control', 'drone_control.lib','drone_control.utils'],
    packages=find_packages(where="src"),
    package_dir={'': 'src'}   # adjust if your layout is different
)
setup(**d)