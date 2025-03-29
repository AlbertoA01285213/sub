from setuptools import find_packages
from setuptools import setup

setup(
    name='sub',
    version='0.0.0',
    packages=find_packages(
        include=('sub', 'sub.*')),
)
