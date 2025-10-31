from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules = cythonize("kinematics_cython.pyx")
)
