'''
(c) REACT LAB, Harvard University
	Author: Ninad Jadhav, Weiying Wang
'''

#!/usr/bin/env python

from setuptools import setup, Extension
from Cython.Build import cythonize
import sysconfig
import os

extra_compile_args = sysconfig.get_config_var('CFLAGS').split()

setup(ext_modules=cythonize(Extension(
                'scripts.libs.wsr_module',
                sources=['Cython_modules/wsr_module.pyx'],
                include_dirs=['include','include/csitoolbox','src'],
                language="c++",
		extra_compile_args=['-std=c++14']
    )))
