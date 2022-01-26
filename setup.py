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
                'scripts.wsr_module',
                sources=['Cython_modules/wsr_module.pyx'],
                include_dirs=['include',
                              'include/csitoolbox',
                              'src',
                              os.path.expanduser('~')+'/Downloads/boost_1_68_0'],
                language="c++",
		            extra_compile_args=['-std=c++14', '-O3', '-fopenmp'],
                extra_link_args=['-lgomp', '-lpthread', '-lm', '-ldl']
    )))
