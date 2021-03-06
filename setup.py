#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import nested_scopes, generators, with_statement, unicode_literals, absolute_import, division, print_function

import os
from setuptools import setup, find_packages

def read(fname):
	'''open the long description of the project'''
	return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
	name='elevator',
	version='0.0.3',
	author='Niklas Beuster',
	author_email='niklas.beuster@tu-ilmenau.de',
	description=('Python interface for a linear actuator with serial interface'),
	license='Beer license',
	keywords='positioner elevator lift linear',
	url='https://git01.iis.fhg.de/dvt-forte/Elevator',
	packages=find_packages(),
	long_description=read('README.md'),
	install_requires=['pyserial>=3.0'],
	classifiers=[
		'Development Status :: 1 - Alpha',
		'Topic :: Utilities',
		'License :: Me :: proprietary',
	],
)
