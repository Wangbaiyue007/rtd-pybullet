#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='rtdBullet',
      version='0.0.1',
      description='RTD simulation in pybullet',
      author='Baiyue Wang',
      author_email='baiyuew@umich.edu',
      packages = find_packages(),
      install_requires=['pybullet']
     )