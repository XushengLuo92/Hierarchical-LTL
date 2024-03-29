from setuptools import setup, find_packages

setup(
    name='hierarchical_ltl',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        'gurobipy==10.0.2',
        'matplotlib==3.8.1',
        'networkx==3.1',
        'numpy==1.25.2',
        'sympy==1.12',
        'termcolor==2.3.0',
    ],
    author='Xusheng Luo',
    author_email='xusheng.luo2@gmail.com',
    description='Cource code for the paper "Robotic Planning under Hierarchical Temporal Logic Specifications"',
    license='MIT',
    url='https://github.com/XushengLuo92/Hierarchical-LTL',
)
