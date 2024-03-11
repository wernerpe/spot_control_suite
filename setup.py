from setuptools import setup, find_packages

setup(
    name='spot_control_suite',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        # list your dependencies here
        'numpy>=1.26.4',
        'scipy>=1.12.0',
    ],
    package_data={
        'spot_control_suite': ['assets/*'],
    },
)