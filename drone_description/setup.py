import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'drone_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
            (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
            (os.path.join('share', package_name), glob('urdf/*')),
            ]
)