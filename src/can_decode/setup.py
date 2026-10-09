from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'can_decode'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('lib', 'python3.12/site-packages/can_decode'), glob('can_decode/ugv.dbc')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isla',
    maintainer_email='1294644079@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_decode = can_decode.can_decode:main'
        ],
    },
)
