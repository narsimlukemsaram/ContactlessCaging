from setuptools import setup
from glob import glob
import os
package_name = 'contactlesscaging_control'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ContactlessCaging research team',
    maintainer_email='narsimlu.kemsaram@um.edu.my',
    description='ROS 2 PhaseSpace pose, differential-drive formation control and reference trajectory',
    license='Licence to be supplied by repository maintainers',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'formation_controller = contactlesscaging_control.formation_controller:main',
        'phasespace_udp_bridge = contactlesscaging_control.phasespace_udp_bridge:main',
        'transport_reference = contactlesscaging_control.transport_reference:main',
    ]},
)
