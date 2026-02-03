from setuptools import setup
import os
from glob import glob

package_name = 'teknofest_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],  # Düz yapı - modül yok
  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarını dahil et
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config dosyalarını dahil et
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'python-socketio[client]',
        'numpy',
        'scipy',
    ],
    zip_safe=True,
    maintainer='havk',
    maintainer_email='mailin@example.com',
    description='Teknofest Kontrol Paketi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_tracking_node = teknofest_control.gps_tracking_node:main',
            'mock_server_bridge = teknofest_control.mock_server_bridge:main',
            'px4_to_mock_bridge = teknofest_control.px4_to_mock_bridge:main',
            'mock_target_receiver = teknofest_control.mock_target_receiver:main',
        ],
    },
)
