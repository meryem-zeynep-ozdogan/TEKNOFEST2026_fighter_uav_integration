from setuptools import setup
import os
from glob import glob

package_name = 'teknofest_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],  # Düz yapı - modül yok
    py_modules=[
        'gps_tracking_node',
        'mock_server_bridge',
        'px4_to_mock_bridge',
    ],
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
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='HAVK Team',
    maintainer_email='teknofest@havk.team',
    description='TEKNOFEST Savaşan İHA - GPS Takip ve Kontrol Sistemi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_tracking_node = gps_tracking_node:main',
            'mock_server_bridge = mock_server_bridge:main',
            'px4_to_mock_bridge = px4_to_mock_bridge:main',
        ],
    },
)
