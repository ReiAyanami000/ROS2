from glob import glob
from setuptools import find_packages, setup

setup(
    name='stm32_bridge', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/stm32_bridge']),
        ('share/stm32_bridge', ['package.xml']),
        ('share/stm32_bridge/config', glob('config/*.yaml')),
        ('share/stm32_bridge/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='rei', maintainer_email='Hongli20002@outlook.com',
    description='STM32 serial motion bridge and simulator', license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'stm32_bridge = stm32_bridge.bridge_node:main',
        'fake_stm32 = stm32_bridge.fake_stm32:main',
    ]},
)
