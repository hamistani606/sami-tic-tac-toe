import os
from setuptools import find_packages, setup
from glob import glob


package_name = 'sami_ttt'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
        (os.path.join('share', package_name, 'assets'), glob('assets/**/*', recursive=True)),
        (os.path.join('share', package_name, 'behaviorbank'), glob('behaviorbank/test1.py')),
        (os.path.join('share', package_name, 'behaviorbank'), glob('behaviorbank/test2.py')),
        (os.path.join('share', package_name, 'behaviorbank'), glob('behaviorbank/dummytest1.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyle Vickstrom',
    maintainer_email='vickskyl@oregonstate.edu',
    description='ROB421 Project: SAMI Tic Tac Toe',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ttt_console = sami_ttt.ttt_game_console:createConsole',
            'ttt_game = sami_ttt.ttt_game:createGame',
            'misty_control = sami_ttt.misty_control:createMisty',
            'ttt_gui = sami_ttt.ttt_gui:main'
            'misty_testread = sami_ttt.misty_control:dummymovetest1',
        ],
    },
)
