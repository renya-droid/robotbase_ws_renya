from glob import glob
import os
from setuptools import setup

name = 'restaurant_nav_experiment'


def files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]


setup(
    name=name, version='0.1.0', packages=[name], tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/launch', files('launch/*')),
        ('share/' + name + '/generated', files('generated/*')),
        ('share/' + name + '/generated/worlds', files('generated/worlds/*')),
        ('share/' + name + '/generated/maps', files('generated/maps/*')),
        ('share/' + name + '/scripts', files('scripts/*')),
    ],
    entry_points={'console_scripts': [
        'people_bridge = restaurant_nav_experiment.people_bridge:main',
        'serving_runner = restaurant_nav_experiment.serving_runner:main',
        'twist_mux_lite = restaurant_nav_experiment.twist_mux_lite:main',
        'ground_truth_tf = restaurant_nav_experiment.ground_truth_tf:main',
    ]},
)
