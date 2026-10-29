from setuptools import setup
from glob import glob
setup(name='restaurant_sim',version='0.1.0',packages=['restaurant_sim'],data_files=[('share/ament_index/resource_index/packages',['resource/restaurant_sim']),('share/restaurant_sim',['package.xml'])]+[('share/restaurant_sim/'+d,glob(d+'/*')) for d in ['launch','config','worlds','docs']],entry_points={'console_scripts':['people = restaurant_sim.controller:main']})
