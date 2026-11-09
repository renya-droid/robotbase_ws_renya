from setuptools import setup
from glob import glob
import os
# Files only: a test importing a launch file leaves launch/__pycache__, which broke the copy.
setup(tests_require=['pytest'],name='restaurant_sim',version='0.1.0',packages=['restaurant_sim'],data_files=[('share/ament_index/resource_index/packages',['resource/restaurant_sim']),('share/restaurant_sim',['package.xml'])]+[('share/restaurant_sim/'+d,[f for f in glob(d+'/*') if os.path.isfile(f)]) for d in ['launch','config','worlds','docs']],entry_points={'console_scripts':['people = restaurant_sim.controller:main']})
