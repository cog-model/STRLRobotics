from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import pip
pip.main(['install', '-r', 'requirements.txt'])
d = generate_distutils_setup(
    packages=['skill_bot',
              'skill_bot.demo',
              'skill_bot.primitives', 
              'skill_bot.robot', 
              'skill_bot.utils',
              'skill_bot.planner'],
)
setup(**d)