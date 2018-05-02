
from os.path import dirname, basename, isfile
import glob

bmsd = "benchmark_scripts" # benchmark_script_directory (relative)
bmse = ".py" # benchmark_script_extension
bmsel = len(bmse) # benchmark_script_extension_length

dir_path = dirname(__file__) + "/*" + bmse
#print "dir_path:", dir_path

modules = glob.glob(dir_path)
#print "modules:", modules

__all__ = [ basename(f)[:-bmsel] for f in modules if isfile(f) and not f.endswith('__init__.py')]
#print "__all__:", __all__


