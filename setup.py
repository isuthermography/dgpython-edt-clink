import os
import sys
import os.path
import shutil
import glob
import numpy as np
from setuptools import setup,Extension
try:
    from setuptools.command.build import build
    pass
except ModuleNotFoundError:
    from distutils.command.build import build
    pass
from setuptools.command.install import install

from Cython.Build import cythonize
#import dataguzzler_python


ext_modules=cythonize(Extension("dgpython_edt_clink.edtcapture",
                                sources=["dgpython_edt_clink/edtcapture.pyx"],
                                libraries=[ ],
                                include_dirs=[np.get_include()]),
                      language_level=3)


class BuildCommand(build):
    user_options = build.user_options + [
        ('with-libedt=',None,'Path to EDT SDK')
    ]
    def initialize_options(self):
        build.initialize_options(self)
        self.with_libedt = None
        pass

    def finalize_options(self):
        if self.with_libedt is not None:
            #for ext in ext_modules:
            build_ext_cmd = self.distribution.get_command_obj("build_ext")
            if not hasattr(build_ext_cmd,"include_dirs") or build_ext_cmd.include_dirs is None:
                build_ext_cmd.include_dirs=[]
                pass
            build_ext_cmd.include_dirs.append(self.with_libedt)
            print(f"include_dirs={str(build_ext_cmd.include_dirs):s}")
            #build_ext_cmd.include_dirs.append(os.path.join(self.with_azurekinect,'sdk','include'))
            if not hasattr(build_ext_cmd,"library_dirs") or build_ext_cmd.library_dirs is None:
                build_ext_cmd.library_dirs=[]
                pass

            build_ext_cmd.library_dirs.append(os.path.join(self.with_libedt,'lib'))
        #    if sys.platform=="win32":
        #        build_ext_cmd.library_dirs.append(os.path.join(self.with_azurekinect,'sdk','windows-desktop','amd64','release','lib'))
        #        pass
        #    if not hasattr(build_ext_cmd,"rpath") or build_ext_cmd.rpath is None:
        #        build_ext_cmd.rpath=[]
        #        pass
        #    if sys.platform != "win32":
        #        build_ext_cmd.rpath.insert(0,os.path.join(self.with_azurekinect,'lib'))            
        #        build_ext_cmd.rpath.insert(0,"$ORIGIN")
        #        pass
            
        #    pass
        build.finalize_options(self)
        #import pdb
        #pdb.set_trace()
        pass

    def run(self):
        print("with_libedt=%s" % (self.with_libedt))
        print("ext[0].include_dirs=%s" % (str(ext_modules[0].include_dirs)))
        build.run(self)
        pass
    pass


class InstallCommand(install):
    user_options = install.user_options + [
        ('with-libedt=',None,'Path to EDT SDK')
    ]
    def initialize_options(self):
        install.initialize_options(self)
        self.with_libedt = None
        pass

    def finalize_options(self):
        if self.with_libedt is not None:
            pass
        install.finalize_options(self)
        pass

    def run(self):
        #print("with_azurekinect=%s" % (self.with_azurekinect))
        #print("ext[0].include_dirs=%s" % (str(ext_modules[0].include_dirs)))
        install.run(self)
        #if sys.platform=="win32":
        #    # install Azure Kinect DLL's in with our driver
        #    print("Installing Azure Kinect DLLs into %s" % (self.install_lib))
        #    dlldir = os.path.join(self.with_azurekinect,'sdk','windows-desktop','amd64','release','bin')
        #    depthengine_dlls = glob.glob(os.path.join(dlldir,"depthengine*.dll"))
        #    for depthengine_dll in depthengine_dlls:
        #        source = depthengine_dll
        #        destination = os.path.join(self.install_lib,"dgpython_azurekinect",os.path.split(depthengine_dll)[1])
        #        print("  %s -> %s" % (source,destination))
        #        shutil.copyfile(source,destination)
        #        pass
        #    for source in ["k4a.dll","k4a.pdb","k4arecord.dll","k4arecord.pdb"]:
        #        sourcepath = os.path.join(dlldir,source)
        #        destination = os.path.join(self.install_lib,"dgpython_azurekinect",source)
        #        print("  %s -> %s" % (sourcepath,destination))
        #        shutil.copyfile(sourcepath,destination)
        #        pass
        #    pass
        pass
    pass


setup(name="dgpython_edt_clink",
      description="edt camera link for dgpython",
      author="Stephen D. Holland",
      url="http://thermal.cnde.iastate.edu",
      ext_modules=ext_modules,
      zip_safe=False,
      packages=["dgpython_edt_clink"],
      cmdclass = { 'build': BuildCommand, "install": InstallCommand })
