dgpython-edt-clink
--------------------

This is a Dataguzzler-Python/SpatialNDE2 driver for the EDT camera link  frame grabber.



Installation
------------

This package requires the EDT SDK (uses the C
API), Dataguzzler-Python, Cython, and SpatialNDE2.

The dgpython-edt-clink package installs with the usual
"pip install .", but you probably need to tell it where to find the
EDT SDK files (see below). Make sure that you install into
the same Python environment you are using for SpatialNDE2 and
Dataguzzler-Python

To tell the setup script where to find the EDT SDK files,
you need to either modify setup.cfg configuring the path to the SDK
in the with-libpdv parameter:

[build]
with-libpdv=/opt/EDTpdv

or you can create a new
setup_local.cfg (similar to setup.cfg) with your path configured.
If you do the latter, you must set the DIST_EXTRA_CONFIG environment
variable to point at your setup_local.cfg, as illustrated in
setupcmd.sh (Linux) and setupcmd.bat (Windows).

To perform the install, if you modified setup.cfg you can just
(possibly as root or administrator):

pip install --no-deps --no-build-isolation .

If you created setup_local.cfg, instead run:

sudo ./setupcmd.sh   (Linux; central install)

or,

./setupcmd.sh   (Linux; user install)

or,

.\setupcmd.bat  (Windows)  



NOTE: If you upgrade spatialnde2 it is highly recommended that, after
rebuilding and performing the Python reinstall ("python setup.py
install" from the spatialnde2 build directory) that you clean out your
dgpython-edt-clink build/ and dist/ directories and do a full
reinstall, e.g. from the dgpython-edt-clink directory:

      WINDOWS:
            rmdir /s build dest
            setupcmd.bat
      LINUX:
            rm -r build/ dest/
            bash setupcmd.sh

Usage
-----





