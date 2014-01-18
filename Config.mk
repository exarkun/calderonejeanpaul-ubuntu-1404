REFLEXHOME = /usr/local
BULLET_HOME = /usr

# The following attempts to run 'pypy' to determine dynamically the correct
# place to install site packages (this should work with virtualenv, etc, too).
# If this doesn't work for some reason, you may need to just explicitly set
# SITE_PACKAGES to your appropriate PyPy site-packages path.
SITE_PACKAGES = $(shell pypy -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")
