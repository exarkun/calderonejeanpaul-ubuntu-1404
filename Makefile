include Config.mk

BULLET_INCDIR = $(BULLET_HOME)/include
FIXED_INCDIR = include

.PHONY: all check install clean distclean python cppyy

all check install:
	$(MAKE) -C cppyy $@
	$(MAKE) -C python $@

clean distclean: 
	$(MAKE) -C cppyy $@
	$(MAKE) -C python $@
	rm -rf $(FIXED_INCDIR) .fix_includes

python: 
	$(MAKE) -C python all

cppyy:
	$(MAKE) -C cppyy all

