include Config.mk

BULLET_INCDIR = $(BULLET_HOME)/include
FIXED_INCDIR = include

.PHONY: all check install clean distclean python cppyy

all check install: .fix_includes
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

# This is a hack to work around bugs in the current Bullet header files
.fix_includes: fix_includes.patch
	mkdir -p  include
	cp -rp $(BULLET_INCDIR)/bullet $(FIXED_INCDIR)
	cd $(FIXED_INCDIR) && patch -f -p1 < ../fix_includes.patch
	touch .fix_includes
