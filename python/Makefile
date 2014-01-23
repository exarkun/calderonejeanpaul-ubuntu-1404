include ../Config.mk

.PHONY: all check clean distclean install

all: bullet/__init__.py

check: all
	LD_LIBRARY_PATH=../cppyy pypy -c "from bullet import bt; print '--- Import Check OK'"

clean:
	$(RM) bullet/*.pyc

distclean: clean
	$(RM) bullet/__init__.py

#FIXME: This is not really a proper Python package install procedure.
install: all
	mkdir -p $(SITE_PACKAGES)/bullet
	cp bullet/*.py $(SITE_PACKAGES)/bullet

%.py: %.py.template ../cppyy/cppyy_bullet.so
	../cppyy/util/symbols_to_python --template=$< ../cppyy/cppyy_bullet.so > $@
