include ../Config.mk

APP_INCDIRS = ../include/bullet $(BULLET_HOME)/include/bullet
APP_LIBDIRS = $(BULLET_HOME)/lib
APP_LIBS = -lBulletDynamics -lBulletCollision -lLinearMath
TARGETS = cppyy_bullet.so
INSTALL_DIR = $(SITE_PACKAGES)/bullet

GENREFLEX_FLAGS = 

# Note: From here on should not need to be tweaked for most purposes.

comma = ,

GENREFLEX = $(REFLEXHOME)/bin/genreflex
GENREFLEX_FILTER = util/genreflex_filter
CPPYYCHECK = util/cppyy_check 

APP_INCLUDES = $(patsubst %,-I%,$(APP_INCDIRS))
CPPFLAGS = -I$(REFLEXHOME)/include $(APP_INCLUDES)
LDFLAGS = -L$(REFLEXHOME)/lib $(patsubst %,-L%,$(APP_LIBDIRS)) $(patsubst %,-Wl$(comma)-rpath$(comma)%,$(APP_LIBDIRS))
LIBS = -lReflex $(APP_LIBS)
CFLAGS = -O2
SOFLAGS = -fPIC -rdynamic -shared
GENREFLEX_INCLUDES = $(patsubst %,--gccxmlopt=%,$(APP_INCLUDES))
ROOTMAP_TARGETS = $(TARGETS:.so=.rootmap)
ALL_TARGETS = $(TARGETS) $(ROOTMAP_TARGETS)
INTERMEDIATES = $(TARGETS:.so=.cpp) $(TARGETS:.so=.list) $(TARGETS:.so=.gccxml)

LDLIBPATH = $(subst $(space),:,$(APP_LIBDIRS)):$(REFLEXHOME)/lib

.PHONY: all check clean distclean

all: $(ALL_TARGETS)

check: all $(TARGETS:.so=..check)

install: all
	mkdir -p $(INSTALL_DIR)
	cp cppyy_bullet.so $(INSTALL_DIR)
	cp cppyy_bullet.rootmap $(INSTALL_DIR)

clean:
	rm -f $(INTERMEDIATES)

distclean: clean
	rm -f $(ALL_TARGETS)

%.cpp %.rootmap %.list: %.h %.lcgdict
	$(GENREFLEX) $< -s $*.lcgdict -o $*.cpp --rootmap=$*.rootmap --rootmap-lib=$*.so $(GENREFLEX_INCLUDES) $(GENREFLEX_FLAGS) | $(GENREFLEX_FILTER) $*.list

%.so: %.cpp
	$(CXX) $(CFLAGS) $(SOFLAGS) $(CPPFLAGS) $(LDFLAGS) $< -o $@ $(LIBS)

# The "check" targets really should be declared phony, but GNU Make doesn't
# support that with pattern rules.. sigh.
%..check: %.so %.list
	LD_LIBRARY_PATH=$(LDLIBPATH) $(CPPYYCHECK) ./$*.so --listfile=$*.list

# The following is not used by any of the actual build rules, but can be useful
# for debugging/troubleshooting, to see the gccxml output all the other
# commands are basing everything off of..
%.gccxml: %.h
	gccxml $^ -fxml=$@ $(APP_INCLUDES)
