import cppyy

#FIXME: Make all this threadsafe

class MemoryManager (object):
    memory_managers = {}

    def __new__(cls, name=None, persistent=None):
        if persistent is None:
            persistent = (name is None)
        if persistent:
            try:
                return cls.memory_managers[name]
            except KeyError:
                o = object.__new__(cls, name, persistent)
                cls.memory_managers[name] = o
                return o
        else:
            return object.__new__(cls, name, persistent)

    def __init__(self, name=None, persistent=None):
        self.name = name
        self.persistent = persistent
        self._objects = {}

    def __call__(self, obj):
        return self.keep(obj)

    def keep(self, obj):
        #TODO: handle non-cppyy objects?
        objid = cppyy.addressof(obj)
        self._objects.setdefault(objid, [0, obj])[0] += 1
        return obj

    def release(self, obj):
        objid = cppyy.addressof(obj)
        info = self._objects[objid]
        info[0] -= 1
        if info[0] <= 0:
            del self._objects[objid]
            return True
        return False

    def release_all(self, force=False):
        if force:
            self._objects = {}
        else:
            for count, obj in list(self._objects.values()):
                self.release(obj)
        return (not self._objects)

    def __repr__(self):
        if self.name is not None:
            name = "({!r})".format(self.name)
        else:
            name = " at 0x{:x}"
        if self.persistent:
            persistent = " [persistent]"
        else:
            persistent = ""
        return "<{}{}{}: {} kept objects>".format(self.__class__.__name__, name, persistent, len(self._objects))

