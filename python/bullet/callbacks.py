import cppyy
import cffi
from bullet import bt

ffi = cffi.FFI()

class GlobalCallback (object):
    cffi_signature = None

    def __init__(self):
        self.func = None

    def __get__(self, obj, objtype):
        return self.func

    def __set__(self, obj, value):
        self.func = value
        if value:
            cb = ffi.callback(self.cffi_signature, self.cffi_callback)
            cb = int(ffi.cast("intptr_t", cb))
        else:
            cb = 0
        getattr(cppyy.gbl, self.cpp_setter)(cb)


class ContactAddedCallback (GlobalCallback):
    cpp_setter = "_set_gContactAddedCallback"
    cffi_signature = "bool (*f)(intptr_t i_cp, intptr_t i_colObj0, int partId0, int index0, intptr_t i_colObj1, int partId1, int index1)"

    def cffi_callback(self, i_cp, i_colObj0, partId0, index0, i_colObj1, partId1, index1):
        cp = cppyy.bind_object(i_cp, bt.ManifoldPoint)
        colObj0 = cppyy.bind_object(i_colObj0, bt.CollisionObject)
        colObj1 = cppyy.bind_object(i_colObj1, bt.CollisionObject)
        return bool(self.func(cp, colObj0, partId0, index0, colObj1, partId1, index1))


class ContactProcessedCallback (GlobalCallback):
    cpp_setter = "_set_gContactProcessedCallback"
    cffi_signature = "bool (*f)(intptr_t i_cp, intptr_t i_body0, intptr_t i_body1)"

    def cffi_callback(self, i_cp, i_body0, i_body1):
        cp = cppyy.bind_object(i_cp, bt.ManifoldPoint)
        body0 = cppyy.bind_object(i_body0, bt.CollisionObject)
        body1 = cppyy.bind_object(i_body1, bt.CollisionObject)
        return bool(self.func(cp, body0, body1))

class ContactDestroyedCallback (GlobalCallback):
    cpp_setter = "_set_gContactDestroyedCallback"
    cffi_signature = "bool (*f)(intptr_t i_userPersistentData)"

    def cffi_callback(self, i_userPersistentData):
        return bool(self.func(i_userPersistentData))

