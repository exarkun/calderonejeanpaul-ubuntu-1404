import cppyy
import cffi
from bullet import bt

ffi = cffi.FFI()

class GlobalCallback (object):
    cffi_signature = None

    def __init__(self):
        self.cffi_cbobj = ffi.callback(self.cffi_signature, self.cffi_callback)
        self.func = None

    def __get__(self, obj, objtype):
        return self.func

    def __set__(self, obj, value):
        self.func = value
        if value:
            cb = int(ffi.cast("intptr_t", self.cffi_cbobj))
        else:
            cb = 0
        getattr(cppyy.gbl, self.cpp_setter)(cb)


class ContactAddedCallback (GlobalCallback):
    cpp_setter = "_py_set_gContactAddedCallback"
    cffi_signature = "bool (*f)(intptr_t i_cp, intptr_t i_colObj0, int partId0, int index0, intptr_t i_colObj1, int partId1, int index1)"

    def cffi_callback(self, i_cp, i_colObj0, partId0, index0, i_colObj1, partId1, index1):
        cp = cppyy.bind_object(i_cp, bt.ManifoldPoint)
        colObj0 = cppyy.bind_object(i_colObj0, bt.CollisionObject)
        colObj1 = cppyy.bind_object(i_colObj1, bt.CollisionObject)
        return bool(self.func(cp, colObj0, partId0, index0, colObj1, partId1, index1))


class ContactProcessedCallback (GlobalCallback):
    cpp_setter = "_py_set_gContactProcessedCallback"
    cffi_signature = "bool (*f)(intptr_t i_cp, intptr_t i_body0, intptr_t i_body1)"

    def cffi_callback(self, i_cp, i_body0, i_body1):
        cp = cppyy.bind_object(i_cp, bt.ManifoldPoint)
        body0 = cppyy.bind_object(i_body0, bt.CollisionObject)
        body1 = cppyy.bind_object(i_body1, bt.CollisionObject)
        return bool(self.func(cp, body0, body1))

class ContactDestroyedCallback (GlobalCallback):
    cpp_setter = "_py_set_gContactDestroyedCallback"
    cffi_signature = "bool (*f)(intptr_t i_userPersistentData)"

    def cffi_callback(self, i_userPersistentData):
        return bool(self.func(i_userPersistentData))


class btTriangleCallback (cppyy.gbl._py_btTriangleCallback):
    def __new__(cls):
        o = cppyy.gbl._py_btTriangleCallback.__new__(cls)
        o.__class__ = cls
        return o

    def __init__(self):
        self._cffi_cbobj = ffi.callback("void (*f)(intptr_t, intptr_t, intptr_t, int, int)", self._cffi_processTriangle)
        self._processTriangle_ptr = int(ffi.cast("intptr_t", self._cffi_cbobj))

    def _cffi_processTriangle(self, i_triangle0, i_triangle1, i_triangle2, partId, triangleIndex):
        triangle0 = cppyy.bind_object(i_triangle0, bt.Vector3)
        triangle1 = cppyy.bind_object(i_triangle1, bt.Vector3)
        triangle2 = cppyy.bind_object(i_triangle2, bt.Vector3)
        self.processTriangle((triangle0, triangle1, triangle2), partId, triangleIndex)

    def __repr__(self):
        return "<{} object at 0x{:x}>".format(self.__class__.__name__, cppyy.addressof(self))

class btInternalTriangleIndexCallback (cppyy.gbl._py_btInternalTriangleIndexCallback):
    def __new__(cls):
        o = cppyy.gbl._py_btInternalTriangleIndexCallback.__new__(cls)
        o.__class__ = cls
        return o

    def __init__(self):
        self._cffi_cbobj = ffi.callback("void (*f)(intptr_t, intptr_t, intptr_t, int, int)", self._cffi_internalProcessTriangleIndex)
        self._internalProcessTriangleIndex_ptr = int(ffi.cast("intptr_t", self._cffi_cbobj))

    def _cffi_internalProcessTriangleIndex(self, i_triangle0, i_triangle1, i_triangle2, partId, triangleIndex):
        triangle0 = cppyy.bind_object(i_triangle0, bt.Vector3)
        triangle1 = cppyy.bind_object(i_triangle1, bt.Vector3)
        triangle2 = cppyy.bind_object(i_triangle2, bt.Vector3)
        self.internalProcessTriangleIndex((triangle0, triangle1, triangle2), partId, triangleIndex)

    def __repr__(self):
        return "<{} object at 0x{:x}>".format(self.__class__.__name__, cppyy.addressof(self))

