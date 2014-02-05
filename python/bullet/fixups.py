# This file contains fixups to be applied to the bullet classes in cppyy.gbl
# after they are loaded.  Simply importing this module (which is done from
# __init__) will apply the appropriate fixups.
#
# Note: This module must be imported after cppyy.load_reflection_info() is
# called in __init__.py

import cppyy
import cffi

ffi = cffi.FFI()

# Extra operator functions which should be added to some classes:

cppyy.gbl.btVector3.__mul__ = getattr(cppyy.gbl, 'operator*')
cppyy.gbl.btVector3.__rmul__ = getattr(cppyy.gbl, 'operator*')
cppyy.gbl.btVector3.__add__ = getattr(cppyy.gbl, 'operator+')
cppyy.gbl.btVector3.__radd__ = getattr(cppyy.gbl, 'operator+')
cppyy.gbl.btVector3.__sub__ = getattr(cppyy.gbl, 'operator-')
cppyy.gbl.btVector3.__neg__ = getattr(cppyy.gbl, 'operator-')
cppyy.gbl.btVector3.__div__ = getattr(cppyy.gbl, 'operator/')
cppyy.gbl.btVector3.__truediv__ = getattr(cppyy.gbl, 'operator/')

cppyy.gbl.btMatrix3x3.__mul__ = getattr(cppyy.gbl, 'operator*')
cppyy.gbl.btMatrix3x3.__rmul__ = getattr(cppyy.gbl, 'operator*')
cppyy.gbl.btMatrix3x3.__add__ = getattr(cppyy.gbl, 'operator+')
cppyy.gbl.btMatrix3x3.__radd__ = getattr(cppyy.gbl, 'operator+')
cppyy.gbl.btMatrix3x3.__sub__ = getattr(cppyy.gbl, 'operator-')

cppyy.gbl.btQuaternion.__mul__ = getattr(cppyy.gbl, 'operator*')
cppyy.gbl.btQuaternion.__rmul__ = getattr(cppyy.gbl, 'operator*')

# The array-subscript usage of btVector3 appears to be unusual/obscure, but it
# is used in the demos, so we should support it (it also makes it easy to do
# things like convert a btVector3 to a list/tuple):

def __getitem__(self, index):
    if index > 2:
        raise IndexError(index)
    return self.m_floats[index]

def __setitem__(self, index, value):
    self.m_floats[index] = value

def __len__(self):
    return 3

cppyy.gbl.btVector3.__getitem__ = __getitem__
cppyy.gbl.btVector3.__setitem__ = __setitem__
cppyy.gbl.btVector3.__len__ = __len__

# Misc overrides for particular methods:

def getOpenGLMatrix(self, matrix=None):
    if matrix is None:
        return list(cppyy.gbl._py_OpenGLMatrix(self).m_matrix)
    else:
        matrix[:] = cppyy.gbl._py_OpenGLMatrix(self).m_matrix
        return matrix

cppyy.gbl.btTransform.getOpenGLMatrix = getOpenGLMatrix

def getBuffer(self):
    cdata = ffi.cast("char *", self.getBufferPointer().buffer)
    return ffi.buffer(cdata, self.getCurrentBufferSize())

cppyy.gbl.btSerializer.getBuffer = getBuffer
