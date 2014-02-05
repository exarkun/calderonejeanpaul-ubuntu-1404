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

cppyy.gbl.btMatrix3x3.__mul__ = getattr(cppyy.gbl, 'operator*')
cppyy.gbl.btMatrix3x3.__rmul__ = getattr(cppyy.gbl, 'operator*')
cppyy.gbl.btMatrix3x3.__add__ = getattr(cppyy.gbl, 'operator+')
cppyy.gbl.btMatrix3x3.__radd__ = getattr(cppyy.gbl, 'operator+')
cppyy.gbl.btMatrix3x3.__sub__ = getattr(cppyy.gbl, 'operator-')

cppyy.gbl.btQuaternion.__mul__ = getattr(cppyy.gbl, 'operator*')
cppyy.gbl.btQuaternion.__rmul__ = getattr(cppyy.gbl, 'operator*')

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
