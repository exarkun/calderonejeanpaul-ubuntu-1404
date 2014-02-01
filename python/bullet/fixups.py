# This file contains fixups to be applied to the bullet classes in cppyy.gbl
# after they are loaded.  Simply importing this module (which is done from
# __init__) will apply the appropriate fixups.
#
# Note: This module must be imported after cppyy.load_reflection_info() is
# called in __init__.py

import cppyy

def getOpenGLMatrix(self, matrix=None):
    if matrix is None:
        return list(cppyy.gbl._py_OpenGLMatrix(self).m_matrix)
    else:
        m = cppyy.gbl._py_OpenGLMatrix(self)
        del matrix[:]
        matrix.extend(m.m_matrix)
        return matrix

cppyy.gbl.btTransform.getOpenGLMatrix = getOpenGLMatrix

