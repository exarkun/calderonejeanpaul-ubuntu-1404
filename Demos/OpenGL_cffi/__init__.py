# This is a quick hack-up of CFFI bindings for a few of the most used OpenGL
# routines in the Bullet demo code.  It does not attempt to be anywhere near
# complete.  The idea is that after doing "from OpenGL.GL import *" you can
# then attempt to "from OpenGL_cffi import *", and it will 'upgrade' some of
# the more critical functions to use CFFI instead of the default ctypes
# implementation (for a very substantial speedup, at least under PyPy).

import traceback

__all__ = []

try:
    import ctypes
    import cffi
    ffi = cffi.FFI()
    ffi.cdef("""
        typedef unsigned int    GLenum;
        typedef void            GLvoid;
        typedef int             GLint;
        typedef unsigned int    GLuint;
        typedef int             GLsizei;
        typedef float           GLfloat;
        typedef double          GLdouble;
        void glMatrixMode( GLenum mode );
        void glScalef( GLfloat x, GLfloat y, GLfloat z );
        void glLoadIdentity( void );
        void glPushMatrix( void );
        void glPopMatrix( void );
        void glColor3f( GLfloat red, GLfloat green, GLfloat blue );
        void glMultMatrixd( const GLdouble *m );
        void glMultMatrixf( const GLfloat *m );
        void glDrawElements( GLenum mode, GLsizei count, GLenum type, const GLvoid *indices );
        void glDrawArrays( GLenum mode, GLint first, GLsizei count );
        void glVertexPointer( GLint size, GLenum type, GLsizei stride, const GLvoid *ptr );
        void glEnable( GLenum cap );
        void glDisable( GLenum cap );
        void glCallList( GLuint list );
    """)
    gl = ffi.verify("""
        #include <GL/gl.h>
    """, libraries=['GL'])

    glMatrixMode = gl.glMatrixMode
    glScalef = gl.glScalef
    glLoadIdentity = gl.glLoadIdentity
    glPushMatrix = gl.glPushMatrix
    glPopMatrix = gl.glPopMatrix
    glColor3f = gl.glColor3f
    glDrawArrays = gl.glDrawArrays
    glEnable = gl.glEnable
    glDisable = gl.glDisable
    glCallList = gl.glCallList

    def glMultMatrixd(m):
        m = ffi.cast('double*', ctypes.addressof(m))
        gl.glMultMatrixd(m)

    def glMultMatrixf(m):
        m = ffi.cast('float*', ctypes.addressof(m))
        gl.glMultMatrixf(m)

    def glDrawElements(mode, count, type, indices):
        indices = ffi.cast('void*', ctypes.addressof(indices))
        gl.glDrawElements(mode, count, type, indices)

    def glVertexPointer(size, type, stride, ptr):
        ptr = ffi.cast('void*', ctypes.addressof(ptr))
        gl.glVertexPointer(size, type, stride, ptr)

    __all__ = [
        'glMatrixMode',
        'glScalef',
        'glLoadIdentity',
        'glPushMatrix',
        'glPopMatrix',
        'glColor3f',
        'glMultMatrixd',
        'glMultMatrixf',
        'glDrawElements',
        'glDrawArrays',
        'glVertexPointer',
        'glEnable',
        'glDisable',
        'glCallList',
    ]
except Exception as e:
    traceback.print_exc()
    print "WARNING: Could not load OpenGL_cffi routines:  Rendering may be slow!"
