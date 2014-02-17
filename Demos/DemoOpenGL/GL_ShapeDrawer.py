import gc, time
from bullet import bt

from GlutStuff import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.images import *
from OpenGL_cffi import *

USE_ARRAYS = True
USE_DISPLAY_LISTS = True

mm = bt.memory_manager('DemoApplication')

box_vertices = (GLfloat * (24 * 3))(
    1, 1, 1,  -1, 1, 1,  -1,-1, 1,   1,-1, 1,  # v0,v1,v2,v3 (front)
    1, 1, 1,   1,-1, 1,   1,-1,-1,   1, 1,-1,  # v0,v3,v4,v5 (right)
    1, 1, 1,   1, 1,-1,  -1, 1,-1,  -1, 1, 1,  # v0,v5,v6,v1 (top)
   -1, 1, 1,  -1, 1,-1,  -1,-1,-1,  -1,-1, 1,  # v1,v6,v7,v2 (left)
   -1,-1,-1,   1,-1,-1,   1,-1, 1,  -1,-1, 1,  # v7,v4,v3,v2 (bottom)
    1,-1,-1,  -1,-1,-1,  -1, 1,-1,   1, 1,-1,  # v4,v7,v6,v5 (back)
)

box_normals = (GLfloat * (24 * 3))(
    0, 0, 1,   0, 0, 1,   0, 0, 1,   0, 0, 1,  # v0,v1,v2,v3 (front)
    1, 0, 0,   1, 0, 0,   1, 0, 0,   1, 0, 0,  # v0,v3,v4,v5 (right)
    0, 1, 0,   0, 1, 0,   0, 1, 0,   0, 1, 0,  # v0,v5,v6,v1 (top)
   -1, 0, 0,  -1, 0, 0,  -1, 0, 0,  -1, 0, 0,  # v1,v6,v7,v2 (left)
    0,-1, 0,   0,-1, 0,   0,-1, 0,   0,-1, 0,  # v7,v4,v3,v2 (bottom)
    0, 0,-1,   0, 0,-1,   0, 0,-1,   0, 0,-1,  # v4,v7,v6,v5 (back)
)

box_indices = (GLubyte * 36)(
    0, 1, 2,   2, 3, 0,  # front
    4, 5, 6,   6, 7, 4,  # right
    8, 9,10,  10,11, 8,  # top
   12,13,14,  14,15,12,  # left
   16,17,18,  18,19,16,  # bottom
   20,21,22,  22,23,20,  # back
)


# OpenGL shape drawing
class GL_ShapeDrawer:
    class ShapeCache:
        class Edge:
            def __init__(self, n=[None, None], v=[0, 0]):
                self.n = list(n)
                self.v = list(v)

        def __init__(self, s):
            self.m_shapehull = bt.ShapeHull(s)
            self.m_edges = []

    def __init__(self):
        self.m_shapecaches = []
        self.m_texturehandle = None
        self.m_textureenabled = False
        self.m_textureinitialized = False
        self.m_svacache_old = {}
        self.m_svacache_new = {}

    def __del__(self):
        if self.m_textureinitialized:
            glDeleteTextures(1, self.m_texturehandle)

    def drawOpenGLInit(self):
        if self.m_textureenabled:
            if not self.m_textureinitialized:
                image = createTargetArray(GL_RGB, [256, 256], GL_UNSIGNED_BYTE)
                for y in xrange(256):
                    t = y >> 4
                    for x in xrange(256):
                        s = x >> 4
                        b = 180

                        c = b + ((s + (t & 1)) & 1) * (255 - b)
                        image[0][x][y] = image[1][x][y] = image[2][x][y] = c

                self.m_texturehandle = glGenTextures(1)
                glBindTexture(GL_TEXTURE_2D, self.m_texturehandle)
                glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE)
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
                gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 256, 256, GL_RGB, GL_UNSIGNED_BYTE, image)
                self.m_textureinitialized = True

            planex = (ctypes.c_float * 4)(1, 0, 0, 0)
            planez = (ctypes.c_float * 4)(0, 0, 1, 0)
            glTexGenfv(GL_S, GL_OBJECT_PLANE, planex)
            glTexGenfv(GL_T, GL_OBJECT_PLANE, planez)
            glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR)
            glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR)
            glEnable(GL_TEXTURE_GEN_S)
            glEnable(GL_TEXTURE_GEN_T)
            glEnable(GL_TEXTURE_GEN_R)

            glEnable(GL_COLOR_MATERIAL)
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, self.m_texturehandle)
        else:
            glDisable(GL_TEXTURE_2D)

        glEnableClientState(GL_NORMAL_ARRAY)
        glEnableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
        glDisableClientState(GL_TEXTURE_COORD_ARRAY)

    def drawOpenGLStart(self):
        self.m_lastShapeType = None

    def drawBox(self, shape):
        halfExtent = shape.getHalfExtentsWithMargin()

        glMatrixMode(GL_TEXTURE)
        glLoadIdentity()
        glScalef(0.025 * halfExtent[0], 0.025 * halfExtent[1], 0.025 * halfExtent[2])
        glMatrixMode(GL_MODELVIEW)
        glScalef(halfExtent[0], halfExtent[1], halfExtent[2])

        if self.m_lastShapeType != bt.BOX_SHAPE_PROXYTYPE:
            glNormalPointer(GL_FLOAT, 0, box_normals)
            glVertexPointer(3, GL_FLOAT, 0, box_vertices)

        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, box_indices)

    def drawOpenGL(self, m, shape, color, debugMode, worldBoundsMin, worldBoundsMax):
        shapetype = shape.getShapeType()

        if shapetype == bt.CUSTOM_CONVEX_SHAPE_TYPE:
            org = bt.Vector3(m[12], m[13], m[14])
            dx = bt.Vector3(m[0], m[1], m[2])
            dy = bt.Vector3(m[4], m[5], m[6])
            halfExtent = shape.getHalfExtentsWithMargin()
            dx *= halfExtent[0]
            dy *= halfExtent[1]
            glColor3f(1, 1, 1)
            glDisable(GL_LIGHTING)
            glLineWidth(2)

            glBegin(GL_LINE_LOOP)
            glDrawVector(org - dx - dy)
            glDrawVector(org - dx + dy)
            glDrawVector(org + dx + dy)
            glDrawVector(org + dx - dy)
            glEnd()
            return
        elif (shapetype == bt.BOX_SHAPE_PROXYTYPE) and (debugMode & bt.IDebugDraw.DBG_FastWireframe):
            org = bt.Vector3(m[12], m[13], m[14])
            dx = bt.Vector3(m[0], m[1], m[2])
            dy = bt.Vector3(m[4], m[5], m[6])
            dz = bt.Vector3(m[8], m[9], m[10])
            halfExtent = shape.getHalfExtentsWithMargin()
            dx *= halfExtent[0]
            dy *= halfExtent[1]
            dz *= halfExtent[2]
            glBegin(GL_LINE_LOOP)
            glDrawVector(org - dx - dy - dz)
            glDrawVector(org + dx - dy - dz)
            glDrawVector(org + dx + dy - dz)
            glDrawVector(org - dx + dy - dz)
            glDrawVector(org - dx + dy + dz)
            glDrawVector(org + dx + dy + dz)
            glDrawVector(org + dx - dy + dz)
            glDrawVector(org - dx - dy + dz)
            glEnd()
            glBegin(GL_LINES)
            glDrawVector(org + dx - dy - dz)
            glDrawVector(org + dx - dy + dz)
            glDrawVector(org + dx + dy - dz)
            glDrawVector(org + dx + dy + dz)
            glDrawVector(org - dx - dy - dz)
            glDrawVector(org - dx + dy - dz)
            glDrawVector(org - dx - dy + dz)
            glDrawVector(org - dx + dy + dz)
            glEnd()
            return

        glPushMatrix()
        btglMultMatrix(m)

        if shapetype == bt.UNIFORM_SCALING_SHAPE_PROXYTYPE:
            convexShape = shape.getChildShape()
            scalingFactor = shape.getUniformScalingFactor()
            tmpScaling = [
                scalingFactor, 0, 0, 0,
                0, scalingFactor, 0, 0,
                0, 0, scalingFactor, 0,
                0, 0, 0, 1,
            ]

            self.drawOpenGL(tmpScaling, convexShape, color, debugMode, worldBoundsMin, worldBoundsMax)
            glPopMatrix()
            return

        if shapetype == bt.COMPOUND_SHAPE_PROXYTYPE:
            for i in xrange(shape.getNumChildShapes()):
                childTrans = shape.getChildTransform(i)
                colShape = shape.getChildShape(i)
                childMat = childTrans.getOpenGLMatrix()
                drawOpenGL(childMat, colShape, color, debugMode, worldBoundsMin, worldBoundsMax)
        else:
            glColor3f(color.x(), color.y(), color.z())

            useWireframeFallback = True

            if not (debugMode & bt.IDebugDraw.DBG_DrawWireframe):
                ## you can comment out any of the specific cases, and use the default

                ## the benefit of 'default' is that it approximates the actual collision shape including collision margin
                if shapetype == bt.SPHERE_SHAPE_PROXYTYPE:
                    glMatrixMode(GL_TEXTURE)
                    glLoadIdentity()
                    glScalef(0.025, 0.025, 0.025)
                    glMatrixMode(GL_MODELVIEW)

                    radius = shape.getMargin()  # radius doesn't include the margin, so draw with margin
                    drawSphere(radius, 10, 10)
                    useWireframeFallback = False
                elif shapetype == bt.BOX_SHAPE_PROXYTYPE:
                    self.drawBox(shape)
                    useWireframeFallback = False

                elif shapetype == bt.STATIC_PLANE_PROXYTYPE:
                    glMatrixMode(GL_TEXTURE)
                    glLoadIdentity()
                    glScalef(0.025, 0.025, 0.025)
                    glMatrixMode(GL_MODELVIEW)

                    planeConst = shape.getPlaneConstant()
                    planeNormal = shape.getPlaneNormal()
                    planeOrigin = planeNormal * planeConst
                    vec0 = bt.Vector3()
                    vec1 = bt.Vector3()
                    bt.PlaneSpace1(planeNormal, vec0, vec1)
                    vecLen = 100
                    pt0 = planeOrigin + vec0 * vecLen
                    pt1 = planeOrigin - vec0 * vecLen
                    pt2 = planeOrigin + vec1 * vecLen
                    pt3 = planeOrigin - vec1 * vecLen
                    glBegin(GL_LINES)
                    glVertex3f(pt0.getX(), pt0.getY(), pt0.getZ())
                    glVertex3f(pt1.getX(), pt1.getY(), pt1.getZ())
                    glVertex3f(pt2.getX(), pt2.getY(), pt2.getZ())
                    glVertex3f(pt3.getX(), pt3.getY(), pt3.getZ())
                    glEnd()

                elif shapetype == bt.MULTI_SPHERE_SHAPE_PROXYTYPE:
                    glMatrixMode(GL_TEXTURE)
                    glLoadIdentity()
                    glScalef(0.025, 0.025, 0.025)
                    glMatrixMode(GL_MODELVIEW)

                    childTransform = bt.Transform()
                    childTransform.setIdentity()

                    for i in xrange(shape.getSphereCount()):
                        sc = bt.SphereShape(shape.getSphereRadius(i))
                        childTransform.setOrigin(shape.getSpherePosition(i))
                        childMat = childTransform.getOpenGLMatrix()
                        drawOpenGL(childMat, sc, color, debugMode, worldBoundsMin, worldBoundsMax)

                else:
                    glMatrixMode(GL_TEXTURE)
                    glLoadIdentity()
                    glScalef(0.025, 0.025, 0.025)
                    glMatrixMode(GL_MODELVIEW)

                    if shape.isConvex():
                        if shape.isPolyhedral():
                            poly = shape.getConvexPolyhedron()
                            glBegin(GL_TRIANGLES)
                            for i in xrange(poly.m_faces.size()):
                                centroid = bt.Vector3(0, 0, 0)
                                numVerts = poly.m_faces[i].m_indices.size()
                                if numVerts > 2:
                                    v1 = poly.m_vertices[poly.m_faces[i].m_indices[0]]
                                    for v in xrange(poly.m_faces[i].m_indices.size() - 2):
                                        v2 = poly.m_vertices[poly.m_faces[i].m_indices[v + 1]]
                                        v3 = poly.m_vertices[poly.m_faces[i].m_indices[v + 2]]
                                        normal = (v3 - v1).cross(v2 - v1)
                                        normal.normalize()
                                        glNormal3f(normal.getX(), normal.getY(), normal.getZ())
                                        glVertex3f(v1.x(), v1.y(), v1.z())
                                        glVertex3f(v2.x(), v2.y(), v2.z())
                                        glVertex3f(v3.x(), v3.y(), v3.z())
                            glEnd()
                        else:
                            sc = self.cache(shape)
                            hull = sc.m_shapehull

                            if hull.numTriangles() > 0:
                                index = 0
                                idx = hull.getIndexPointer()
                                vtx = hull.getVertexPointer()

                                glBegin(GL_TRIANGLES)

                                for i in xrange(hull.numTriangles()):
                                    i1 = index + 0
                                    i2 = index + 1
                                    i3 = index + 2

                                    index1 = idx[i1]
                                    index2 = idx[i2]
                                    index3 = idx[i3]

                                    v1 = vtx[index1]
                                    v2 = vtx[index2]
                                    v3 = vtx[index3]
                                    normal = (v3 - v1).cross(v2 - v1)
                                    normal.normalize()
                                    glNormal3f(normal.getX(), normal.getY(), normal.getZ())
                                    glVertex3f(v1.x(), v1.y(), v1.z())
                                    glVertex3f(v2.x(), v2.y(), v2.z())
                                    glVertex3f(v3.x(), v3.y(), v3.z())

                                glEnd()

            ## for polyhedral shapes
            if debugMode == bt.IDebugDraw.DBG_DrawFeaturesText and shape.isPolyhedral():
                # [Porting note: This entire section is do-nothing code in the original too.  Left in for completeness.]
                glColor3f(1, 1, 1)
                for i in xrange(shape.getNumVertices()):
                    vtx = bt.Vector3()
                    shape.getVertex(i, vtx)
                    buf = " %d" % (i,)
                    #btDrawString(BMF_GetFont(BMF_kHelvetica10), buf)

                for i in xrange(shape.getNumPlanes()):
                    normal = bt.Vector3()
                    vtx = bt.Vector3()
                    shape.getPlane(normal, vtx, i)
                    #d = vtx.dot(normal)

                    #buf = " plane %d" % (i,)
                    #btDrawString(BMF_GetFont(BMF_kHelvetica10), buf)

            if USE_DISPLAY_LISTS:
                if shapetype in (bt.TRIANGLE_MESH_SHAPE_PROXYTYPE, bt.GIMPACT_SHAPE_PROXYTYPE):
                    dlist = OGL_get_displaylist_for_shape(shape)
                    if dlist:
                        glCallList(dlist)
            else:
                if shape.isConcave() and not shape.isInfinite():
                    drawCallback = GlDrawcallback()
                    drawCallback.m_wireframe = (debugMode & bt.IDebugDraw.DBG_DrawWireframe) != 0

                    shape.processAllTriangles(drawCallback, worldBoundsMin, worldBoundsMax)

        glPopMatrix()
        self.m_lastShapeType = shapetype

    def drawShadowStart(self):
        if USE_ARRAYS:
            glDisableClientState(GL_NORMAL_ARRAY)
            glEnableClientState(GL_VERTEX_ARRAY)
            glDisableClientState(GL_COLOR_ARRAY)
            glDisableClientState(GL_TEXTURE_COORD_ARRAY)
            #print (len(self.m_svacache_old) - len(self.m_svacache_new))
            self.m_svacache_old = self.m_svacache_new
            self.m_svacache_new = {}
            self.m_currentSVA = None

    def drawShadow(self, m, extrusion, shape, worldBoundsMin, worldBoundsMax):
        glPushMatrix()
        btglMultMatrix(m)
        if shape.getShapeType() == bt.UNIFORM_SCALING_SHAPE_PROXYTYPE:
            convexShape = shape.getChildShape()
            scalingFactor = shape.getUniformScalingFactor()
            tmpScaling = [
                scalingFactor, 0, 0, 0,
                0, scalingFactor, 0, 0,
                0, 0, scalingFactor, 0,
                0, 0, 0, 1,
            ]
            self.drawShadow(tmpScaling, extrusion, convexShape, worldBoundsMin, worldBoundsMax)
            glPopMatrix()
            return
        elif shape.getShapeType() == bt.COMPOUND_SHAPE_PROXYTYPE:
            for i in xrange(shape.getNumChildShapes()):
                childTrans = shape.getChildTransform(i)
                colShape = shape.getChildShape(i)
                childMat = childTrans.getOpenGLMatrix()
                self.drawShadow(childMat, extrusion * childTrans.getBasis(), colShape, worldBoundsMin, worldBoundsMax)
        else:
            if shape.isConvex():
                if USE_ARRAYS:
                    sva_id = (shape, tuple(int(x) for x in extrusion))
                    sva = self.m_svacache_old.get(sva_id)
                    if not sva:
                        sc = self.cache(shape)
                        hull = sc.m_shapehull
                        verts = hull.getVertexPointer()
                        sv = (GLfloat * (3 * 4 * len(sc.m_edges)))()
                        i = 0
                        for edge in sc.m_edges:
                            d = bt.Dot(edge.n[0], extrusion)
                            if (d * bt.Dot(edge.n[1], extrusion)) < 0:
                                if d >= 0:
                                    a = verts[edge.v[0]]
                                    b = verts[edge.v[1]]
                                else:
                                    a = verts[edge.v[1]]
                                    b = verts[edge.v[0]]
                                sv[i + 0] = a.getX()
                                sv[i + 1] = a.getY()
                                sv[i + 2] = a.getZ()
                                sv[i + 3] = b.getX()
                                sv[i + 4] = b.getY()
                                sv[i + 5] = b.getZ()
                                aext = a + extrusion
                                bext = b + extrusion
                                sv[i + 6] = bext.getX()
                                sv[i + 7] = bext.getY()
                                sv[i + 8] = bext.getZ()
                                sv[i + 9] = aext.getX()
                                sv[i + 10] = aext.getY()
                                sv[i + 11] = aext.getZ()
                                i += 12
                        sva = (sv, i / 3)
                        self.m_svacache_old[sva_id] = sva
                    self.m_svacache_new[sva_id] = sva
                    if self.m_currentSVA != sva:
                        glVertexPointer(3, GL_FLOAT, 0, sva[0])
                        self.m_currentSVA = sva
                    glDrawArrays(GL_QUADS, 0, sva[1])
                else:
                    sc = self.cache(shape)
                    hull = sc.m_shapehull
                    verts = hull.getVertexPointer()
                    glBegin(GL_QUADS)
                    for edge in sc.m_edges:
                        d = bt.Dot(edge.n[0], extrusion)
                        if (d * bt.Dot(edge.n[1], extrusion)) < 0:
                            if d >= 0:
                                a = verts[edge.v[0]]
                                b = verts[edge.v[1]]
                            else:
                                a = verts[edge.v[1]]
                                b = verts[edge.v[0]]
                            glVertex3f(a.getX(), a.getY(), a.getZ())
                            glVertex3f(b.getX(), b.getY(), b.getZ())
                            aext = a + extrusion
                            bext = b + extrusion
                            glVertex3f(bext.getX(), bext.getY(), bext.getZ())
                            glVertex3f(aext.getX(), aext.getY(), aext.getZ())
                    glEnd()

        if shape.isConcave():
            drawCallback = GlDrawcallback()
            drawCallback.m_wireframe = False

            shape.processAllTriangles(drawCallback, worldBoundsMin, worldBoundsMax)

        glPopMatrix()

    def enableTexture(self, enable):
        p = self.m_textureenabled
        self.m_textureenabled = enable
        return p

    def hasTextureEnabled(self):
        return self.m_textureenabled

    def drawCylinder(self, radius, halfHeight, upAxis):
        glPushMatrix()
        if upAxis == 0:
            glRotatef(-90.0, 0.0, 1.0, 0.0)
            glTranslatef(0.0, 0.0, -halfHeight)
        elif upAxis == 1:
            glRotatef(-90.0, 1.0, 0.0, 0.0)
            glTranslatef(0.0, 0.0, -halfHeight)
        elif upAxis == 2:
            glTranslatef(0.0, 0.0, -halfHeight)
        else:
            raise ValueError("Bad value for upAxis")

        quadObj = gluNewQuadric()

        # The gluCylinder subroutine draws a cylinder that is oriented along the z axis.
        # The base of the cylinder is placed at z = 0; the top of the cylinder is placed at z=height.
        # Like a sphere, the cylinder is subdivided around the z axis into slices and along the z axis into stacks.

        gluQuadricDrawStyle(quadObj, GLU_FILL)
        gluQuadricNormals(quadObj, GLU_SMOOTH)

        gluDisk(quadObj, 0, radius, 15, 10)

        gluCylinder(quadObj, radius, radius, 2 * halfHeight, 15, 10)
        glTranslatef(0.0, 0.0, 2 * halfHeight)
        glRotatef(-180.0, 0.0, 1.0, 0.0)
        gluDisk(quadObj, 0., radius, 15, 10)

        glPopMatrix()
        gluDeleteQuadric(quadObj)

    def drawSphere(self, radius, lats, longs):
        for i in xrange(lats):
            lat0 = math.pi * (-0.5 + (i - 1) / lats)
            z0 = radius * math.sin(lat0)
            zr0 = radius * math.cos(lat0)

            lat1 = math.pi * (-0.5 + i / lats)
            z1 = radius * math.sin(lat1)
            zr1 = radius * math.cos(lat1)

            glBegin(GL_QUAD_STRIP)
            for j in xrange(longs):
                lng = 2 * math.pi * (j - 1) / longs
                x = math.cos(lng)
                y = math.sin(lng)
                glNormal3f(x * zr1, y * zr1, z1)
                glVertex3f(x * zr1, y * zr1, z1)
                glNormal3f(x * zr0, y * zr0, z0)
                glVertex3f(x * zr0, y * zr0, z0)
            glEnd()

    def drawCoordSystem(self):
        glBegin(GL_LINES)
        glColor3f(1, 0, 0)
        glVertex3d(0, 0, 0)
        glVertex3d(1, 0, 0)
        glColor3f(0, 1, 0)
        glVertex3d(0, 0, 0)
        glVertex3d(0, 1, 0)
        glColor3f(0, 0, 1)
        glVertex3d(0, 0, 0)
        glVertex3d(0, 0, 1)
        glEnd()

    def cache(self, shape):
        sc = shape.getUserPointer()
        if not sc:
            sc = self.ShapeCache(shape)
            sc.m_shapehull.buildHull(shape.getMargin())
            self.m_shapecaches.append(sc)
            shape.setUserPointer(sc)
            # Build edges
            ni = sc.m_shapehull.numIndices()
            nv = sc.m_shapehull.numVertices()
            pi = sc.m_shapehull.getIndexPointer()
            pv = sc.m_shapehull.getVertexPointer()
            edges = {}
            for i in xrange(0, ni, 3):
                nrm = bt.Cross(pv[pi[i + 1]] - pv[pi[i]], pv[pi[i + 2]] - pv[pi[i]]).normalized()
                j = 2
                k = 0
                while k < 3:
                    a = pi[i + j]
                    b = pi[i + k]
                    e = edges.get((min(a, b), max(a, b)))
                    if not e:
                        e = GL_ShapeDrawer.ShapeCache.Edge()
                        edges[(min(a,b), max(a,b))] = e
                        sc.m_edges.append(e)
                        e.n[0] = nrm
                        e.n[1] = -nrm
                        e.v[0] = a
                        e.v[1] = b
                    else:
                        e.n[1] = nrm
                    j = k
                    k += 1
        return sc


g_display_lists = {}


class GlDisplaylistDrawcallback (bt.TriangleCallback):
    def processTriangle(self, triangle, partId, triangleIndex):
        diff1 = triangle[1] - triangle[0]
        diff2 = triangle[2] - triangle[0]
        normal = diff1.cross(diff2)

        normal.normalize()

        glBegin(GL_TRIANGLES)
        glColor3f(1, 1, 1)
        glNormal3d(normal.getX(), normal.getY(), normal.getZ())
        glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ())

        #glColor3f(0, 1, 0)
        glNormal3d(normal.getX(), normal.getY(), normal.getZ())
        glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ())

        #glColor3f(0, 1, 0)
        glNormal3d(normal.getX(), normal.getY(), normal.getZ())
        glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ())
        glEnd()


def OGL_get_displaylist_for_shape(shape):
    return g_display_lists.get(shape, 0)


def OGL_displaylist_clean():
    for dlist in g_display_lists.values():
        glDeleteLists(dlist, 1)


def OGL_displaylist_register_shape(shape):
    aabbMax = bt.Vector3(bt.LARGE_FLOAT, bt.LARGE_FLOAT, bt.LARGE_FLOAT)
    aabbMin = bt.Vector3(-bt.LARGE_FLOAT, -bt.LARGE_FLOAT, -bt.LARGE_FLOAT)
    drawCallback = GlDisplaylistDrawcallback()

    dlist = glGenLists(1)

    g_display_lists[shape] = dlist

    glNewList(dlist, GL_COMPILE)

    glCullFace(GL_BACK)

    if shape.isConcave():
        # todo pass camera, for some culling
        shape.processAllTriangles(drawCallback, aabbMin, aabbMax)

    glEndList()


class GlDrawcallback (bt.TriangleCallback):
    def __init__(self):
        self.m_wireframe = False

    def processTriangle(triangle, partId, triangleIndex):
        if self.m_wireframe:
            glBegin(GL_LINES)
            glColor3f(1, 0, 0)
            glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ())
            glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ())
            glColor3f(0, 1, 0)
            glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ())
            glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ())
            glColor3f(0, 0, 1)
            glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ())
            glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ())
            glEnd()
        else:
            glBegin(GL_TRIANGLES)

            glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ())
            glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ())
            glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ())

            glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ())
            glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ())
            glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ())
            glEnd()


class TriangleGlDrawcallback (bt.InternalTriangleIndexCallback):
    def internalProcessTriangleIndex(triangle, partId, triangleIndex):
        glBegin(GL_TRIANGLES)
        glColor3f(1, 0, 0)
        glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ())
        glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ())
        glColor3f(0, 1, 0)
        glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ())
        glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ())
        glColor3f(0, 0, 1)
        glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ())
        glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ())
        glEnd()


def renderSquareA(x, y, z):
    glBegin(GL_LINE_LOOP)
    glVertex3f(x, y, z)
    glVertex3f(x + 10, y, z)
    glVertex3f(x + 10, y + 10, z)
    glVertex3f(x, y + 10, z)
    glEnd()


def glDrawVector(v):
    glVertex3d(v[0], v[1], v[2])
