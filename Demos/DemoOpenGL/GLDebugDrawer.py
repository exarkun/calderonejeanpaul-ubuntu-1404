from bullet import bt
import math
from OpenGL.GL import *

class GLDebugDrawer (bt.IDebugDraw):
    #int m_debugMode;

    def __init__(self):
        self.m_debugMode = 0

    def drawLine(self, vfrom, vto, fromColor, toColor=None):
        if toColor is None:
            toColor = fromColor
        glBegin(GL_LINES)
        glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ())
        glVertex3d(vfrom.getX(), vfrom.getY(), vfrom.getZ())
        glColor3f(toColor.getX(), toColor.getY(), toColor.getZ())
        glVertex3d(vto.getX(), vto.getY(), vto.getZ())
        glEnd()

    def drawSphere(self, p, radius, color):
        glColor4f(color.getX(), color.getY(), color.getZ(), 1.0)
        glPushMatrix()
        glTranslatef(p.getX(), p.getY(), p.getZ())

        lats = 5
        longs = 5

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

                glNormal3f(x * zr0, y * zr0, z0)
                glVertex3f(x * zr0, y * zr0, z0)
                glNormal3f(x * zr1, y * zr1, z1)
                glVertex3f(x * zr1, y * zr1, z1)
            glEnd()

        glPopMatrix()

    def drawTriangle(self, a, b, c, color, alpha):
        n = bt.Cross(b - a, c - a).normalized()
        glBegin(GL_TRIANGLES)
        glColor4f(color.getX(), color.getY(), color.getZ(), alpha)
        glNormal3d(n.getX(), n.getY(), n.getZ())
        glVertex3d(a.getX(), a.getY(), a.getZ())
        glVertex3d(b.getX(), b.getY(), b.getZ())
        glVertex3d(c.getX(), c.getY(), c.getZ())
        glEnd()

    def drawContactPoint(self, PointOnB, normalOnB, distance, lifeTime, color):
        vto = pointOnB + normalOnB * 1
        vfrom = pointOnB
        glColor4f(color.getX(), color.getY(), color.getZ(), 1.0)
        glBegin(GL_LINES)
        glVertex3d(vfrom.getX(), vfrom.getY(), vfrom.getZ())
        glVertex3d(vto.getX(), vto.getY(), vto.getZ())
        glEnd()

    def reportErrorWarning(self, warningString):
        print warningString

    def draw3dText(self, location, textString):
        glRasterPos3f(location.x(), location.y(), location.z())
        # BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString)

    def setDebugMode(self, debugMode):
        self.m_debugMode = debugMode

    def getDebugMode(self):
        return self.m_debugMode
