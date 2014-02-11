from bullet import bt

from DemoApplication import DemoApplication
from GlutStuff import *

mm = bt.memory_manager('DemoApplication')


class GlutDemoApplication (DemoApplication):
    def specialKeyboard(self, key, x, y):
        if key == GLUT_KEY_F1:
            pass
        elif key == GLUT_KEY_F2:
            pass
        elif key == GLUT_KEY_END:
            numObj = getDynamicsWorld().getNumCollisionObjects()
            if numObj:
                obj = getDynamicsWorld().getCollisionObjectArray()[numObj - 1]

                getDynamicsWorld().removeCollisionObject(obj)
                if obj.getMotionState():
                    mm.release(obj.getMotionState())
                mm.release(obj)
        elif key == GLUT_KEY_LEFT:
            self.stepLeft()
        elif key == GLUT_KEY_RIGHT:
            self.stepRight()
        elif key == GLUT_KEY_UP:
            self.stepFront()
        elif key == GLUT_KEY_DOWN:
            self.stepBack()
        elif key == GLUT_KEY_PAGE_UP:
            self.zoomIn()
        elif key == GLUT_KEY_PAGE_DOWN:
            self.zoomOut()
        elif key == GLUT_KEY_HOME:
            self.toggleIdle()
        else:
            #print "unused (special) key : %s" % (key,)
            pass

        glutPostRedisplay()

    def swapBuffers(self):
        glutSwapBuffers()

    def updateModifierKeys(self):
        self.m_modifierKeys = 0
        if glutGetModifiers() & GLUT_ACTIVE_ALT:
                self.m_modifierKeys |= BT_ACTIVE_ALT

        if glutGetModifiers() & GLUT_ACTIVE_CTRL:
                self.m_modifierKeys |= BT_ACTIVE_CTRL

        if glutGetModifiers() & GLUT_ACTIVE_SHIFT:
                self.m_modifierKeys |= BT_ACTIVE_SHIFT


PlatformDemoApplication = GlutDemoApplication
