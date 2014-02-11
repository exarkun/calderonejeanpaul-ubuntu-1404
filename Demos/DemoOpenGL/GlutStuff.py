from OpenGL.GL import *
from OpenGL.GLUT import *
from bullet import bt

BT_USE_FREEGLUT = True

BT_KEY_K = ord('k')
BT_KEY_LEFT = GLUT_KEY_LEFT
BT_KEY_RIGHT = GLUT_KEY_RIGHT
BT_KEY_UP = GLUT_KEY_UP
BT_KEY_DOWN = GLUT_KEY_DOWN
BT_KEY_F1 = GLUT_KEY_F1
BT_KEY_F2 = GLUT_KEY_F2
BT_KEY_F3 = GLUT_KEY_F3
BT_KEY_F4 = GLUT_KEY_F4
BT_KEY_F5 = GLUT_KEY_F5
BT_KEY_PAGEUP = GLUT_KEY_PAGE_UP
BT_KEY_PAGEDOWN = GLUT_KEY_PAGE_DOWN
BT_KEY_END = GLUT_KEY_END
BT_KEY_HOME = GLUT_KEY_HOME
BT_ACTIVE_ALT = GLUT_ACTIVE_ALT
BT_ACTIVE_CTRL = GLUT_ACTIVE_CTRL
BT_ACTIVE_SHIFT = GLUT_ACTIVE_SHIFT

if bt.USE_DOUBLE_PRECISION:
    btglLoadMatrix = glLoadMatrixd
    btglMultMatrix = glMultMatrixd
    btglColor3 = glColor3d
    btglVertex3 = glVertex3d
else:
    btglLoadMatrix = glLoadMatrixf
    btglMultMatrix = glMultMatrixf
    btglColor3 = glColor3f
    btglVertex3 = glVertex3d # [Porting note: bug in original?]


def glutKeyboardCallback(key, x, y):
    global gDemoApplication
    gDemoApplication.keyboardCallback(key, x, y)


def glutKeyboardUpCallback(key, x, y):
    global gDemoApplication
    gDemoApplication.keyboardUpCallback(key, x, y)


def glutSpecialKeyboardCallback(key, x, y):
    global gDemoApplication
    gDemoApplication.specialKeyboard(key, x, y)


def glutSpecialKeyboardUpCallback(key, x, y):
    global gDemoApplication
    gDemoApplication.specialKeyboardUp(key, x, y)


def glutReshapeCallback(w, h):
    global gDemoApplication
    gDemoApplication.reshape(w, h)


def glutMoveAndDisplayCallback():
    global gDemoApplication
    gDemoApplication.moveAndDisplay()


def glutMouseFuncCallback(button, state, x, y):
    global gDemoApplication
    gDemoApplication.mouseFunc(button, state, x, y)


def glutMotionFuncCallback(x, y):
    global gDemoApplication
    gDemoApplication.mouseMotionFunc(x, y)


def glutDisplayCallback():
    global gDemoApplication
    gDemoApplication.displayCallback()


def glutmain(argv, width, height, title, demoApp):
    global gDemoApplication

    gDemoApplication = demoApp
    glutInit(argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL)
    glutInitWindowPosition(0, 0)
    glutInitWindowSize(width, height)
    glutCreateWindow(title)
    if BT_USE_FREEGLUT:
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS)

    gDemoApplication.myinit()

    glutKeyboardFunc(glutKeyboardCallback)
    glutKeyboardUpFunc(glutKeyboardUpCallback)
    glutSpecialFunc(glutSpecialKeyboardCallback)
    glutSpecialUpFunc(glutSpecialKeyboardUpCallback)

    glutReshapeFunc(glutReshapeCallback)
    glutIdleFunc(glutMoveAndDisplayCallback)
    glutMouseFunc(glutMouseFuncCallback)
    glutPassiveMotionFunc(glutMotionFuncCallback)
    glutMotionFunc(glutMotionFuncCallback)
    glutDisplayFunc(glutDisplayCallback)

    glutMoveAndDisplayCallback()

#TODO: do we need to do this?
#
##enable vsync to avoid tearing on Apple (todo: for Windows)
#
##if defined(__APPLE__) && !defined (VMDMESA)
#int swap_interval = 1;
#CGLContextObj cgl_context = CGLGetCurrentContext();
#CGLSetParameter(cgl_context, kCGLCPSwapInterval, &swap_interval);
##endif

    glutMainLoop()
