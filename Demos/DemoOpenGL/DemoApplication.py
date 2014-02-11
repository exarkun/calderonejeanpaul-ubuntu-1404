from __future__ import division

import math
from bullet import bt

from GL_ShapeDrawer import GL_ShapeDrawer
from GLDebugFont import *
from GlutStuff import *
from OpenGL.GLU import *

USE_MOTIONSTATE = True
SHOW_NUM_DEEP_PENETRATIONS = True

numObjects = 0
maxNumObjects = 16384
STEPSIZE = 5

gOldPickingPos = None
gHitPos = bt.Vector3(-1, -1, -1)
gOldPickingDist = 0
pickedBody = None  # for deactivation state
mousePickClamping = 30

mm = bt.memory_manager('DemoApplication')


class DemoApplication:
    def __init__(self):
        self.m_dynamicsWorld = None
        self.m_pickConstraint = None
        self.m_shootBoxShape = None
        self.m_cameraDistance = 15.0
        self.m_debugMode = 0
        self.m_ele = 20.0
        self.m_azi = 0.0
        self.m_cameraPosition = bt.Vector3(0, 0, 0)
        self.m_cameraTargetPosition = bt.Vector3(0, 0, 0)
        self.m_mouseOldX = 0
        self.m_mouseOldY = 0
        self.m_mouseButtons = 0
        self.m_modifierKeys = 0
        self.m_scaleBottom = 0.5
        self.m_scaleFactor = 2.0
        self.m_cameraUp = bt.Vector3(0, 1, 0)
        self.m_forwardAxis = 2
        self.m_zoomStepSize = 0.4
        self.m_glutScreenWidth = 0
        self.m_glutScreenHeight = 0
        self.m_frustumZNear = 1.0
        self.m_frustumZFar = 10000.0
        self.m_ortho = False
        self.m_ShootBoxInitialSpeed = 40.0
        self.m_stepping = True
        self.m_singleStep = False
        self.m_idle = False
        self.m_lastKey = None

        self.m_enableshadows = False
        self.m_sundirection = bt.Vector3(1, -2, 1) * 1000
        self.m_defaultContactProcessingThreshold = bt.LARGE_FLOAT

        if not bt.NO_PROFILE:
            self.m_profileIterator = bt.CProfileManager.Get_Iterator()

        self.m_shapeDrawer = GL_ShapeDrawer()
        self.m_shapeDrawer.enableTexture(True)

        if bt.USE_BT_CLOCK:
            self.m_clock = bt.Clock()

    def __del__(self):
        if not bt.NO_PROFILE:
            bt.CProfileManager.Release_Iterator(self.m_profileIterator)

    def displayProfileString(self, xOffset, yStart, message):
        glRasterPos3f(xOffset, yStart, 0)
        GLDebugDrawString(xOffset, yStart, message)

    def removePickingConstraint(self):
        global pickedBody

        if self.m_pickConstraint and self.m_dynamicsWorld:
            self.m_dynamicsWorld.removeConstraint(self.m_pickConstraint)
            self.m_pickConstraint = None
            pickedBody.forceActivationState(bt.ACTIVE_TAG)
            pickedBody.setDeactivationTime(0)
            pickedBody = 0

    def showProfileInfo(self, xOffset, yStart, yIncr):
        if not bt.NO_PROFILE:
            time_since_reset = 0
            if not self.m_idle:
                time_since_reset = bt.CProfileManager.Get_Time_Since_Reset()

            # recompute profiling data, and store profile strings

            totalTime = 0

            frames_since_reset = bt.CProfileManager.Get_Frame_Count_Since_Reset()

            self.m_profileIterator.First()

            if self.m_profileIterator.Is_Root():
                parent_time = time_since_reset
            else:
                parent_time = self.m_profileIterator.Get_Current_Parent_Total_Time()

            blockTime = "--- Profiling: %s (total running time: %.3f ms) ---" % (self.m_profileIterator.Get_Current_Parent_Name(), parent_time)
            self.displayProfileString(xOffset, yStart, blockTime)
            yStart += yIncr
            blockTime = "press (1,2...) to display child timings, or 0 for parent"
            self.displayProfileString(xOffset, yStart, blockTime)
            yStart += yIncr

            accumulated_time = 0

            i = 0
            while not self.m_profileIterator.Is_Done():
                current_total_time = self.m_profileIterator.Get_Current_Total_Time()
                accumulated_time += current_total_time
                if parent_time > bt.SIMD_EPSILON:
                    fraction = (current_total_time / parent_time) * 100
                else:
                    fraction = 0

                i += 1
                blockTime = "%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)" % (
                    i, self.m_profileIterator.Get_Current_Name(), fraction,
                    (current_total_time / frames_since_reset), self.m_profileIterator.Get_Current_Total_Calls())
                self.displayProfileString(xOffset, yStart, blockTime)
                yStart += yIncr
                totalTime += current_total_time
                self.m_profileIterator.Next()

            if parent_time > bt.SIMD_EPSILON:
                unaccounted_fraction = ((parent_time - accumulated_time) / parent_time) * 100
            else:
                unaccounted_fraction = 0
            blockTime = "%s (%.3f %%) :: %.3f ms" % ("Unaccounted", unaccounted_fraction, parent_time - accumulated_time)

            self.displayProfileString(xOffset, yStart, blockTime)
            yStart += yIncr

            blockTime = "-------------------------------------------------"
            self.displayProfileString(xOffset, yStart, blockTime)
            yStart += yIncr

    def renderscene(self, pass_no):
        rot = bt.Matrix3x3()
        rot.setIdentity()
        numObjects = self.m_dynamicsWorld.getNumCollisionObjects()
        wireColor = bt.Vector3(1, 0, 0)
        for i in xrange(numObjects):
            colObj = self.m_dynamicsWorld.getCollisionObjectArray()[i]
            body = colObj
            if isinstance(body, bt.RigidBody) and body.getMotionState():
                myMotionState = body.getMotionState()
                m = myMotionState.m_graphicsWorldTrans.getOpenGLMatrix()
                rot = myMotionState.m_graphicsWorldTrans.getBasis()
            else:
                m = colObj.getWorldTransform().getOpenGLMatrix()
                rot = colObj.getWorldTransform().getBasis()
            wireColor = bt.Vector3(1, 1, 0.5)  # wants deactivation
            if i & 1:
                wireColor = bt.Vector3(0, 0, 1)
            ## color differently for active, sleeping, wantsdeactivation states
            if colObj.getActivationState() == 1:  # active
                if i & 1:
                    wireColor += bt.Vector3(1, 0, 0)
                else:
                    wireColor += bt.Vector3(0.5, 0, 0)
            if colObj.getActivationState() == 2:  # ISLAND_SLEEPING
                if i & 1:
                    wireColor += bt.Vector3(0, 1, 0)
                else:
                    wireColor += bt.Vector3(0, 0.5, 0)

            aabbMin = bt.Vector3()
            aabbMax = bt.Vector3()
            self.m_dynamicsWorld.getBroadphase().getBroadphaseAabb(aabbMin, aabbMax)

            aabbMin -= bt.Vector3(bt.LARGE_FLOAT, bt.LARGE_FLOAT, bt.LARGE_FLOAT)
            aabbMax += bt.Vector3(bt.LARGE_FLOAT, bt.LARGE_FLOAT, bt.LARGE_FLOAT)

            if not (self.getDebugMode() & bt.IDebugDraw.DBG_DrawWireframe):
                if pass_no == 0:
                    self.m_shapeDrawer.drawOpenGL(m, colObj.getCollisionShape(), wireColor, self.getDebugMode(), aabbMin, aabbMax)
                elif pass_no == 1:
                    self.m_shapeDrawer.drawShadow(m, self.m_sundirection * rot, colObj.getCollisionShape(), aabbMin, aabbMax)
                elif pass_no == 2:
                    self.m_shapeDrawer.drawOpenGL(m, colObj.getCollisionShape(), wireColor * 0.3, 0, aabbMin, aabbMax)

    def getDynamicsWorld(self):
        return self.m_dynamicsWorld

    def initPhysics(self):
        pass

    def setDrawClusters(self, drawClusters):
        pass

    def overrideGLShapeDrawer(self, shapeDrawer):
        shapeDrawer.enableTexture(self.m_shapeDrawer.hasTextureEnabled())
        self.m_shapeDrawer = shapeDrawer

    # See http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
    def setOrthographicProjection(self):
        # switch to projection mode
        glMatrixMode(GL_PROJECTION)

        # save previous matrix which contains the
        # settings for the perspective projection
        glPushMatrix()
        # reset matrix
        glLoadIdentity()
        # set a 2D orthographic projection
        gluOrtho2D(0, self.m_glutScreenWidth, 0, self.m_glutScreenHeight)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # invert the y axis, down is positive
        glScalef(1, -1, 1)
        # mover the origin from the bottom left corner
        # to the upper left corner
        glTranslatef(0, -self.m_glutScreenHeight, 0)

    def resetPerspectiveProjection(self):
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        self.updateCamera()

    def setTexturing(self, enable):
        return self.m_shapeDrawer.enableTexture(enable)

    def setShadows(self, enable):
        p = self.m_enableshadows
        self.m_enableshadows = enable
        return p

    def getTexturing(self):
        return self.m_shapeDrawer.hasTextureEnabled()

    def getShadows(self):
        return self.m_enableshadows

    def getDebugMode(self):
        return self.m_debugMode

    def setDebugMode(self, mode):
        self.m_debugMode = mode
        if self.getDynamicsWorld() and self.getDynamicsWorld().getDebugDrawer():
            self.getDynamicsWorld().getDebugDrawer().setDebugMode(mode)

    def setAzi(self, azi):
        self.m_azi = azi

    def setCameraUp(self, camUp):
        self.m_cameraUp = camUp

    def setCameraForwardAxis(self, axis):
        self.m_forwardAxis = axis

    def myinit(self):
        light_ambient = [0.2, 0.2, 0.2, 1.0]
        light_diffuse = [1.0, 1.0, 1.0, 1.0]
        light_specular = [1.0, 1.0, 1.0, 1.0]
        #      light_position is NOT default value
        light_position0 = [1.0, 10.0, 1.0, 0.0]
        light_position1 = [-1.0, -10.0, -1.0, 0.0]

        glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse)
        glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular)
        glLightfv(GL_LIGHT0, GL_POSITION, light_position0)

        glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient)
        glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse)
        glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular)
        glLightfv(GL_LIGHT1, GL_POSITION, light_position1)

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHT1)

        glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        glClearColor(0.7, 0.7, 0.7, 0)

    def toggleIdle(self):
        if self.m_idle:
            self.m_idle = False
        else:
            self.m_idle = True

    def updateCamera(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        rele = self.m_ele * 0.01745329251994329547  # rads per deg
        razi = self.m_azi * 0.01745329251994329547  # rads per deg

        rot = bt.Quaternion(self.m_cameraUp, razi)

        eyePos = bt.Vector3(0, 0, 0)
        eyePos[self.m_forwardAxis] = -self.m_cameraDistance

        forward = bt.Vector3(eyePos[0], eyePos[1], eyePos[2])
        if forward.length2() < bt.SIMD_EPSILON:
            forward.setValue(1, 0, 0)
        right = self.m_cameraUp.cross(forward)
        roll = bt.Quaternion(right, -rele)

        eyePos = bt.Matrix3x3(rot) * bt.Matrix3x3(roll) * eyePos

        self.m_cameraPosition[0] = eyePos.getX()
        self.m_cameraPosition[1] = eyePos.getY()
        self.m_cameraPosition[2] = eyePos.getZ()
        self.m_cameraPosition += self.m_cameraTargetPosition

        if self.m_glutScreenWidth == 0 and self.m_glutScreenHeight == 0:
            return

        extents = bt.Vector3()

        aspect = self.m_glutScreenWidth / self.m_glutScreenHeight
        extents.setValue(aspect * 1.0, 1.0, 0)

        if self.m_ortho:
            # reset matrix
            glLoadIdentity()

            extents *= self.m_cameraDistance
            lower = self.m_cameraTargetPosition - extents
            upper = self.m_cameraTargetPosition + extents
            glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(), -1000, 1000)

            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
        else:
            glFrustum(-aspect * self.m_frustumZNear, aspect * self.m_frustumZNear, -self.m_frustumZNear, self.m_frustumZNear, self.m_frustumZNear, self.m_frustumZFar)
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            gluLookAt(self.m_cameraPosition[0], self.m_cameraPosition[1], self.m_cameraPosition[2], self.m_cameraTargetPosition[0], self.m_cameraTargetPosition[1], self.m_cameraTargetPosition[2], self.m_cameraUp.getX(), self.m_cameraUp.getY(), self.m_cameraUp.getZ())

    def getCameraPosition(self):
        return self.m_cameraPosition

    def getCameraTargetPosition(self):
        return self.m_cameraTargetPosition

    def getDeltaTimeMicroseconds(self):
        if bt.USE_BT_CLOCK:
            dt = self.m_clock.getTimeMicroseconds()
            self.m_clock.reset()
            return dt
        else:
            return bt.Scalar(16666.)

    def setFrustumZPlanes(self, zNear, zFar):
        self.m_frustumZNear = zNear
        self.m_frustumZFar = zFar

    # glut callbacks

    def getCameraDistance(self):
        return self.m_cameradistance

    def setCameraDistance(self, dist):
        self.m_cameraDistance = dist

    def moveAndDisplay(self):
        if not self.m_idle:
            self.clientMoveAndDisplay()
        else:
            self.displayCallback()

    def clientMoveAndDisplay(self):
        pass

    def clientResetScene(self):
        global gNumDeepPenetrationChecks, gNumGjkChecks, gNumClampedCcdMotions

        self.removePickingConstraint()

        if SHOW_NUM_DEEP_PENETRATIONS:
            gNumDeepPenetrationChecks = 0
            gNumGjkChecks = 0

        gNumClampedCcdMotions = 0
        numObjects = 0

        if self.m_dynamicsWorld:
            numConstraints = self.m_dynamicsWorld.getNumConstraints()
            for i in xrange(numConstraints):
                self.m_dynamicsWorld.getConstraint(0).setEnabled(true)
            numObjects = self.m_dynamicsWorld.getNumCollisionObjects()

            ## create a copy of the array, not a reference!
            copyArray = bt.CollisionObjectArray(self.m_dynamicsWorld.getCollisionObjectArray())

            for i in xrange(numObjects):
                colObj = copyArray[i]
                body = colObj
                if isinstance(body, bt.RigidBody):
                    if body.getMotionState():
                        myMotionState = body.getMotionState()
                        myMotionState.m_graphicsWorldTrans = myMotionState.m_startWorldTrans
                        body.setCenterOfMassTransform(myMotionState.m_graphicsWorldTrans)
                        colObj.setInterpolationWorldTransform(myMotionState.m_startWorldTrans)
                        colObj.forceActivationState(bt.ACTIVE_TAG)
                        colObj.activate()
                        colObj.setDeactivationTime(0)
                    # remove cached contact points (this is not necessary if all objects have been removed from the dynamics world)
                    if self.m_dynamicsWorld.getBroadphase().getOverlappingPairCache():
                        self.m_dynamicsWorld.getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(colObj.getBroadphaseHandle(), getDynamicsWorld().getDispatcher())

                    if not body.isStaticObject():
                        body.setLinearVelocity(bt.Vector3(0, 0, 0))
                        body.setAngularVelocity(bt.Vector3(0, 0, 0))

            ## reset some internal cached data in the broadphase
            self.m_dynamicsWorld.getBroadphase().resetPool(self.getDynamicsWorld().getDispatcher())
            self.m_dynamicsWorld.getConstraintSolver().reset()

    # Demo functions

    def setShootBoxShape(self):
        if not self.m_shootBoxShape:
            box = bt.BoxShape(bt.Vector3(0.5, 0.5, 0.5))
            self.m_shootBoxShape = box

    def shootBox(self, destination):
        if self.m_dynamicsWorld:
            mass = 1
            startTransform = bt.Transform()
            startTransform.setIdentity()
            camPos = self.getCameraPosition()
            startTransform.setOrigin(camPos)

            self.setShootBoxShape()

            body = self.localCreateRigidBody(mass, startTransform, self.m_shootBoxShape)
            body.setLinearFactor(bt.Vector3(1, 1, 1))

            linVel = bt.Vector3(destination[0] - camPos[0], destination[1] - camPos[1], destination[2] - camPos[2])
            linVel.normalize()
            linVel *= self.m_ShootBoxInitialSpeed

            body.getWorldTransform().setOrigin(camPos)
            body.getWorldTransform().setRotation(bt.Quaternion(0, 0, 0, 1))
            body.setLinearVelocity(linVel)
            body.setAngularVelocity(bt.Vector3(0, 0, 0))
            body.setCcdMotionThreshold(0.5)
            body.setCcdSweptSphereRadius(0.4)  # value should be smaller (embedded) than the half extends of the box (see ::setShootBoxShape)

    def getRayTo(self, x, y):
        if self.m_ortho:
            aspect = self.m_glutScreenWidth / self.m_glutScreenHeight
            extents = bt.Vector3(aspect * 1.0, 1.0, 0)

            extents *= self.m_cameraDistance
            lower = self.m_cameraTargetPosition - extents
            upper = self.m_cameraTargetPosition + extents

            u = x / self.m_glutScreenWidth
            v = (self.m_glutScreenHeight - y) / self.m_glutScreenHeight

            p = bt.Vector3((1.0 - u) * lower.getX() + u * upper.getX(), (1.0 - v) * lower.getY() + v * upper.getY(), self.m_cameraTargetPosition.getZ())
            return p

        top = 1
        bottom = -1
        nearPlane = 1
        tanFov = (top - bottom) * 0.5 / nearPlane
        fov = 2.0 * bt.Atan(tanFov)

        rayFrom = self.getCameraPosition()
        rayForward = self.getCameraTargetPosition() - self.getCameraPosition()
        rayForward.normalize()
        farPlane = 10000
        rayForward *= farPlane

        vertical = self.m_cameraUp

        hor = rayForward.cross(vertical)
        hor.normalize()
        vertical = hor.cross(rayForward)
        vertical.normalize()

        tanfov = math.tan(0.5 * fov)

        hor *= 2 * farPlane * tanfov
        vertical *= 2 * farPlane * tanfov

        aspect = self.m_glutScreenWidth / self.m_glutScreenHeight

        hor *= aspect

        rayToCenter = rayFrom + rayForward
        dHor = hor * 1 / self.m_glutScreenWidth
        dVert = vertical * 1 / self.m_glutScreenHeight

        rayTo = rayToCenter - 0.5 * hor + 0.5 * vertical
        rayTo += x * dHor
        rayTo -= y * dVert
        return rayTo

    def localCreateRigidBody(self, mass, startTransform, shape):
        assert((not shape) or shape.getShapeType() != bt.INVALID_SHAPE_PROXYTYPE)

        # rigidbody is dynamic if and only if mass is non zero, otherwise static
        isDynamic = (mass != 0)

        localInertia = bt.Vector3(0, 0, 0)
        if isDynamic:
            shape.calculateLocalInertia(mass, localInertia)

        # using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

        if USE_MOTIONSTATE:
            myMotionState = mm(bt.DefaultMotionState(startTransform))

            cInfo = bt.RigidBody.btRigidBodyConstructionInfo(mass, myMotionState, shape, localInertia)

            body = mm(bt.RigidBody(cInfo))
            body.setContactProcessingThreshold(self.m_defaultContactProcessingThreshold)

        else:
            body = mm(bt.RigidBody(mass, 0, shape, localInertia))
            body.setWorldTransform(startTransform)

        self.m_dynamicsWorld.addRigidBody(body)

        return body

    # callback methods by glut

    def keyboardCallback(self, key, x, y):
        self.m_lastKey = None

        if not bt.NO_PROFILE:
            if key >= '1' and key <= '9':
                child = ord(key) - 0x31
                self.m_profileIterator.Enter_Child(child)
            elif key == '0':
                self.m_profileIterator.Enter_Parent()

        if key == 'q':
            if BT_USE_FREEGLUT:
                # return from glutMainLoop(), detect memory leaks etc.
                glutLeaveMainLoop()
            else:
                sys.exit(0)

        elif key == 'l':
            self.stepLeft()
        elif key == 'r':
            self.stepRight()
        elif key == 'f':
            self.stepFront()
        elif key == 'b':
            self.stepBack()
        elif key == 'z':
            self.zoomIn()
        elif key == 'x':
            self.zoomOut()
        elif key == 'i':
            self.toggleIdle()
        elif key == 'g':
            self.m_enableshadows = not self.m_enableshadows
        elif key == 'u':
            self.m_shapeDrawer.enableTexture(not self.m_shapeDrawer.enableTexture(False))
        elif key == 'h':
            self.m_debugMode ^= bt.IDebugDraw.DBG_NoHelpText
        elif key == 'w':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DrawWireframe
        elif key == 'p':
            self.m_debugMode ^= bt.IDebugDraw.DBG_ProfileTimings
        elif key == '=':
            maxSerializeBufferSize = 1024 * 1024 * 5
            serializer = bt.DefaultSerializer(maxSerializeBufferSize)
            self.m_dynamicsWorld.serialize(serializer)
            f2 = open("testFile.bullet", "wb")
            f2.write(serializer.getBuffer())
            f2.close()
        elif key == 'm':
            self.m_debugMode ^= bt.IDebugDraw.DBG_EnableSatComparison
        elif key == 'n':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DisableBulletLCP
        elif key == 'N':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DrawNormals
        elif key == 't':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DrawText
        elif key == 'y':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DrawFeaturesText
        elif key == 'a':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DrawAabb
        elif key == 'c':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DrawContactPoints
        elif key == 'C':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DrawConstraints
        elif key == 'L':
            self.m_debugMode ^= bt.IDebugDraw.DBG_DrawConstraintLimits
        elif key == 'd':
            self.m_debugMode ^= bt.IDebugDraw.DBG_NoDeactivation
            if self.m_debugMode & bt.IDebugDraw.DBG_NoDeactivation:
                bt.gDisableDeactivation = True
            else:
                bt.gDisableDeactivation = False
        elif key == 'o':
            self.m_ortho = not self.m_ortho
        elif key == 's':
            clientMoveAndDisplay()
        elif key == ' ':
            clientResetScene()
        elif key == '1':
            self.m_debugMode ^= bt.IDebugDraw.DBG_EnableCCD
        elif key == '.':
            self.shootBox(self.getRayTo(x, y))
        elif key == '+':
            self.m_ShootBoxInitialSpeed += 10
        elif key == '-':
            self.m_ShootBoxInitialSpeed -= 10
        else:
            # print "unused key : {!r}".format(key)
            pass

        if self.getDynamicsWorld() and self.getDynamicsWorld().getDebugDrawer():
            self.getDynamicsWorld().getDebugDrawer().setDebugMode(self.m_debugMode)

    def keyboardUpCallback(self, key, x, y):
        pass

    def specialKeyboard(self, key, x, y):
        pass

    def specialKeyboardUp(self, key, x, y):
        pass

    def reshape(self, w, h):
        GLDebugResetFont(w, h)

        self.m_glutScreenWidth = w
        self.m_glutScreenHeight = h

        glViewport(0, 0, w, h)
        self.updateCamera()

    def mouseFunc(self, button, state, x, y):
        global pickedBody, mousePickClamping, gOldPickingPos, gHitPos, gOldPickingDist

        if state == 0:
            self.m_mouseButtons |= 1 << button
        else:
            self.m_mouseButtons = 0

        self.m_mouseOldX = x
        self.m_mouseOldY = y

        self.updateModifierKeys()
        if (self.m_modifierKeys & BT_ACTIVE_ALT) and (state == 0):
            return

        rayTo = self.getRayTo(x, y)

        if button == 2:
            if state == 0:
                self.shootBox(rayTo)
        elif button == 0:
            if state == 0:
                # add a point to point constraint for picking
                if self.m_dynamicsWorld:
                    if self.m_ortho:
                        rayFrom = bt.Vector3(rayTo)
                        rayFrom.setZ(-100)
                    else:
                        rayFrom = self.m_cameraPosition

                    rayCallback = bt.CollisionWorld.ClosestRayResultCallback(rayFrom, rayTo)
                    self.m_dynamicsWorld.rayTest(rayFrom, rayTo, rayCallback)
                    if rayCallback.hasHit():
                        body = rayCallback.m_collisionObject
                        if body:
                            # other exclusions?
                            if not (body.isStaticObject() or body.isKinematicObject()):
                                pickedBody = body
                                pickedBody.setActivationState(bt.DISABLE_DEACTIVATION)

                                pickPos = rayCallback.m_hitPointWorld

                                localPivot = body.getCenterOfMassTransform().inverse() * pickPos

                                if (self.m_modifierKeys & BT_ACTIVE_SHIFT) == 0:
                                    tr = bt.Transform()
                                    tr.setIdentity()
                                    tr.setOrigin(localPivot)
                                    dof6 = bt.Generic6DofConstraint(body, tr, False)
                                    dof6.setLinearLowerLimit(bt.Vector3(0, 0, 0))
                                    dof6.setLinearUpperLimit(bt.Vector3(0, 0, 0))
                                    dof6.setAngularLowerLimit(bt.Vector3(0, 0, 0))
                                    dof6.setAngularUpperLimit(bt.Vector3(0, 0, 0))

                                    self.m_dynamicsWorld.addConstraint(dof6, True)
                                    self.m_pickConstraint = dof6

                                    dof6.setParam(bt.CONSTRAINT_STOP_CFM, 0.8, 0)
                                    dof6.setParam(bt.CONSTRAINT_STOP_CFM, 0.8, 1)
                                    dof6.setParam(bt.CONSTRAINT_STOP_CFM, 0.8, 2)
                                    dof6.setParam(bt.CONSTRAINT_STOP_CFM, 0.8, 3)
                                    dof6.setParam(bt.CONSTRAINT_STOP_CFM, 0.8, 4)
                                    dof6.setParam(bt.CONSTRAINT_STOP_CFM, 0.8, 5)

                                    dof6.setParam(bt.CONSTRAINT_STOP_ERP, 0.1, 0)
                                    dof6.setParam(bt.CONSTRAINT_STOP_ERP, 0.1, 1)
                                    dof6.setParam(bt.CONSTRAINT_STOP_ERP, 0.1, 2)
                                    dof6.setParam(bt.CONSTRAINT_STOP_ERP, 0.1, 3)
                                    dof6.setParam(bt.CONSTRAINT_STOP_ERP, 0.1, 4)
                                    dof6.setParam(bt.CONSTRAINT_STOP_ERP, 0.1, 5)
                                else:
                                    p2p = bt.Point2PointConstraint(body, localPivot)
                                    self.m_dynamicsWorld.addConstraint(p2p, True)
                                    self.m_pickConstraint = p2p
                                    p2p.m_setting.m_impulseClamp = mousePickClamping
                                    # very weak constraint for picking
                                    p2p.m_setting.m_tau = 0.001

                                # save mouse position for dragging
                                gOldPickingPos = rayTo
                                gHitPos = pickPos

                                gOldPickingDist = (pickPos - rayFrom).length()
            else:
                self.removePickingConstraint()

    def mouseMotionFunc(self, x, y):
        global gOldPickingDist

        if self.m_pickConstraint:
            # move the constraint pivot

            if self.m_pickConstraint.getConstraintType() == bt.D6_CONSTRAINT_TYPE:
                pickCon = self.m_pickConstraint

                # keep it at the same picking distance

                newRayTo = self.getRayTo(x, y)
                oldPivotInB = pickCon.getFrameOffsetA().getOrigin()

                if self.m_ortho:
                    newPivotB = bt.Vector3(oldPivotInB)
                    newPivotB.setX(newRayTo.getX())
                    newPivotB.setY(newRayTo.getY())
                else:
                    rayFrom = self.m_cameraPosition
                    dir = newRayTo - rayFrom
                    dir.normalize()
                    dir *= gOldPickingDist

                    newPivotB = rayFrom + dir
                pickCon.getFrameOffsetA().setOrigin(newPivotB)

            else:
                pickCon = self.m_pickConstraint

                # keep it at the same picking distance

                newRayTo = self.getRayTo(x, y)
                oldPivotInB = pickCon.getPivotInB()
                if m_ortho:
                    newPivotB = bt.Vector3(oldPivotInB)
                    newPivotB.setX(newRayTo.getX())
                    newPivotB.setY(newRayTo.getY())
                else:
                    rayFrom = self.m_cameraPosition
                    dir = newRayTo - rayFrom
                    dir.normalize()
                    dir *= gOldPickingDist

                    newPivotB = rayFrom + dir
                pickCon.setPivotB(newPivotB)

        dx = x - self.m_mouseOldX
        dy = y - self.m_mouseOldY

        ## only if ALT key is pressed (Maya style)
        if self.m_modifierKeys & BT_ACTIVE_ALT:
            if self.m_mouseButtons & 2:
                hor = getRayTo(0, 0) - getRayTo(1, 0)
                vert = getRayTo(0, 0) - getRayTo(0, 1)
                multiplierX = 0.001
                multiplierY = 0.001
                if self.m_ortho:
                    multiplierX = 1
                    multiplierY = 1

                self.m_cameraTargetPosition += hor * dx * multiplierX
                self.m_cameraTargetPosition += vert * dy * multiplierY

            if (self.m_mouseButtons & (2 << 2)) and (self.m_mouseButtons & 1):
                pass
            elif self.m_mouseButtons & 1:
                self.m_azi += dx * 0.2
                self.m_azi %= 360
                self.m_ele += dy * 0.2
                self.m_ele %= 180
            elif self.m_mouseButtons & 4:
                self.m_cameraDistance -= dy * 0.02
                if self.m_cameraDistance < 0.1:
                    self.m_cameraDistance = 0.1

        self.m_mouseOldX = x
        self.m_mouseOldY = y
        self.updateCamera()

    def displayCallback(self):
        pass

    def renderme(self):
        if self.m_glutScreenWidth == 0 or self.m_glutScreenHeight == 0:
            return

        self.myinit()

        self.updateCamera()

        if self.m_dynamicsWorld:
            if self.m_enableshadows:
                glClear(GL_STENCIL_BUFFER_BIT)
                glEnable(GL_CULL_FACE)
                self.renderscene(0)

                glDisable(GL_LIGHTING)
                glDepthMask(GL_FALSE)
                glDepthFunc(GL_LEQUAL)
                glEnable(GL_STENCIL_TEST)
                glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE)
                glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFF)
                glFrontFace(GL_CCW)
                glStencilOp(GL_KEEP, GL_KEEP, GL_INCR)
                self.renderscene(1)
                glFrontFace(GL_CW)
                glStencilOp(GL_KEEP, GL_KEEP, GL_DECR)
                self.renderscene(1)
                glFrontFace(GL_CCW)

                glPolygonMode(GL_FRONT, GL_FILL)
                glPolygonMode(GL_BACK, GL_FILL)
                glShadeModel(GL_SMOOTH)
                glEnable(GL_DEPTH_TEST)
                glDepthFunc(GL_LESS)
                glEnable(GL_LIGHTING)
                glDepthMask(GL_TRUE)
                glCullFace(GL_BACK)
                glFrontFace(GL_CCW)
                glEnable(GL_CULL_FACE)
                glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE)

                glDepthFunc(GL_LEQUAL)
                glStencilFunc(GL_NOTEQUAL, 0, 0xFFFFFFFF)
                glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP)
                glDisable(GL_LIGHTING)
                self.renderscene(2)
                glEnable(GL_LIGHTING)
                glDepthFunc(GL_LESS)
                glDisable(GL_STENCIL_TEST)
                glDisable(GL_CULL_FACE)
            else:
                glDisable(GL_CULL_FACE)
                self.renderscene(0)

            xOffset = 10
            yStart = 20
            yIncr = 20

            glDisable(GL_LIGHTING)
            glColor3f(0, 0, 0)

            if (self.m_debugMode & bt.IDebugDraw.DBG_NoHelpText) == 0:
                self.setOrthographicProjection()

                self.showProfileInfo(xOffset, yStart, yIncr) #SLOW

                # [Porting note: The USE_QUICKPROF code that was here in the
                # original is apparently obsolete code written against a fairly
                # old version of Bullet (btProfiler has been replaced by the
                # CProfile* stuff in newer versions).  It has therefore not
                # been ported]

                self.resetPerspectiveProjection()

            glDisable(GL_LIGHTING)

        self.updateCamera()

    def swapBuffers(self):
        pass

    def updateModifierKeys(self):
        pass

    def stepLeft(self):
        self.m_azi -= STEPSIZE
        if self.m_azi < 0:
            self.m_azi += 360
        self.updateCamera()

    def stepRight(self):
        self.m_azi += STEPSIZE
        if self.m_azi >= 360:
            self.m_azi -= 360
        self.updateCamera()

    def stepFront(self):
        self.m_ele += STEPSIZE
        if self.m_ele >= 360:
            self.m_ele -= 360
        self.updateCamera()

    def stepBack(self):
        self.m_ele -= STEPSIZE
        if self.m_ele < 0:
            self.m_ele += 360
        self.updateCamera()

    def zoomIn(self):
        self.m_cameraDistance -= self.m_zoomStepSize
        self.updateCamera()
        if self.m_cameraDistance < 0.1:
            self.m_cameraDistance = 0.1

    def zoomOut(self):
        self.m_cameraDistance += self.m_zoomStepSize
        self.updateCamera()

    def isIdle(self):
        return self.m_idle

    def setIdle(self, idle):
        self.m_idle = idle

    @classmethod
    def Create(cls):
        demo = cls()
        demo.myinit()
        demo.initPhysics()
        return demo
