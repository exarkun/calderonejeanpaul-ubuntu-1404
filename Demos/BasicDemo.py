#!/usr/bin/env python

from __future__ import division

import sys
from bullet import bt

import OpenGL
OpenGL.ERROR_CHECKING = False

from DemoOpenGL.GlutStuff import *
from DemoOpenGL.GlutDemoApplication import PlatformDemoApplication
from DemoOpenGL.GLDebugDrawer import GLDebugDrawer

# create 125 (5x5x5) dynamic object
ARRAY_SIZE_X = 5
ARRAY_SIZE_Y = 5
ARRAY_SIZE_Z = 5

# maximum number of objects (and allow user to shoot additional boxes)
MAX_PROXIES = (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z + 1024)

# scaling of the objects (0.1 = 20 centimeter boxes )
SCALING = 1.
START_POS_X = -5
START_POS_Y = -5
START_POS_Z = -3

#gDebugDraw = GLDebugDrawer()

mm = bt.memory_manager('DemoApplication')


class BasicDemo (PlatformDemoApplication):
    #btBroadphaseInterface*  m_broadphase;
    #btCollisionDispatcher*  m_dispatcher;
    #btConstraintSolver*     m_solver;
    #btDefaultCollisionConfiguration* m_collisionConfiguration;

    def __init__(self):
        PlatformDemoApplication.__init__(self)
        self.m_collisionShapes = []

    def __del__(self):
        PlatformDemoApplication.__del__(self)
        self.exitPhysics()

    def initPhysics(self):
        self.setTexturing(True)
        self.setShadows(True)

        self.setCameraDistance(SCALING * 50.)

        ## collision configuration contains default setup for memory, collision setup
        self.m_collisionConfiguration = bt.DefaultCollisionConfiguration()

        ## use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
        self.m_dispatcher = bt.CollisionDispatcher(self.m_collisionConfiguration)

        self.m_broadphase = bt.DbvtBroadphase()

        ## the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
        self.m_solver = bt.SequentialImpulseConstraintSolver()

        self.m_dynamicsWorld = bt.DiscreteDynamicsWorld(self.m_dispatcher, self.m_broadphase, self.m_solver, self.m_collisionConfiguration)
        #self.m_dynamicsWorld.setDebugDrawer(gDebugDraw) #FIXME: make this work

        self.m_dynamicsWorld.setGravity(bt.Vector3(0, -10, 0))

        ## create a few basic rigid bodies
        groundShape = bt.BoxShape(bt.Vector3(50, 50, 50))

        self.m_collisionShapes.append(groundShape)

        groundTransform = bt.Transform()
        groundTransform.setIdentity()
        groundTransform.setOrigin(bt.Vector3(0, -50, 0))

        # We can also use DemoApplication.localCreateRigidBody, but for clarity it is provided here:
        mass = 0

        # rigidbody is dynamic if and only if mass is non zero, otherwise static
        isDynamic = (mass != 0)

        localInertia = bt.Vector3(0, 0, 0)
        if isDynamic:
            groundShape.calculateLocalInertia(mass, localInertia)

        # using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        myMotionState = mm(bt.DefaultMotionState(groundTransform))
        rbInfo = bt.RigidBody.btRigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia)
        body = mm(bt.RigidBody(rbInfo))

        # add the body to the dynamics world
        self.m_dynamicsWorld.addRigidBody(body)

        # create a few dynamic rigidbodies
        #  Re-using the same collision is better for memory usage and performance

        colShape = bt.BoxShape(bt.Vector3(SCALING, SCALING, SCALING))
        self.m_collisionShapes.append(colShape)

        ## Create Dynamic Objects
        startTransform = bt.Transform()
        startTransform.setIdentity()

        mass = 1.0

        # rigidbody is dynamic if and only if mass is non zero, otherwise static
        isDynamic = (mass != 0)

        localInertia = bt.Vector3(0, 0, 0)
        if isDynamic:
            colShape.calculateLocalInertia(mass, localInertia)

        start_x = START_POS_X - ARRAY_SIZE_X / 2
        start_y = START_POS_Y
        start_z = START_POS_Z - ARRAY_SIZE_Z / 2

        for k in xrange(ARRAY_SIZE_Y):
            for i in xrange(ARRAY_SIZE_X):
                for j in xrange(ARRAY_SIZE_Z):
                    startTransform.setOrigin(SCALING * bt.Vector3(
                        2.0 * i + start_x,
                        20 + 2.0 * k + start_y,
                        2.0 * j + start_z))

                    # using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
                    myMotionState = mm(bt.DefaultMotionState(startTransform))
                    rbInfo = bt.RigidBody.btRigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia)
                    body = mm(bt.RigidBody(rbInfo))

                    self.m_dynamicsWorld.addRigidBody(body)

    def exitPhysics(self):
        # cleanup in the reverse order of creation/initialization

        # remove the rigidbodies from the dynamics world and delete them
        for obj in list(reversed(self.m_dynamicsWorld.getCollisionObjectArray())):
            if isinstance(obj, bt.RigidBody) and obj.getMotionState():
                mm.release(obj.getMotionState())
            self.m_dynamicsWorld.removeCollisionObject(obj)
            mm.release(obj)

        # delete collision shapes
        self.m_collisionshapes = []

    def clientMoveAndDisplay(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # simple dynamics world doesn't handle fixed-time-stepping
        ms = self.getDeltaTimeMicroseconds()

        ## step the simulation
        if self.m_dynamicsWorld:
            self.m_dynamicsWorld.stepSimulation(ms / 1000000.)
            # optional but useful: debug drawing
            self.m_dynamicsWorld.debugDrawWorld()

        self.renderme()
        glFlush()
        self.swapBuffers()

    def displayCallback(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self.renderme()

        # optional but useful: debug drawing to detect problems
        if self.m_dynamicsWorld:
            self.m_dynamicsWorld.debugDrawWorld()

        glFlush()
        self.swapBuffers()

    def clientResetScene(self):
        self.exitPhysics()
        self.initPhysics()

    @staticmethod
    def Create():
        demo = BasicDemo()
        demo.myinit()
        demo.initPhysics()
        return demo


if __name__ == '__main__':
    ccdDemo = BasicDemo()
    ccdDemo.initPhysics()

    glutmain(sys.argv, 1024, 600, "Bullet Physics Demo. http://bulletphysics.org", ccdDemo)
