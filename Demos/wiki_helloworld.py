#!/usr/bin/env pypy

# This is a python port of the "Hello World" tutorial code from:
#
#     http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World
#
# ..using the cppyy-bullet bindings for PyPy
#
# (For comparison, the original C++ version can be found in the same directory
# as this file, under the name "wiki_helloworld.cpp")
#
# It illustrates, very simply, how to initialize Bullet, set up a dynamics
# world, and simulate a falling sphere, with a minimum of extra bells and
# whistles.
#
# In particular, this PyPy version illustrates that it is (or at least should
# be) possible to take existing C++ Bullet code and translate it almost 1-to-1
# to the equivalent code in Python.

from __future__ import division

from bullet import bt

if __name__ == "__main__":
    broadphase = bt.DbvtBroadphase()

    collisionConfiguration = bt.DefaultCollisionConfiguration()
    dispatcher = bt.CollisionDispatcher(collisionConfiguration)

    solver = bt.SequentialImpulseConstraintSolver()

    dynamicsWorld = bt.DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration)

    dynamicsWorld.setGravity(bt.Vector3(0, -10, 0))

    groundShape = bt.StaticPlaneShape(bt.Vector3(0, 1, 0), 1)

    fallShape = bt.SphereShape(1)

    groundMotionState = bt.DefaultMotionState(bt.Transform(bt.Quaternion(0, 0, 0, 1), bt.Vector3(0, -1, 0)))
    groundRigidBodyCI = bt.RigidBody.btRigidBodyConstructionInfo(0, groundMotionState, groundShape, bt.Vector3(0, 0, 0))
    groundRigidBody = bt.RigidBody(groundRigidBodyCI)
    dynamicsWorld.addRigidBody(groundRigidBody)

    fallMotionState = bt.DefaultMotionState(bt.Transform(bt.Quaternion(0, 0, 0, 1), bt.Vector3(0, 50, 0)))
    mass = 1
    fallInertia = bt.Vector3(0, 0, 0)
    fallShape.calculateLocalInertia(mass, fallInertia)
    fallRigidBodyCI = bt.RigidBody.btRigidBodyConstructionInfo(mass, fallMotionState, fallShape, fallInertia)
    fallRigidBody = bt.RigidBody(fallRigidBodyCI)
    dynamicsWorld.addRigidBody(fallRigidBody)

    trans = bt.Transform()

    for i in xrange(300):
        dynamicsWorld.stepSimulation(1.0 / 60, 10)

        fallRigidBody.getMotionState().getWorldTransform(trans)

        print "sphere height: {:0.6}".format(trans.getOrigin().getY())

    dynamicsWorld.removeRigidBody(fallRigidBody)
    dynamicsWorld.removeRigidBody(groundRigidBody)
