#!/usr/bin/env pypy

# NOTE: This file is a port of the HelloWorld.cpp file from the Bullet
# distribution to Python (using cppyy-bullet).  As such, it is not a great
# example of how to program in Python (it is pretty much a line-by-line
# transliteration of C++ code, which often has different conventions than
# "good" Python code).
#
# The original HelloWorld.cpp file is licensed under the following terms:
#
# Bullet Continuous Collision Detection and Physics Library
# Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/
#
# This software is provided 'as-is', without any express or implied warranty.
# In no event will the authors be held liable for any damages arising from the
# use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
#
# 1. The origin of this software must not be misrepresented; you must not claim
# that you wrote the original software. If you use this software in a product,
# an acknowledgment in the product documentation would be appreciated but is
# not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

## -----includes_start-----

from bullet import bt

## This is a Hello World program for running a basic Bullet physics simulation

if __name__ == "__main__":
    ## -----initialization_start-----

    # [Porting note: Unlike C++, we don't need to explicitly delete objects
    # when we're done with them, because Python will do that for us.  However,
    # since Python doesn't know what objects are still being used by the C++
    # side of things, it may garbage collect them too early.  The following
    # memory manager object (and the calls to mm.keep() are an easy way to make
    # sure this doesn't happen.]
    mm = bt.memory_manager()

    # collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = bt.DefaultCollisionConfiguration()

    # use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = bt.CollisionDispatcher(collisionConfiguration)

    # bt.DbvtBroadphase is a good general purpose broadphase. You can also try out bt.Axis3Sweep.
    overlappingPairCache = bt.DbvtBroadphase()

    # the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = bt.SequentialImpulseConstraintSolver()

    dynamicsWorld = bt.DiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration)

    dynamicsWorld.setGravity(bt.Vector3(0, -10, 0))

    ## -----initialization_end-----

    # create a few basic rigid bodies
    groundShape = bt.BoxShape(bt.Vector3(50., 50., 50.))

    # make sure to re-use collision shapes among rigid bodies whenever possible!

    groundTransform = bt.Transform()
    groundTransform.setIdentity()
    groundTransform.setOrigin(bt.Vector3(0, -56, 0))

    mass = 0.

    # rigidbody is dynamic if and only if mass is non zero, otherwise static
    isDynamic = (mass != 0.)

    localInertia = bt.Vector3(0, 0, 0)
    if isDynamic:
        groundShape.calculateLocalInertia(mass, localInertia)

    # using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    myMotionState = bt.DefaultMotionState(groundTransform)
    mm.keep(myMotionState)
    rbInfo = bt.RigidBody.btRigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia)
    body = bt.RigidBody(rbInfo)
    mm.keep(body)

    # add the body to the dynamics world
    dynamicsWorld.addRigidBody(body)

    # create a dynamic rigidbody

    # bt.CollisionShape* colShape = new bt.BoxShape(bt.Vector3(1, 1, 1))
    colShape = bt.SphereShape(1.)

    ##  Create Dynamic Objects
    startTransform = bt.Transform()
    startTransform.setIdentity()

    mass = 1.

    # rigidbody is dynamic if and only if mass is non zero, otherwise static
    isDynamic = (mass != 0.)

    localInertia = bt.Vector3(0, 0, 0)
    if isDynamic:
        colShape.calculateLocalInertia(mass, localInertia)

    startTransform.setOrigin(bt.Vector3(2, 10, 0))

    # using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    myMotionState = bt.DefaultMotionState(startTransform)
    mm.keep(myMotionState)
    rbInfo = bt.RigidBody.btRigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia)
    body = bt.RigidBody(rbInfo)
    mm.keep(body)

    dynamicsWorld.addRigidBody(body)

    ##  Do some simulation

    ## -----stepsimulation_start-----
    for i in xrange(100):
        dynamicsWorld.stepSimulation(1.0 / 60., 10)

        # print positions of all objects
        # [Porting note: Not sure why we're doing this in reverse order, but
        # that's what the original HelloWorld.cpp does, so keeping it
        # consistent here]
        for body in reversed(dynamicsWorld.getCollisionObjectArray()):
            if body.getMotionState():
                trans = bt.Transform()
                body.getMotionState().getWorldTransform(trans)
                print "world pos = %f,%f,%f" % (trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ())

    ## -----stepsimulation_end-----

    # cleanup in the reverse order of creation/initialization

    ## -----cleanup_start-----

    # remove the rigidbodies from the dynamics world and delete them
    for body in list(dynamicsWorld.getCollisionObjectArray()):
        dynamicsWorld.removeCollisionObject(body)

    # [Porting note: We don't technically need to do this.  An anonymous memory
    # manager will automatically release all objects when we go out of scope
    # and it gets garbage collected anyway, but it doesn't hurt to be explicit]
    mm.release_all()

    ## -----cleanup_end-----
