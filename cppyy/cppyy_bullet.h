#include <btBulletDynamicsCommon.h>
#include "callbacks.h"

// btVoronoiSimplexSolver, btConvexPenetrationDepthSolver,
// btSimulationIslandManager, and btPoolAllocator are referenced in various
// places, but the header files where they're actually defined are never
// included from anything..
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <LinearMath/btPoolAllocator.h>

// The following are referenced but never defined in the public headers at all,
// so we just create dummy definitions for them for Reflex's sake:
struct btClockData {};
struct InplaceSolverIslandCallback {};
struct btCollisionResult {};
struct btConvexPolyhedron {};

// Explicit template instantiation so that genreflex can see them:

template class btAlignedObjectArray<btDbvtProxy*>;
template class btAxisSweep3Internal<unsigned int>::Edge;
template class btAxisSweep3Internal<unsigned int>::Handle;
template class btAxisSweep3Internal<unsigned short>::Edge;
template class btAxisSweep3Internal<unsigned short>::Handle;

// Macros/Constants we want to make available to the Python side

static btScalar _py_BT_LARGE_FLOAT = BT_LARGE_FLOAT;
