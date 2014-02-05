#include <btBulletDynamicsCommon.h>
#include "callbacks.h"
#include "fixups.h"

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

#ifdef BT_NO_PROFILE
    static const bool _py_BT_NO_PROFILE = true;
#else
    static const bool _py_BT_NO_PROFILE = false;
#endif
#ifdef USE_BT_CLOCK
    static const bool _py_USE_BT_CLOCK = true;
#else
    static const bool _py_USE_BT_CLOCK = false;
#endif

static const btScalar _py_BT_LARGE_FLOAT = BT_LARGE_FLOAT;

static const btScalar _py_SIMD_2_PI = SIMD_2_PI;
static const btScalar _py_SIMD_PI = SIMD_PI;
static const btScalar _py_SIMD_HALF_PI = SIMD_HALF_PI;
static const btScalar _py_SIMD_RADS_PER_DEG = SIMD_RADS_PER_DEG;
static const btScalar _py_SIMD_DEGS_PER_RAD = SIMD_DEGS_PER_RAD;
static const btScalar _py_SIMDSQRT12 = SIMDSQRT12;
static const btScalar _py_SIMD_EPSILON = SIMD_EPSILON;
static const btScalar _py_SIMD_INFINITY = SIMD_INFINITY;

static const int _py_ACTIVE_TAG = ACTIVE_TAG;
static const int _py_ISLAND_SLEEPING = ISLAND_SLEEPING;
static const int _py_WANTS_DEACTIVATION = WANTS_DEACTIVATION;
static const int _py_DISABLE_DEACTIVATION = DISABLE_DEACTIVATION;
static const int _py_DISABLE_SIMULATION = DISABLE_SIMULATION;

static const int _py_BT_6DOF_FLAGS_AXIS_SHIFT = BT_6DOF_FLAGS_AXIS_SHIFT;

static const int _py_TRI_INFO_V0V1_CONVEX = TRI_INFO_V0V1_CONVEX;
static const int _py_TRI_INFO_V1V2_CONVEX = TRI_INFO_V1V2_CONVEX;
static const int _py_TRI_INFO_V2V0_CONVEX = TRI_INFO_V2V0_CONVEX;
static const int _py_TRI_INFO_V0V1_SWAP_NORMALB = TRI_INFO_V0V1_SWAP_NORMALB;
static const int _py_TRI_INFO_V1V2_SWAP_NORMALB = TRI_INFO_V1V2_SWAP_NORMALB;
static const int _py_TRI_INFO_V2V0_SWAP_NORMALB = TRI_INFO_V2V0_SWAP_NORMALB;

static const btScalar _py_CONVEX_DISTANCE_MARGIN = CONVEX_DISTANCE_MARGIN;

