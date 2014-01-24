/* This file includes helper functions/classes/etc to support callbacks from
 * C++ into Python */

#include <stdint.h>

// gContactAddedCallback

typedef bool (*_cffi_gContactAddedCallback_t)(intptr_t i_cp, intptr_t i_colObj0, int partId0, int index0, intptr_t i_colObj1, int partId1, int index1);

static _cffi_gContactAddedCallback_t _python_gContactAddedCallback = NULL;

static bool _gContactAddedCallback_thunk(btManifoldPoint& cp, const btCollisionObject* colObj0, int partId0, int index0, const btCollisionObject* colObj1, int partId1, int index1) {
    intptr_t i_cp = (intptr_t)&cp;
    intptr_t i_colObj0 = (intptr_t)colObj0;
    intptr_t i_colObj1 = (intptr_t)colObj1;
    return _python_gContactAddedCallback(i_cp, i_colObj0, partId0, index0, i_colObj1, partId1, index1);
}

static void _py_set_gContactAddedCallback(intptr_t pyptr) {
    if (pyptr) {
        _python_gContactAddedCallback = (_cffi_gContactAddedCallback_t)pyptr;
        gContactAddedCallback = &_gContactAddedCallback_thunk;
    } else {
        gContactAddedCallback = NULL;
    }
}

// gContactProcessedCallback

typedef bool (*_cffi_gContactProcessedCallback_t)(intptr_t cp, intptr_t body0, intptr_t body1);

static _cffi_gContactProcessedCallback_t _python_gContactProcessedCallback = NULL;

static bool _gContactProcessedCallback_thunk(btManifoldPoint& cp, void* body0,void* body1) {
    intptr_t i_cp = (intptr_t)&cp;
    intptr_t i_body0 = (intptr_t)body0;
    intptr_t i_body1 = (intptr_t)body1;
    return _python_gContactProcessedCallback(i_cp, i_body0, i_body1);
}

static void _py_set_gContactProcessedCallback(intptr_t pyptr) {
    if (pyptr) {
        _python_gContactProcessedCallback = (_cffi_gContactProcessedCallback_t)pyptr;
        gContactProcessedCallback = &_gContactProcessedCallback_thunk;
    } else {
        gContactProcessedCallback = NULL;
    }
}

// gContactDestroyedCallback

typedef bool (*_cffi_gContactDestroyedCallback_t)(intptr_t userPersistentData);

static _cffi_gContactDestroyedCallback_t _python_gContactDestroyedCallback = NULL;

static bool _gContactDestroyedCallback_thunk(void* userPersistentData) {
    intptr_t i_userPersistentData = (intptr_t)userPersistentData;
    return _python_gContactDestroyedCallback(i_userPersistentData);
}

static void _py_set_gContactDestroyedCallback(intptr_t pyptr) {
    if (pyptr) {
        _python_gContactDestroyedCallback = (_cffi_gContactDestroyedCallback_t)pyptr;
        gContactDestroyedCallback = &_gContactDestroyedCallback_thunk;
    } else {
        gContactDestroyedCallback = NULL;
    }
}

// btTriangleCallback

typedef void (_cffi_btTriangleCallback_processTriangle_t)(intptr_t, intptr_t, intptr_t, int, int);

class _py_btTriangleCallback : public btTriangleCallback {
public:
    intptr_t _processTriangle_ptr;

    virtual void processTriangle(btVector3 *triangle, int partId, int triangleIndex) {
        _cffi_btTriangleCallback_processTriangle_t *cb = (_cffi_btTriangleCallback_processTriangle_t *)_processTriangle_ptr;
        cb((intptr_t)triangle, (intptr_t)(triangle+1), (intptr_t)(triangle+2), partId, triangleIndex);
    }
};
