#include "BulletCollision/CollisionShapes/btShapeHull.h"

class _py_btShapeHull : public btShapeHull {
public:
    _py_btShapeHull(const btConvexShape *shape) : btShapeHull(shape) {}
    btAlignedObjectArray<btVector3> getVertexPointer() { return m_vertices; }
    btAlignedObjectArray<unsigned int> getIndexPointer() { return m_indices; }
};

void _py_getOpenGLMatrix(btTransform *t, uintptr_t matrix) {
    t->getOpenGLMatrix((btScalar *)matrix);
}
