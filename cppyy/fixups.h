#include "BulletCollision/CollisionShapes/btShapeHull.h"

class _py_btShapeHull : public btShapeHull {
public:
    _py_btShapeHull(const btConvexShape *shape) : btShapeHull(shape) {}
    btAlignedObjectArray<btVector3> getVertexPointer() { return m_vertices; }
    btAlignedObjectArray<unsigned int> getIndexPointer() { return m_indices; }
};

class _py_OpenGLMatrix {
public:
    btScalar m_matrix[16];
    _py_OpenGLMatrix(btTransform t) { t.getOpenGLMatrix(m_matrix); }
};
