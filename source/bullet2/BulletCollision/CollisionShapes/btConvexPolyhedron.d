module bullet2.BulletCollision.CollisionShapes.btConvexPolyhedron;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btAlignedObjectArray;

enum TEST_INTERNAL_OBJECTS = 1;

struct btFace
{
	btAlignedObjectArray!int m_indices;
	//	btAlignedObjectArray<int>	m_connectedFaces;
	btScalar[4] m_plane;
};

align(16) class btConvexPolyhedron
{
public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	this();
	/*virtual*/ ~this();

	btAlignedObjectArray!btVector3 m_vertices;
	btAlignedObjectArray!btFace m_faces;
	btAlignedObjectArray!btVector3 m_uniqueEdges;

	btVector3 m_localCenter;
	btVector3 m_extents;
	btScalar m_radius;
	btVector3 mC;
	btVector3 mE;

	final void initialize();
	final void initialize2();
	final bool testContainment() const;

	final void project(ref const(btTransform) trans, ref const(btVector3) dir, ref btScalar minProj, ref btScalar maxProj, ref btVector3 witnesPtMin, ref btVector3 witnesPtMax) const;
};
