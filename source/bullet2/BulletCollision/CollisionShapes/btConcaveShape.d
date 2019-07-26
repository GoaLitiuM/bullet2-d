module bullet2.BulletCollision.CollisionShapes.btConcaveShape;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;  // for the types
import bullet2.BulletCollision.CollisionShapes.btTriangleCallback;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;

/// PHY_ScalarType enumerates possible scalar types.
/// See the btStridingMeshInterface or btHeightfieldTerrainShape for its use
enum PHY_ScalarType
{
	PHY_FLOAT,
	PHY_DOUBLE,
	PHY_INTEGER,
	PHY_SHORT,
	PHY_FIXEDPOINT88,
	PHY_UCHAR
}

///The btConcaveShape class provides an interface for non-moving (static) concave shapes.
///It has been implemented by the btStaticPlaneShape, btBvhTriangleMeshShape and btHeightfieldTerrainShape.
align(16) class btConcaveShape : btCollisionShape
{
protected:
	btScalar m_collisionMargin;

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this();

	/*virtual*/ ~this();

	abstract /*virtual*/ void processAllTriangles(btTriangleCallback callback, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;

	override /*virtual*/ btScalar getMargin() const
	{
		return m_collisionMargin;
	}
	override /*virtual*/ void setMargin(btScalar collisionMargin)
	{
		m_collisionMargin = collisionMargin;
	}
};
