module bullet2.BulletCollision.CollisionShapes.btTriangleMeshShape;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btConcaveShape;
import bullet2.BulletCollision.CollisionShapes.btStridingMeshInterface;
import bullet2.BulletCollision.CollisionShapes.btTriangleCallback;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;

///The btTriangleMeshShape is an internal concave triangle mesh interface. Don't use this class directly, use btBvhTriangleMeshShape instead.
align(16) class btTriangleMeshShape : btConcaveShape
{
protected:
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;
	btStridingMeshInterface m_meshInterface;

	///btTriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
	///Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
	final this(btStridingMeshInterface meshInterface);

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	/*virtual*/ ~this();

	/*virtual*/ btVector3 localGetSupportingVertex(ref const(btVector3) vec) const;

	/*virtual*/ btVector3 localGetSupportingVertexWithoutMargin(ref const(btVector3) vec) const
	{
		btAssert(0);
		return localGetSupportingVertex(vec);
	}

	final void recalcLocalAabb();

	override /*virtual*/ void getAabb(ref const(btTransform) t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	override /*virtual*/ void processAllTriangles(btTriangleCallback callback, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;

	override /*virtual*/ void calculateLocalInertia(btScalar mass, ref btVector3 inertia) const;

	override /*virtual*/ void setLocalScaling(ref const(btVector3) scaling);
	override /*virtual*/ ref const(btVector3) getLocalScaling() const;

	final btStridingMeshInterface getMeshInterface()
	{
		return m_meshInterface;
	}

	final const(btStridingMeshInterface) getMeshInterface() const
	{
		return m_meshInterface;
	}

	final ref const(btVector3) getLocalAabbMin() const
	{
		return m_localAabbMin;
	}
	final ref const(btVector3) getLocalAabbMax() const
	{
		return m_localAabbMax;
	}

	//debugging
	override /*virtual*/ const(char*) getName() const { return "TRIANGLEMESH"; }
};
