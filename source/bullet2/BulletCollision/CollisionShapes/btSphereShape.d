module bullet2.BulletCollision.CollisionShapes.btSphereShape;

extern (C++):

import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.BulletCollision.CollisionShapes.btConvexInternalShape;
//import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
//import bullet2.LinearMath.btAabbUtil2;

///The btSphereShape implements an implicit sphere, centered around a local origin with radius.
align(16) class btSphereShape : btConvexInternalShape
{
public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this(btScalar radius)
	{
        super();
		m_shapeType = BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
		m_localScaling.setValue(1.0, 1.0, 1.0);
		m_implicitShapeDimensions.setZero();
		m_implicitShapeDimensions.setX(radius);
		m_collisionMargin = radius;
		m_padding = 0;
	}

	override /*virtual*/ btVector3 localGetSupportingVertex(ref const(btVector3) vec) const;
	/*virtual*/ btVector3 localGetSupportingVertexWithoutMargin(ref const(btVector3) vec) const;
	//notice that the vectors should be unit length
	override /*virtual*/ void batchedUnitVectorGetSupportingVertexWithoutMargin(const(btVector3)* vectors, btVector3* supportVerticesOut, int numVectors) const;

	override /*virtual*/ void calculateLocalInertia(btScalar mass, ref btVector3 inertia) const;

	override /*virtual*/ void getAabb(ref const(btTransform) t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	final btScalar getRadius() const { return m_implicitShapeDimensions.getX() * m_localScaling.getX(); }

	final void setUnscaledRadius(btScalar radius)
	{
		m_implicitShapeDimensions.setX(radius);
		btConvexInternalShape.setMargin(radius);
	}

	//debugging
	override /*virtual*/ const(char*) getName() const { return "SPHERE"; }

	override /*virtual*/ void setMargin(btScalar margin)
	{
		btConvexInternalShape.setMargin(margin);
	}
	override /*virtual*/ btScalar getMargin() const
	{
		//to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		//this means, non-uniform scaling is not supported anymore
		return getRadius();
	}
};
