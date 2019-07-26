module bullet2.BulletCollision.CollisionShapes.btConvexShape;

extern (C++):


import bullet2.BulletCollision.CollisionShapes.btCollisionShape;

import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btMatrix3x3;
import bullet2.LinearMath.btScalar;
//import bullet2.LinearMath.btAlignedAllocator;
import bullet2.BulletCollision.CollisionShapes.btCollisionMargin;

enum MAX_PREFERRED_PENETRATION_DIRECTIONS = 10;

/// The btConvexShape is an abstract shape interface, implemented by all convex shapes such as btBoxShape, btConvexHullShape etc.
/// It describes general convex shapes using the localGetSupportingVertex interface, used by collision detectors such as btGjkPairDetector.

align(16) class btConvexShape : btCollisionShape
{
public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this();

	/*virtual*/ ~this();

	abstract /*virtual*/ btVector3 localGetSupportingVertex(ref const(btVector3) vec) const;

////////
/+#ifndef __SPU__
	abstract /*virtual*/ btVector3 localGetSupportingVertexWithoutMargin(ref const(btVector3) vec) const;
#endif  //#ifndef __SPU__+/

	final btVector3 localGetSupportVertexWithoutMarginNonvirtual(ref const(btVector3) vec) const;
	final btVector3 localGetSupportVertexNonvirtual(ref const(btVector3) vec) const;
	final btScalar getMarginNonvirtual() const;
	final void getAabbNonvirtual(const ref btTransform t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	/*virtual*/ void project(const ref btTransform trans, ref const(btVector3) dir, ref btScalar minProj, ref btScalar maxProj, ref btVector3 witnesPtMin, ref btVector3 witnesPtMax) const;

	//notice that the vectors should be unit length
	abstract /*virtual*/ void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const;

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	override void getAabb(const ref btTransform t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	abstract /*virtual*/ void getAabbSlow(const ref btTransform t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	override /*virtual*/ void setLocalScaling(ref const(btVector3) scaling);
	override /*virtual*/ ref const(btVector3) getLocalScaling() const;

	override /*virtual*/ void setMargin(btScalar margin);

	override /*virtual*/ btScalar getMargin() const;

	abstract /*virtual*/ int getNumPreferredPenetrationDirections() const;

	abstract /*virtual*/ void getPreferredPenetrationDirection(int index, ref btVector3 penetrationVector) const;
};
