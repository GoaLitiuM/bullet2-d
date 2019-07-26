module bullet2.BulletCollision.CollisionShapes.btPolyhedralConvexShape;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btConvexInternalShape;
import bullet2.BulletCollision.CollisionShapes.btConvexPolyhedron;

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btMatrix3x3;
import bullet2.LinearMath.btAabbUtil2;

///The btPolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
align(16) class btPolyhedralConvexShape : btConvexInternalShape
{
protected:
	btConvexPolyhedron m_polyhedron;

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	this();

	/*virtual*/ ~this();

	///optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
	///experimental/work-in-progress
	/*virtual*/ bool initializePolyhedralFeatures(int shiftVerticesByMargin = 0);

	pragma(mangle, "?setPolyhedralFeatures@btPolyhedralConvexShape@@UEAAXAEAVbtConvexPolyhedron@@@Z")
	/*virtual*/ void setPolyhedralFeatures(btConvexPolyhedron polyhedron);

	const(btConvexPolyhedron) getConvexPolyhedron() const
	{
		return m_polyhedron;
	}

	//brute force implementations

	/*virtual*/ btVector3 localGetSupportingVertexWithoutMargin(ref const(btVector3) vec) const;
	override /*virtual*/ void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const;

	override /*virtual*/ void calculateLocalInertia(btScalar mass, ref btVector3 inertia) const;

	abstract /*virtual*/ int getNumVertices() const;
	abstract /*virtual*/ int getNumEdges() const;
	abstract /*virtual*/ void getEdge(int i, ref btVector3 pa, ref btVector3 pb) const;
	abstract /*virtual*/ void getVertex(int i, ref btVector3 vtx) const;
	abstract /*virtual*/ int getNumPlanes() const;
	abstract /*virtual*/ void getPlane(ref btVector3 planeNormal, ref btVector3 planeSupport, int i) const;
	//	final override /*virtual*/ int getIndex(int i) const ;

	abstract /*virtual*/ bool isInside(ref const(btVector3) pt, btScalar tolerance) const;
};

///The btPolyhedralConvexAabbCachingShape adds aabb caching to the btPolyhedralConvexShape
class btPolyhedralConvexAabbCachingShape : btPolyhedralConvexShape
{
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;
	bool m_isLocalAabbValid;

protected:
	void setCachedLocalAabb(ref const(btVector3) aabbMin, ref const(btVector3) aabbMax)
	{
		m_isLocalAabbValid = true;
		m_localAabbMin = aabbMin;
		m_localAabbMax = aabbMax;
	}

	/*inline*/ void getCachedLocalAabb(ref btVector3 aabbMin, ref btVector3 aabbMax) const
	{
		btAssert(m_isLocalAabbValid);
		aabbMin = m_localAabbMin;
		aabbMax = m_localAabbMax;
	}

protected:
	this();

public:
	/*inline*/ void getNonvirtualAabb(const ref btTransform trans, ref btVector3 aabbMin, ref btVector3 aabbMax, btScalar margin) const
	{
		//lazy evaluation of local aabb
		btAssert(m_isLocalAabbValid);
		btTransformAabb(m_localAabbMin, m_localAabbMax, margin, trans, aabbMin, aabbMax);
	}

	override /*virtual*/ void setLocalScaling(ref const(btVector3) scaling);

	override /*virtual*/ void getAabb(const ref btTransform t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	final void recalcLocalAabb();
};
