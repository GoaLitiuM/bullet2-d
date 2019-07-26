module bullet2.BulletCollision.CollisionShapes.btConvexInternalShape;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btConvexShape;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btSerializer;
import bullet2.LinearMath.btAabbUtil2;
//#include "LinearMath/btAabbUtil2.h"

///The btConvexInternalShape is an internal base class, shared by most convex shape implementations.
///The btConvexInternalShape uses a default collision margin set to CONVEX_DISTANCE_MARGIN.
///This collision margin used by Gjk and some other algorithms, see also btCollisionMargin.h
///Note that when creating small shapes (derived from btConvexInternalShape),
///you need to make sure to set a smaller collision margin, using the 'setMargin' API
///There is a automatic mechanism 'setSafeMargin' used by btBoxShape and btCylinderShape
align(16) class btConvexInternalShape : btConvexShape
{
protected:
	//local scaling. collisionMargin is not scaled !
	btVector3 m_localScaling;

	btVector3 m_implicitShapeDimensions;

	btScalar m_collisionMargin;

	btScalar m_padding;

	final this();

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	/*virtual*/~this()
	{
	}

	override /*virtual*/ btVector3 localGetSupportingVertex(ref const(btVector3) vec) const;

	final ref const(btVector3) getImplicitShapeDimensions() const
	{
		return m_implicitShapeDimensions;
	}

	///warning: use setImplicitShapeDimensions with care
	///changing a collision shape while the body is in the world is not recommended,
	///it is best to remove the body from the world, then make the change, and re-add it
	///alternatively flush the contact points, see documentation for 'cleanProxyFromPairs'
	final void setImplicitShapeDimensions(ref const(btVector3) dimensions)
	{
		m_implicitShapeDimensions = dimensions;
	}

	final void setSafeMargin(btScalar minDimension, btScalar defaultMarginMultiplier = 0.1f)
	{
		btScalar safeMargin = defaultMarginMultiplier * minDimension;
		if (safeMargin < getMargin())
		{
			setMargin(safeMargin);
		}
	}
	final void setSafeMargin(ref const(btVector3) halfExtents, btScalar defaultMarginMultiplier = 0.1f)
	{
		//see http://code.google.com/p/bullet/issues/detail?id=349
		//this margin check could could be added to other collision shapes too,
		//or add some assert/warning somewhere
		btScalar minDimension = halfExtents[halfExtents.minAxis()];
		setSafeMargin(minDimension, defaultMarginMultiplier);
	}

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	override void getAabb(const ref btTransform t, ref btVector3 aabbMin, ref btVector3 aabbMax) const
	{
		getAabbSlow(t, aabbMin, aabbMax);
	}

	override /*virtual*/ void getAabbSlow(const ref btTransform t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	override /*virtual*/ void setLocalScaling(ref const(btVector3) scaling);
	override /*virtual*/ ref const(btVector3) getLocalScaling() const
	{
		return m_localScaling;
	}

	final ref const(btVector3) getLocalScalingNV() const
	{
		return m_localScaling;
	}

	override /*virtual*/ void setMargin(btScalar margin)
	{
		m_collisionMargin = margin;
	}
	override /*virtual*/ btScalar getMargin() const
	{
		return m_collisionMargin;
	}

	final btScalar getMarginNV() const
	{
		return m_collisionMargin;
	}

	override /*virtual*/ int getNumPreferredPenetrationDirections() const
	{
		return 0;
	}

	override /*virtual*/ void getPreferredPenetrationDirection(int index, ref btVector3 penetrationVector) const
	{
		//(void)penetrationVector;
		//(void)index;
		btAssert(0);
	}

	override /*virtual*/ int calculateSerializeBufferSize() const
    {
        return btConvexInternalShapeData.sizeof;
    }

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	override /*virtual*/ const(char*) serialize(void* dataBuffer, btSerializer serializer) const
    {
        btConvexInternalShapeData* shapeData = cast(btConvexInternalShapeData*)dataBuffer;
        btCollisionShape.serialize(&shapeData.m_collisionShapeData, serializer);

        m_implicitShapeDimensions.serializeFloat(shapeData.m_implicitShapeDimensions);
        m_localScaling.serializeFloat(shapeData.m_localScaling);
        shapeData.m_collisionMargin = float(m_collisionMargin);

        // Fill padding with zeros to appease msan.
        shapeData.m_padding = 0;

        return "btConvexInternalShapeData";
    }
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btConvexInternalShapeData
{
	btCollisionShapeData m_collisionShapeData;

	btVector3FloatData m_localScaling;

	btVector3FloatData m_implicitShapeDimensions;

	float m_collisionMargin;

	int m_padding;
};

///btConvexInternalAabbCachingShape adds local aabb caching for convex shapes, to avoid expensive bounding box calculations
class btConvexInternalAabbCachingShape : btConvexInternalShape
{
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;
	bool m_isLocalAabbValid;

protected:
	this();

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

	/*inline*/ void getNonvirtualAabb(const ref btTransform trans, ref btVector3 aabbMin, ref btVector3 aabbMax, btScalar margin) const
	{
		//lazy evaluation of local aabb
		btAssert(m_isLocalAabbValid);
		btTransformAabb(m_localAabbMin, m_localAabbMax, margin, trans, aabbMin, aabbMax);
	}

public:
	final override /*virtual*/ void setLocalScaling(ref const(btVector3) scaling);

	final override /*virtual*/ void getAabb(const ref btTransform t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	void recalcLocalAabb();
};
