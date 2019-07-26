module bullet2.BulletCollision.CollisionShapes.btCollisionShape;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btMatrix3x3;
import bullet2.LinearMath.btSerializer;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy; //for the shape types

///The btCollisionShape class provides an interface for collision shapes that can be shared among btCollisionObjects.
align(16) class btCollisionShape
{
protected:
	int m_shapeType;
	void* m_userPointer;
	int m_userIndex;

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	this()
	{
		m_shapeType = BroadphaseNativeTypes.INVALID_SHAPE_PROXYTYPE;
		m_userPointer = null;
		m_userIndex = -1;
	}

	/*virtual*/ ~this()
	{
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	/*virtual*/ abstract void getAabb(ref const(btTransform) t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	/*virtual*/ void getBoundingSphere(ref btVector3 center, ref btScalar radius) const;

	///getAngularMotionDisc returns the maximum radius needed for Conservative Advancement to handle time-of-impact with rotations.
	/*virtual*/ btScalar getAngularMotionDisc() const;

	/*virtual*/ btScalar getContactBreakingThreshold(btScalar defaultContactThresholdFactor) const;

	///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result is conservative
	final void calculateTemporalAabb(ref const(btTransform) curTrans, ref const(btVector3) linvel, ref const(btVector3) angvel, btScalar timeStep, ref btVector3 temporalAabbMin, ref btVector3 temporalAabbMax) const;

	final /*SIMD_FORCE_INLINE*/ bool isPolyhedral() const
	{
		return btBroadphaseProxy.isPolyhedral(getShapeType());
	}

	final /*SIMD_FORCE_INLINE*/ bool isConvex2d() const
	{
		return btBroadphaseProxy.isConvex2d(getShapeType());
	}

	final /*SIMD_FORCE_INLINE*/ bool isConvex() const
	{
		return btBroadphaseProxy.isConvex(getShapeType());
	}
	final /*SIMD_FORCE_INLINE*/ bool isNonMoving() const
	{
		return btBroadphaseProxy.isNonMoving(getShapeType());
	}
	final /*SIMD_FORCE_INLINE*/ bool isConcave() const
	{
		return btBroadphaseProxy.isConcave(getShapeType());
	}
	final /*SIMD_FORCE_INLINE*/ bool isCompound() const
	{
		return btBroadphaseProxy.isCompound(getShapeType());
	}

	final /*SIMD_FORCE_INLINE*/ bool isSoftBody() const
	{
		return btBroadphaseProxy.isSoftBody(getShapeType());
	}

	///isInfinite is used to catch simulation error (aabb check)
	final /*SIMD_FORCE_INLINE*/ bool isInfinite() const
	{
		return btBroadphaseProxy.isInfinite(getShapeType());
	}

//#ifndef __SPU__
	/*virtual*/ abstract void setLocalScaling(ref const(btVector3) scaling);
	/*virtual*/ abstract ref const(btVector3) getLocalScaling();
	/*virtual*/ abstract void calculateLocalInertia(btScalar mass, ref btVector3 inertia);

	//debugging support
	/*virtual*/ abstract const(char*) getName();
//#endif  //__SPU__

	final int getShapeType() const
	{
		return m_shapeType;
	}

	///the getAnisotropicRollingFrictionDirection can be used in combination with setAnisotropicFriction
	///See Bullet/Demos/RollingFrictionDemo for an example
	/*virtual*/ btVector3 getAnisotropicRollingFrictionDirection() const
	{
		return btVector3(1, 1, 1);
	}
	/*virtual*/ abstract void setMargin(btScalar margin);
	/*virtual*/ abstract btScalar getMargin();

	///optional user data pointer
	final void setUserPointer(void* userPtr)
	{
		m_userPointer = userPtr;
	}

	final void* getUserPointer() //const
	{
		return m_userPointer;
	}
	final void setUserIndex(int index)
	{
		m_userIndex = index;
	}

	final int getUserIndex() const
	{
		return m_userIndex;
	}

	/*virtual*/ int calculateSerializeBufferSize() const
    {
        return btCollisionShapeData.sizeof;
    }

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	pragma(mangle, "?serialize@btCollisionShape@@UEBAPEBDPEAXPEAVbtSerializer@@@Z")
	/*virtual*/ const(char*) serialize(void* dataBuffer, btSerializer serializer) const;

	/*virtual*/ void serializeSingleShape(btSerializer serializer) const;
};

// clang-format off
// parser needs * with the name
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btCollisionShapeData
{
	char	*m_name;
	int		m_shapeType;
	char[4]	m_padding;
};
