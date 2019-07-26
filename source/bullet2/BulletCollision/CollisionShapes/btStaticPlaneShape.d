module bullet2.BulletCollision.CollisionShapes.btStaticPlaneShape;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btConcaveShape;
import bullet2.BulletCollision.CollisionShapes.btTriangleCallback;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btSerializer;

///The btStaticPlaneShape simulates an infinite non-moving (static) collision plane.
align(16) class btStaticPlaneShape : btConcaveShape
{
protected:
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;

	btVector3 m_planeNormal;
	btScalar m_planeConstant;
	btVector3 m_localScaling;

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this(const(btVector3) planeNormal, btScalar planeConstant)
	{
		this(planeNormal, planeConstant);
	}
	final this(ref const(btVector3) planeNormal, btScalar planeConstant);

	/*virtual*/ ~this();

	override /*virtual*/ void getAabb(ref const(btTransform) t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	override /*virtual*/ void processAllTriangles(btTriangleCallback callback, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;

	override /*virtual*/ void calculateLocalInertia(btScalar mass, ref btVector3 inertia) const;

	override /*virtual*/ void setLocalScaling(ref const(btVector3) scaling);
	override /*virtual*/ ref const(btVector3) getLocalScaling() const;

	final ref const(btVector3) getPlaneNormal() const
	{
		return m_planeNormal;
	}

	final ref const(btScalar) getPlaneConstant() const
	{
		return m_planeConstant;
	}

	//debugging
	override /*virtual*/ const(char*) getName() const { return "STATICPLANE"; }

	override /*virtual*/ int calculateSerializeBufferSize() const
	{
		return btStaticPlaneShapeData.sizeof;
	}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	override /*virtual*/ const(char*) serialize(void* dataBuffer, btSerializer serializer) const
	{
		btStaticPlaneShapeData* planeData = cast(btStaticPlaneShapeData*)dataBuffer;
		btCollisionShape.serialize(&planeData.m_collisionShapeData, serializer);

		m_localScaling.serializeFloat(planeData.m_localScaling);
		m_planeNormal.serializeFloat(planeData.m_planeNormal);
		planeData.m_planeConstant = float(m_planeConstant);

		// Fill padding with zeros to appease msan.
		planeData.m_pad[0] = 0;
		planeData.m_pad[1] = 0;
		planeData.m_pad[2] = 0;
		planeData.m_pad[3] = 0;

		return "btStaticPlaneShapeData";
	}
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btStaticPlaneShapeData
{
	btCollisionShapeData m_collisionShapeData;

	btVector3FloatData m_localScaling;
	btVector3FloatData m_planeNormal;
	float m_planeConstant;
	char[4] m_pad;
};
