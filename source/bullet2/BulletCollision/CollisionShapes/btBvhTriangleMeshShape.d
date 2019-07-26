module bullet2.BulletCollision.CollisionShapes.btBvhTriangleMeshShape;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btTriangleMeshShape;
import bullet2.BulletCollision.CollisionShapes.btOptimizedBvh;
import bullet2.BulletCollision.CollisionShapes.btTriangleInfoMap;
import bullet2.BulletCollision.CollisionShapes.btStridingMeshInterface;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.BulletCollision.CollisionShapes.btTriangleCallback;
import bullet2.BulletCollision.BroadphaseCollision.btQuantizedBvh;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btAlignedAllocator;
import bullet2.LinearMath.btSerializer;

///The btBvhTriangleMeshShape is a static-triangle mesh shape, it can only be used for fixed/non-moving objects.
///If you required moving concave triangle meshes, it is recommended to perform convex decomposition
///using HACD, see Bullet/Demos/ConvexDecompositionDemo.
///Alternatively, you can use btGimpactMeshShape for moving concave triangle meshes.
///btBvhTriangleMeshShape has several optimizations, such as bounding volume hierarchy and
///cache friendly traversal for PlayStation 3 Cell SPU.
///It is recommended to enable useQuantizedAabbCompression for better memory usage.
///It takes a triangle mesh as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
align(16) class btBvhTriangleMeshShape : btTriangleMeshShape
{
	btOptimizedBvh m_bvh;
	btTriangleInfoMap* m_triangleInfoMap;

	bool m_useQuantizedAabbCompression;
	bool m_ownsBvh;
/*#ifdef __clang__
	bool m_pad[11] __attribute__((unused));  ////need padding due to alignment
#else*/
	bool[11] m_pad;  ////need padding due to alignment
//#endif

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this(btStridingMeshInterface meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true);

	///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
	final this(btStridingMeshInterface meshInterface, bool useQuantizedAabbCompression, ref const(btVector3) bvhAabbMin, ref const(btVector3) bvhAabbMax, bool buildBvh = true);

	/*virtual*/ ~this();

	final bool getOwnsBvh() const
	{
		return m_ownsBvh;
	}

	final void performRaycast(btTriangleCallback callback, ref const(btVector3) raySource, ref const(btVector3) rayTarget);
	final void performConvexcast(btTriangleCallback callback, ref const(btVector3) boxSource, ref const(btVector3) boxTarget, ref const(btVector3) boxMin, ref const(btVector3) boxMax);

	override /*virtual*/ void processAllTriangles(btTriangleCallback callback, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;

	final void refitTree(ref const(btVector3) aabbMin, ref const(btVector3) aabbMax);

	///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
	final void partialRefitTree(ref const(btVector3) aabbMin, ref const(btVector3) aabbMax);

	//debugging
	override /*virtual*/ const(char*) getName() const { return "BVHTRIANGLEMESH"; }

	override /*virtual*/ void setLocalScaling(ref const(btVector3) scaling);

	final btOptimizedBvh getOptimizedBvh()
	{
		return m_bvh;
	}

    final void setOptimizedBvh(btOptimizedBvh bvh, const(btVector3) localScaling = btVector3(1, 1, 1))
    {
        setOptimizedBvh(bvh, localScaling);
    }
	final void setOptimizedBvh(btOptimizedBvh bvh, ref const(btVector3) localScaling);

	final void buildOptimizedBvh();

	final bool usesQuantizedAabbCompression() const
	{
		return m_useQuantizedAabbCompression;
	}

	final void setTriangleInfoMap(btTriangleInfoMap * triangleInfoMap)
	{
		m_triangleInfoMap = triangleInfoMap;
	}

	final const(btTriangleInfoMap)* getTriangleInfoMap() const
	{
		return m_triangleInfoMap;
	}

	final btTriangleInfoMap* getTriangleInfoMap()
	{
		return m_triangleInfoMap;
	}

	override /*virtual*/ int calculateSerializeBufferSize() const
    {
        return btTriangleMeshShapeData.sizeof;
    }

	///fills the dataBuffer and returns the struct name (and 0 on failure)
    pragma(mangle, "?serialize@btBvhTriangleMeshShape@@UEBAPEBDPEAXPEAVbtSerializer@@@Z")
	override /*virtual*/ const(char*) serialize(void* dataBuffer, btSerializer serializer) const;

	/*virtual*/ void serializeSingleBvh(btSerializer serializer) const;

	/*virtual*/ void serializeSingleTriangleInfoMap(btSerializer serializer) const;
};

// clang-format off

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btTriangleMeshShapeData
{
	btCollisionShapeData	m_collisionShapeData;

	btStridingMeshInterfaceData m_meshInterface;

	btQuantizedBvhFloatData		*m_quantizedFloatBvh;
	btQuantizedBvhDoubleData	*m_quantizedDoubleBvh;

	btTriangleInfoMapData	*m_triangleInfoMap;

	float	m_collisionMargin;

	char[4] m_pad3;
};
