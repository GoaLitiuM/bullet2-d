module bullet2.BulletCollision.CollisionShapes.btOptimizedBvh;

extern (C++):

import bullet2.BulletCollision.BroadphaseCollision.btQuantizedBvh;

import bullet2.BulletCollision.CollisionShapes.btStridingMeshInterface;
import bullet2.LinearMath.btVector3;

///The btOptimizedBvh extends the btQuantizedBvh to create AABB tree for triangle meshes, through the btStridingMeshInterface.
align(16) class btOptimizedBvh : btQuantizedBvh
{
public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

protected:
public:
	this();

	/*virtual*/ ~this();

	final void build(btStridingMeshInterface triangles, bool useQuantizedAabbCompression, ref const(btVector3) bvhAabbMin, ref const(btVector3) bvhAabbMax);

	final void refit(btStridingMeshInterface triangles, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax);

	final void refitPartial(btStridingMeshInterface triangles, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax);

	final void updateBvhNodes(btStridingMeshInterface meshInterface, int firstNode, int endNode, int index);

	/// Data buffer MUST be 16 byte aligned
	/*virtual*/ bool serializeInPlace(void* o_alignedDataBuffer, uint i_dataBufferSize, bool i_swapEndian) const
	{
		return btQuantizedBvh.serialize(o_alignedDataBuffer, i_dataBufferSize, i_swapEndian);
	}

	///deSerializeInPlace loads and initializes a BVH from a buffer in memory 'in place'
	static btOptimizedBvh deSerializeInPlace(void* i_alignedDataBuffer, uint i_dataBufferSize, bool i_swapEndian);
};
