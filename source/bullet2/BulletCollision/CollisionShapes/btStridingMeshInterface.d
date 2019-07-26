module bullet2.BulletCollision.CollisionShapes.btStridingMeshInterface;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.BulletCollision.CollisionShapes.btTriangleCallback;
import bullet2.BulletCollision.CollisionShapes.btConcaveShape;
import bullet2.LinearMath.btSerializer;

///	The btStridingMeshInterface is the interface class for high performance generic access to triangle meshes, used in combination with btBvhTriangleMeshShape and some other collision shapes.
/// Using index striding of 3*sizeof(integer) it can use triangle arrays, using index striding of 1*sizeof(integer) it can handle triangle strips.
/// It allows for sharing graphics and collision meshes. Also it provides locking/unlocking of graphics meshes that are in gpu memory.
align(16) class btStridingMeshInterface
{
protected:
	btVector3 m_scaling;

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this()
	{
        m_scaling = btVector3(1, 1, 1);
	}

	/*virtual*/ ~this();

	/*virtual*/ void InternalProcessAllTriangles(btInternalTriangleIndexCallback callback, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;

	///brute force method to calculate aabb
	final void calculateAabbBruteForce(ref btVector3 aabbMin, ref btVector3 aabbMax);

	/// get read and write access to a subpart of a triangle mesh
	/// this subpart has a continuous array of vertices and indices
	/// in this way the mesh can be handled as chunks of memory with striding
	/// very similar to OpenGL vertexarray support
	/// make a call to unLockVertexBase when the read and write access is finished
	abstract /*virtual*/ void getLockedVertexIndexBase(ubyte** vertexbase, ref int numverts, ref PHY_ScalarType type, ref int stride, ubyte** indexbase, ref int indexstride, ref int numfaces, ref PHY_ScalarType indicestype, int subpart = 0);

	abstract /*virtual*/ void getLockedReadOnlyVertexIndexBase(const(ubyte*)* vertexbase, ref int numverts, ref PHY_ScalarType type, ref int stride, const(ubyte*)* indexbase, ref int indexstride, ref int numfaces, ref PHY_ScalarType indicestype, int subpart = 0) const;

	/// unLockVertexBase finishes the access to a subpart of the triangle mesh
	/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
	abstract /*virtual*/ void unLockVertexBase(int subpart);

	abstract /*virtual*/ void unLockReadOnlyVertexBase(int subpart) const;

	/// getNumSubParts returns the number of seperate subparts
	/// each subpart has a continuous array of vertices and indices
	abstract /*virtual*/ int getNumSubParts() const;

	abstract /*virtual*/ void preallocateVertices(int numverts);
	abstract /*virtual*/ void preallocateIndices(int numindices);

	/*virtual*/ bool hasPremadeAabb() const { return false; }
	/*virtual*/ void setPremadeAabb(ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const
	{
		//(void)aabbMin;
		//(void)aabbMax;
	}
	/*virtual*/ void getPremadeAabb(btVector3 * aabbMin, btVector3 * aabbMax) const
	{
		//(void)aabbMin;
		//(void)aabbMax;
	}

	final ref const(btVector3) getScaling() const
	{
		return m_scaling;
	}
	final void setScaling(ref const(btVector3) scaling)
	{
		m_scaling = scaling;
	}

	/*virtual*/ int calculateSerializeBufferSize() const
    {
        return btStridingMeshInterfaceData.sizeof;
    }

	///fills the dataBuffer and returns the struct name (and 0 on failure)
    pragma(mangle, "?serialize@btStridingMeshInterface@@UEBAPEBDPEAXPEAVbtSerializer@@@Z")
	/*virtual*/ const(char*) serialize(void* dataBuffer, btSerializer serializer) const;
};

struct btIntIndexData
{
	int m_value;
};

struct btShortIntIndexData
{
	short m_value;
	char[2] m_pad;
};

struct btShortIntIndexTripletData
{
	short[3] m_values;
	char[2] m_pad;
};

struct btCharIndexTripletData
{
	char[3] m_values;
	char m_pad;
};

// clang-format off

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btMeshPartData
{
	btVector3FloatData			*m_vertices3f;
	btVector3DoubleData			*m_vertices3d;

	btIntIndexData				*m_indices32;
	btShortIntIndexTripletData	*m_3indices16;
	btCharIndexTripletData		*m_3indices8;

	btShortIntIndexData			*m_indices16;//backwards compatibility

	int                     m_numTriangles;//length of m_indices = m_numTriangles
	int                     m_numVertices;
};


///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btStridingMeshInterfaceData
{
	btMeshPartData	*m_meshPartsPtr;
	btVector3FloatData	m_scaling;
	int	m_numMeshParts;
	char[4] m_padding;
};
