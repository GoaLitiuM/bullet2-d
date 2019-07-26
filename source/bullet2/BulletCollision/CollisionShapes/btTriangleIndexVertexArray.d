module bullet2.BulletCollision.CollisionShapes.btTriangleIndexVertexArray;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btStridingMeshInterface;
import bullet2.BulletCollision.CollisionShapes.btConcaveShape;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;

///The btIndexedMesh indexes a single vertex and index array. Multiple btIndexedMesh objects can be passed into a btTriangleIndexVertexArray using addIndexedMesh.
///Instead of the number of indices, we pass the number of triangles.
align(16) struct btIndexedMesh
{
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	int m_numTriangles;
	/*const(char*)*/char* m_triangleIndexBase;
	// Size in byte of the indices for one triangle (3*sizeof(index_type) if the indices are tightly packed)
	int m_triangleIndexStride;
	int m_numVertices;
	/*const(char*)*/char* m_vertexBase;
	// Size of a vertex, in bytes
	int m_vertexStride;

	// The index type is set when adding an indexed mesh to the
	// btTriangleIndexVertexArray, do not set it manually
	PHY_ScalarType m_indexType = PHY_ScalarType.PHY_INTEGER;

	// The vertex type has a default type similar to Bullet's precision mode (float or double)
	// but can be set manually if you for example run Bullet with double precision but have
	// mesh data in single precision..
    version (BT_USE_DOUBLE_PRECISION)
	    PHY_ScalarType m_vertexType = PHY_ScalarType.PHY_DOUBLE;
    else
        PHY_ScalarType m_vertexType = PHY_ScalarType.PHY_FLOAT;
};

//typedef btAlignedObjectArray<btIndexedMesh> IndexedMeshArray;
alias IndexedMeshArray = btAlignedObjectArray!btIndexedMesh;

///The btTriangleIndexVertexArray allows to access multiple triangle meshes, by indexing into existing triangle/index arrays.
///Additional meshes can be added using addIndexedMesh
///No duplicate is made of the vertex/index data, it only indexes into external vertex/index arrays.
///So keep those arrays around during the lifetime of this btTriangleIndexVertexArray.
align(16) class btTriangleIndexVertexArray : btStridingMeshInterface
{
protected:
	IndexedMeshArray m_indexedMeshes;
	int[2] m_pad;
	/*mutable*/ int m_hasAabb;  // using int instead of bool to maintain alignment
	/*mutable*/ btVector3 m_aabbMin;
	/*mutable*/ btVector3 m_aabbMax;

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this()
	{
        m_hasAabb = 0;
	}

	/*virtual*/ ~this();

	//just to be backwards compatible
	this(int numTriangles, int* triangleIndexBase, int triangleIndexStride, int numVertices, btScalar* vertexBase, int vertexStride);

    //final void addIndexedMesh(ref const(btIndexedMesh) mesh, PHY_ScalarType indexType = PHY_ScalarType.PHY_INTEGER)
    final void addIndexedMesh(ref btIndexedMesh mesh, PHY_ScalarType indexType = PHY_ScalarType.PHY_INTEGER)
	{
		m_indexedMeshes.push_back(mesh);
		m_indexedMeshes[m_indexedMeshes.size() - 1].m_indexType = indexType;
	}

	override /*virtual*/ void getLockedVertexIndexBase(ubyte** vertexbase, ref int numverts, ref PHY_ScalarType type, ref int vertexStride, ubyte** indexbase, ref int indexstride, ref int numfaces, ref PHY_ScalarType indicestype, int subpart = 0);

    pragma(mangle, "?getLockedReadOnlyVertexIndexBase@btTriangleIndexVertexArray@@UEBAXPEAPEBEAEAHAEAW4PHY_ScalarType@@10112H@Z")
	override /*virtual*/ void getLockedReadOnlyVertexIndexBase(const(ubyte*)* vertexbase, ref int numverts, ref PHY_ScalarType type, ref int vertexStride, const(ubyte*)* indexbase, ref int indexstride, ref int numfaces, ref PHY_ScalarType indicestype, int subpart = 0) const;

	/// unLockVertexBase finishes the access to a subpart of the triangle mesh
	/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
	override /*virtual*/ void unLockVertexBase(int subpart) { /*(void)subpart;*/ }

	override /*virtual*/ void unLockReadOnlyVertexBase(int subpart) const { /*(void)subpart;*/ }

	/// getNumSubParts returns the number of seperate subparts
	/// each subpart has a continuous array of vertices and indices
	override /*virtual*/ int getNumSubParts() const
	{
		return cast(int)m_indexedMeshes.size();
	}

	final ref IndexedMeshArray getIndexedMeshArray()
	{
		return m_indexedMeshes;
	}

	final ref const(IndexedMeshArray) getIndexedMeshArray() const
	{
		return m_indexedMeshes;
	}

	override /*virtual*/ void preallocateVertices(int numverts) { /*(void)numverts;*/ }
	override /*virtual*/ void preallocateIndices(int numindices) { /*(void)numindices;*/ }

	override /*virtual*/ bool hasPremadeAabb() const;
	override /*virtual*/ void setPremadeAabb(ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;
	override /*virtual*/ void getPremadeAabb(btVector3* aabbMin, btVector3* aabbMax) const;
};
