module bullet2.BulletCollision.CollisionShapes.btTriangleMesh;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btTriangleIndexVertexArray;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btAlignedObjectArray;

///The btTriangleMesh class is a convenience class derived from btTriangleIndexVertexArray, that provides storage for a concave triangle mesh. It can be used as data for the btBvhTriangleMeshShape.
///It allows either 32bit or 16bit indices, and 4 (x-y-z-w) or 3 (x-y-z) component vertices.
///If you want to share triangle/index data between graphics mesh and collision mesh (btBvhTriangleMeshShape), you can directly use btTriangleIndexVertexArray or derive your own class from btStridingMeshInterface.
///Performance of btTriangleMesh and btTriangleIndexVertexArray used in a btBvhTriangleMeshShape is the same.
class btTriangleMesh : btTriangleIndexVertexArray
{
	btAlignedObjectArray!btVector3 m_4componentVertices;
	btAlignedObjectArray!btScalar m_3componentVertices;

	btAlignedObjectArray!uint m_32bitIndices;
	btAlignedObjectArray!ushort m_16bitIndices;
	bool m_use32bitIndices;
	bool m_use4componentVertices;

public:
	btScalar m_weldingThreshold;

	final this(bool use32bitIndices = true, bool use4componentVertices = true);

	final bool getUse32bitIndices() const
	{
		return m_use32bitIndices;
	}

	final bool getUse4componentVertices() const
	{
		return m_use4componentVertices;
	}
	///By default addTriangle won't search for duplicate vertices, because the search is very slow for large triangle meshes.
	///In general it is better to directly use btTriangleIndexVertexArray instead.
	final void addTriangle(ref const(btVector3) vertex0, ref const(btVector3) vertex1, ref const(btVector3) vertex2, bool removeDuplicateVertices = false);

	///Add a triangle using its indices. Make sure the indices are pointing within the vertices array, so add the vertices first (and to be sure, avoid removal of duplicate vertices)
	final void addTriangleIndices(int index1, int index2, int index3);

	final int getNumTriangles() const;

	override /*virtual*/ void preallocateVertices(int numverts);
	override /*virtual*/ void preallocateIndices(int numindices);

	///findOrAddVertex is an internal method, use addTriangle instead
	final int findOrAddVertex(ref const(btVector3) vertex, bool removeDuplicateVertices);
	///addIndex is an internal method, use addTriangle instead
	final void addIndex(int index);
};
