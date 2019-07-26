module bullet2.BulletCollision.CollisionShapes.btTriangleCallback;

extern (C++):

import bullet2.LinearMath.btVector3;

///The btTriangleCallback provides a callback for each overlapping triangle when calling processAllTriangles.
///This callback is called by processAllTriangles for all btConcaveShape derived class, such as  btBvhTriangleMeshShape, btStaticPlaneShape and btHeightfieldTerrainShape.
class btTriangleCallback
{
public:
	/*virtual*/ ~this();
	abstract /*virtual*/ void processTriangle(btVector3* triangle, int partId, int triangleIndex) = 0;
};

class btInternalTriangleIndexCallback
{
public:
	/*virtual*/ ~this();
	abstract /*virtual*/ void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex) = 0;
};
