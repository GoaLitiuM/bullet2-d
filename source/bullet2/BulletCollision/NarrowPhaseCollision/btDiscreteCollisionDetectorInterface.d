module bullet2.BulletCollision.NarrowPhaseCollision.btDiscreteCollisionDetectorInterface;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btIDebugDraw;

class btDiscreteCollisionDetectorInterfaceResult
{
    /*virtual*/ ~this() {}

    ///setShapeIdentifiersA/B provides experimental support for per-triangle material / custom material combiner
    abstract void setShapeIdentifiersA(int partId0, int index0);
    abstract void setShapeIdentifiersB(int partId1, int index1);
    abstract void addContactPoint(const ref btVector3 normalOnBInWorld, const ref btVector3 pointInWorld, btScalar depth);
};

/// This interface is made to be used by an iterative approach to do TimeOfImpact calculations
/// This interface allows to query for closest points and penetration depth between two (convex) objects
/// the closest point is on the second object (B), and the normal points from the surface on B towards A.
/// distance is between closest points on B and closest point on A. So you can calculate closest point on A
/// by taking closestPointInA = closestPointInB + m_distance * m_normalOnSurfaceB
class btDiscreteCollisionDetectorInterface
{
	struct ClosestPointInput
	{
		btTransform m_transformA;
		btTransform m_transformB;
		btScalar m_maximumDistanceSquared = BT_LARGE_FLOAT;
	};

	/*virtual*/ ~this(){};

	//
	// give either closest points (distance > 0) or penetration (distance)
	// the normal always points from B towards A
	//
	/*virtual*/ abstract void getClosestPoints(const ref ClosestPointInput input, ref btDiscreteCollisionDetectorInterfaceResult output, btIDebugDraw* debugDraw, bool swapResults = false);
};

class btStorageResult : btDiscreteCollisionDetectorInterfaceResult
{
	btVector3 m_normalOnSurfaceB;
	btVector3 m_closestPointInB;
	btScalar m_distance = BT_LARGE_FLOAT;  //negative means penetration !

/*protected:
	this()
	{
	}*/

public:
	/*virtual*/ ~this(){};

	/*virtual*/ final override void addContactPoint(const ref btVector3 normalOnBInWorld, const ref btVector3 pointInWorld, btScalar depth)
	{
		if (depth < m_distance)
		{
			m_normalOnSurfaceB = normalOnBInWorld;
			m_closestPointInB = pointInWorld;
			m_distance = depth;
		}
	}
};
