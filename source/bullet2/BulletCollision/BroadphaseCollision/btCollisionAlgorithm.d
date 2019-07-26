module bullet2.BulletCollision.BroadphaseCollision.btCollisionAlgorithm;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObjectWrapper;
import bullet2.BulletCollision.CollisionDispatch.btManifoldResult;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;

alias btManifoldArray = btAlignedObjectArray!(btPersistentManifold);

struct btCollisionAlgorithmConstructionInfo
{

	this(btDispatcher dispatcher, int temp)
	{
		m_dispatcher1 = dispatcher;
	}

	btDispatcher m_dispatcher1 = null;
	btPersistentManifold m_manifold = null;

	//	int	getDispatcherId();
};

///btCollisionAlgorithm is an collision interface that is compatible with the Broadphase and btDispatcher.
///It is persistent over frames
abstract class btCollisionAlgorithm
{
protected:
	btDispatcher m_dispatcher;

protected:
	//	int	getDispatcherId();

public:
	this(){};

	this(const ref btCollisionAlgorithmConstructionInfo ci);

	/*virtual*/ ~this(){};

	/*virtual*/ abstract void processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, ref const(btDispatcherInfo) dispatchInfo, btManifoldResult* resultOut);

	/*virtual*/ abstract btScalar calculateTimeOfImpact(btCollisionObject body0, btCollisionObject body1, ref const(btDispatcherInfo) dispatchInfo, btManifoldResult* resultOut);

	/*virtual*/ abstract void getAllContactManifolds(ref btManifoldArray manifoldArray);
};
