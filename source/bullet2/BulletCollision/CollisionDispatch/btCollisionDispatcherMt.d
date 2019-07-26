module bullet2.BulletCollision.CollisionDispatch.btCollisionDispatcherMt;

extern (C++):

import bullet2.BulletCollision.CollisionDispatch.btCollisionDispatcher;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;

import bullet2.BulletCollision.CollisionDispatch.btCollisionConfiguration;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCache;

class btCollisionDispatcherMt : btCollisionDispatcher
{
public:
	this(btCollisionConfiguration config, int grainSize = 40);

    pragma(mangle, "?getNewManifold@btCollisionDispatcherMt@@UEAAPEAVbtPersistentManifold@@PEBVbtCollisionObject@@0@Z")
	override /*virtual*/ btPersistentManifold getNewManifold(const(btCollisionObject) body0, const(btCollisionObject) body1);
	override /*virtual*/ void releaseManifold(btPersistentManifold manifold);

	override /*virtual*/ void dispatchAllCollisionPairs(btOverlappingPairCache pairCache, ref const(btDispatcherInfo) info, btDispatcher dispatcher);

protected:
	bool m_batchUpdating;
	int m_grainSize;
};
