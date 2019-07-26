module bullet2.BulletCollision.BroadphaseCollision.btDispatcher;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.BulletCollision.BroadphaseCollision.btCollisionAlgorithm;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObjectWrapper;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCache;
import bullet2.LinearMath.btIDebugDraw;
import bullet2.LinearMath.btPoolAllocator;

extern (C++, struct) // bug?
struct btDispatcherInfo
{
	enum DispatchFunc : int
	{
		DISPATCH_DISCRETE = 1,
		DISPATCH_CONTINUOUS
	};

	btScalar m_timeStep = btScalar(0.);
	int m_stepCount = 0;
    DispatchFunc m_dispatchFunc = DispatchFunc.DISPATCH_DISCRETE;
    btScalar m_timeOfImpact = btScalar(1.0f);
    bool m_useContinuous = true;
    btIDebugDraw* m_debugDraw = null;
    bool m_enableSatConvex = false;
    bool m_enableSPU = true;
    bool m_useEpa = true;
    btScalar m_allowedCcdPenetration = btScalar(0.04);
    bool m_useConvexConservativeDistanceUtil = false;
    btScalar m_convexConservativeDistanceThreshold = 0.0f;
    bool m_deterministicOverlappingPairs = false;
};

enum ebtDispatcherQueryType
{
	BT_CONTACT_POINT_ALGORITHMS = 1,
	BT_CLOSEST_POINT_ALGORITHMS = 2
};

///The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for overlapping pairs.
///For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user callbacks (game logic).
abstract class btDispatcher
{
public:
	/*virtual*/ ~this();

	/*virtual*/ btCollisionAlgorithm findAlgorithm(const(btCollisionObjectWrapper)* body0Wrap, const(btCollisionObjectWrapper)* body1Wrap, btPersistentManifold sharedManifold, ebtDispatcherQueryType queryType);

	/*virtual*/ abstract btPersistentManifold getNewManifold(btCollisionObject b0, btCollisionObject b1);

	/*virtual*/ abstract void releaseManifold(btPersistentManifold manifold);

	/*virtual*/ abstract void clearManifold(btPersistentManifold manifold);

	/*virtual*/ abstract bool needsCollision(btCollisionObject body0, btCollisionObject body1);

	/*virtual*/ abstract bool needsResponse(btCollisionObject body0, btCollisionObject body1);

	/*virtual*/ abstract void dispatchAllCollisionPairs(btOverlappingPairCache pairCache, const ref btDispatcherInfo dispatchInfo, btDispatcher dispatcher);

	/*virtual*/ abstract int getNumManifolds() const;

	/*virtual*/ abstract btPersistentManifold getManifoldByIndexInternal(int index);

	/*virtual*/ abstract btPersistentManifold* getInternalManifoldPointer();

	/*virtual*/ abstract btPoolAllocator getInternalManifoldPool();

	/*virtual*/ abstract const(btPoolAllocator) getInternalManifoldPool() const;

	/*virtual*/ abstract void* allocateCollisionAlgorithm(int size);

	/*virtual*/ abstract void freeCollisionAlgorithm(void* ptr);
};
