module bullet2.BulletCollision.CollisionDispatch.btCollisionDispatcher;

extern (C++):

import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletCollision.CollisionDispatch.btManifoldResult;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.BulletCollision.BroadphaseCollision.btCollisionAlgorithm;
import bullet2.BulletCollision.CollisionDispatch.btCollisionCreateFunc;
import bullet2.BulletCollision.CollisionDispatch.btCollisionConfiguration;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObjectWrapper;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCache;
import bullet2.LinearMath.btPoolAllocator;

enum USE_DISPATCH_REGISTRY_ARRAY = 1;

///user can override this nearcallback for collision filtering and more finegrained control over collision detection
alias btNearCallback = void function(ref btBroadphasePair collisionPair, ref btCollisionDispatcher dispatcher, const ref btDispatcherInfo dispatchInfo);

///btCollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
///Time of Impact, Closest Points and Penetration Depth.
class btCollisionDispatcher : /*public*/ btDispatcher
{
protected:
	int m_dispatcherFlags;

	btAlignedObjectArray!(btPersistentManifold) m_manifoldsPtr;

	btManifoldResult m_defaultManifoldResult;

	btNearCallback m_nearCallback;

	btPoolAllocator m_collisionAlgorithmPoolAllocator;

	btPoolAllocator m_persistentManifoldPoolAllocator;

    btCollisionAlgorithmCreateFunc[BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES][BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES] m_doubleDispatchContactPoints;

    btCollisionAlgorithmCreateFunc[BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES][BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES] m_doubleDispatchClosestPoints;

	btCollisionConfiguration m_collisionConfiguration;

public:
	enum DispatcherFlags
	{
		CD_STATIC_STATIC_REPORTED = 1,
		CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD = 2,
		CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION = 4
	};

	final int getDispatcherFlags() const
	{
		return m_dispatcherFlags;
	}

	final void setDispatcherFlags(int flags)
	{
		m_dispatcherFlags = flags;
	}

	///registerCollisionCreateFunc allows registration of custom/alternative collision create functions
	final void registerCollisionCreateFunc(int proxyType0, int proxyType1, btCollisionAlgorithmCreateFunc createFunc);

	final void registerClosestPointsCreateFunc(int proxyType0, int proxyType1, btCollisionAlgorithmCreateFunc createFunc);

	override int getNumManifolds() const
	{
		return int(m_manifoldsPtr.size());
	}

	override btPersistentManifold* getInternalManifoldPointer()
	{
		return m_manifoldsPtr.size() ? (&m_manifoldsPtr[0]) : null;
	}

	override btPersistentManifold getManifoldByIndexInternal(int index)
	{
		return m_manifoldsPtr[index];
	}

	final const(btPersistentManifold) getManifoldByIndexInternal(int index) const
	{
		return m_manifoldsPtr[index];
	}

	final this(btCollisionConfiguration collisionConfiguration);

	/*virtual*/ ~this();

	pragma(mangle, "?getNewManifold@btCollisionDispatcher@@UEAAPEAVbtPersistentManifold@@PEBVbtCollisionObject@@0@Z")
	/*virtual*/ override btPersistentManifold getNewManifold(const(btCollisionObject) b0, const(btCollisionObject) b1);

	/*virtual*/ override void releaseManifold(btPersistentManifold manifold);

	/*virtual*/ override void clearManifold(btPersistentManifold manifold);

	override btCollisionAlgorithm findAlgorithm(const(btCollisionObjectWrapper)* body0Wrap, const(btCollisionObjectWrapper)* body1Wrap, btPersistentManifold sharedManifold, ebtDispatcherQueryType queryType);

	pragma(mangle, "?needsCollision@btCollisionDispatcher@@UEAA_NPEBVbtCollisionObject@@0@Z")
	/*virtual*/ override bool needsCollision(const(btCollisionObject) body0, const(btCollisionObject) body1);

	pragma(mangle, "?needsResponse@btCollisionDispatcher@@UEAA_NPEBVbtCollisionObject@@0@Z")
	/*virtual*/ override bool needsResponse(const(btCollisionObject) body0, const(btCollisionObject) body1);

	/*virtual*/ override void dispatchAllCollisionPairs(btOverlappingPairCache pairCache, const ref btDispatcherInfo dispatchInfo, btDispatcher dispatcher);

	final void setNearCallback(btNearCallback nearCallback)
	{
		m_nearCallback = nearCallback;
	}

	final btNearCallback getNearCallback() const
	{
		return m_nearCallback;
	}

	//by default, Bullet will use this near callback
	final static void defaultNearCallback(ref btBroadphasePair collisionPair, ref btCollisionDispatcher dispatcher, const ref btDispatcherInfo dispatchInfo);

	/*virtual*/ override void* allocateCollisionAlgorithm(int size);

	/*virtual*/ override void freeCollisionAlgorithm(void* ptr);

	final btCollisionConfiguration getCollisionConfiguration()
	{
		return m_collisionConfiguration;
	}

	final const(btCollisionConfiguration) getCollisionConfiguration() //const
	{
		return m_collisionConfiguration;
	}

	final void setCollisionConfiguration(btCollisionConfiguration config)
	{
		m_collisionConfiguration = config;
	}

	/*virtual*/ override btPoolAllocator getInternalManifoldPool()
	{
		return m_persistentManifoldPoolAllocator;
	}

	/*virtual*/ override const(btPoolAllocator) getInternalManifoldPool() const
	{
		return m_persistentManifoldPoolAllocator;
	}
};
