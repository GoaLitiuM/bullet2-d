module bullet2.BulletCollision.CollisionDispatch.btCollisionWorld;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btIDebugDraw;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btSerializer;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.CollisionDispatch.btCollisionConfiguration;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.CollisionDispatch.btCollisionDispatcher;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObjectWrapper;
import bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCache;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.BulletCollision.CollisionShapes.btConvexShape;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseInterface;
import bullet2.BulletCollision.NarrowPhaseCollision.btManifoldPoint;

///CollisionWorld is interface and container for the collision detection
abstract class btCollisionWorld
{
protected:
	btAlignedObjectArray!(btCollisionObject) m_collisionObjects;

	btDispatcher m_dispatcher1;

	btDispatcherInfo m_dispatchInfo;

	btBroadphaseInterface m_broadphasePairCache;

	btIDebugDraw* m_debugDrawer;

	///m_forceUpdateAllAabbs can be set to false as an optimization to only update active object AABBs
	///it is true by default, because it is error-prone (setting the position of static objects wouldn't update their AABB)
	bool m_forceUpdateAllAabbs;

	final void serializeCollisionObjects(btSerializer serializer);

	final void serializeContactManifolds(btSerializer serializer);

public:
	//this constructor doesn't own the dispatcher and paircache/broadphase
	final this(btDispatcher dispatcher, btBroadphaseInterface broadphasePairCache, btCollisionConfiguration collisionConfiguration);

	/*virtual*/ ~this();

	final void setBroadphase(btBroadphaseInterface pairCache)
	{
		m_broadphasePairCache = pairCache;
	}

	final const(btBroadphaseInterface) getBroadphase() const
	{
		return m_broadphasePairCache;
	}

	final btBroadphaseInterface getBroadphase()
	{
		return m_broadphasePairCache;
	}

	final btOverlappingPairCache getPairCache()
	{
		return m_broadphasePairCache.getOverlappingPairCache();
	}

	final btDispatcher getDispatcher()
	{
		return m_dispatcher1;
	}

	final const(btDispatcher) getDispatcher() const
	{
		return m_dispatcher1;
	}

	final void updateSingleAabb(btCollisionObject colObj);

	/*virtual*/ void updateAabbs();

	///the computeOverlappingPairs is usually already called by performDiscreteCollisionDetection (or stepSimulation)
	///it can be useful to use if you perform ray tests without collision detection/simulation
	/*virtual*/ void computeOverlappingPairs();

	/*virtual*/ void setDebugDrawer(btIDebugDraw* debugDrawer)
	{
		m_debugDrawer = debugDrawer;
	}

	/*virtual*/ btIDebugDraw* getDebugDrawer()
	{
		return m_debugDrawer;
	}

	/*virtual*/ void debugDrawWorld();

	pragma(mangle, "?debugDrawObject@btCollisionWorld@@UEAAXAEBVbtTransform@@PEBVbtCollisionShape@@AEBVbtVector3@@@Z")
	/*virtual*/ void debugDrawObject(ref const(btTransform) worldTransform, const btCollisionShape shape, ref const(btVector3) color);

	///LocalShapeInfo gives extra information for complex shapes
	///Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart
	struct LocalShapeInfo
	{
		int m_shapePart;
		int m_triangleIndex;

		//const btCollisionShape	m_shapeTemp;
		//const btTransform*	m_shapeLocalTransform;
	};

	struct LocalRayResult
	{
		this(btCollisionObject collisionObject,
					   LocalShapeInfo* localShapeInfo,
					   ref const(btVector3) hitNormalLocal,
					   btScalar hitFraction)
		{
            m_collisionObject = collisionObject;
            m_localShapeInfo = localShapeInfo;
            m_hitNormalLocal = hitNormalLocal;
            m_hitFraction = hitFraction;
		}

		/*const*/ btCollisionObject m_collisionObject;
		LocalShapeInfo* m_localShapeInfo;
		btVector3 m_hitNormalLocal;
		btScalar m_hitFraction;
	};

	///RayResultCallback is used to report new raycast results
    extern (C++, struct)
	class RayResultCallback
	{
		btScalar m_closestHitFraction = 1.0;
		/*const*/ btCollisionObject m_collisionObject = null;
		int m_collisionFilterGroup = btBroadphaseProxy.CollisionFilterGroups.DefaultFilter;
		int m_collisionFilterMask = btBroadphaseProxy.CollisionFilterGroups.AllFilter;
		//@BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see btRaycastCallback.h. Apply any of the EFlags defined there on m_flags here to invoke.
		uint m_flags = 0;

		/*virtual*/ ~this()
		{
		}
		bool hasHit() const
		{
			return (m_collisionObject !is null);
		}

		/*virtual*/ bool needsCollision(btBroadphaseProxy proxy0) const
		{
			bool collides = (proxy0.m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0.m_collisionFilterMask);
			return collides;
		}

		abstract /*virtual*/ btScalar addSingleResult(ref LocalRayResult rayResult, bool normalInWorldSpace);
	};

	extern (C++, struct)
	class ClosestRayResultCallback : RayResultCallback
	{
		this(ref const(btVector3) rayFromWorld, ref const(btVector3) rayToWorld)
		{
            m_rayFromWorld = rayFromWorld;
			m_rayToWorld = rayToWorld;
		}

		btVector3 m_rayFromWorld;  //used to calculate hitPointWorld from hitFraction
		btVector3 m_rayToWorld;

		btVector3 m_hitNormalWorld;
		btVector3 m_hitPointWorld;

		override /*virtual*/ btScalar addSingleResult(ref LocalRayResult rayResult, bool normalInWorldSpace)
		{
			//caller already does the filter on the m_closestHitFraction
			btAssert(rayResult.m_hitFraction <= m_closestHitFraction);

			m_closestHitFraction = rayResult.m_hitFraction;
			m_collisionObject = rayResult.m_collisionObject;
			if (normalInWorldSpace)
			{
				m_hitNormalWorld = rayResult.m_hitNormalLocal;
			}
			else
			{
				///need to transform normal into worldspace
				m_hitNormalWorld = m_collisionObject.getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
			}
			m_hitPointWorld.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);
			return rayResult.m_hitFraction;
		}
	};

	extern (C++, struct)
	class AllHitsRayResultCallback : RayResultCallback
	{
		this(ref const(btVector3) rayFromWorld, ref const(btVector3) rayToWorld)
		{
            m_rayFromWorld = rayFromWorld;
			m_rayToWorld = rayToWorld;
		}

		btAlignedObjectArray!(btCollisionObject) m_collisionObjects;

		btVector3 m_rayFromWorld;  //used to calculate hitPointWorld from hitFraction
		btVector3 m_rayToWorld;

		btAlignedObjectArray!btVector3 m_hitNormalWorld;
		btAlignedObjectArray!btVector3 m_hitPointWorld;
		btAlignedObjectArray!btScalar m_hitFractions;

		override /*virtual*/ btScalar addSingleResult(ref LocalRayResult rayResult, bool normalInWorldSpace)
		{
			m_collisionObject = rayResult.m_collisionObject;
			m_collisionObjects.push_back(rayResult.m_collisionObject);
			btVector3 hitNormalWorld;
			if (normalInWorldSpace)
			{
				hitNormalWorld = rayResult.m_hitNormalLocal;
			}
			else
			{
				///need to transform normal into worldspace
				hitNormalWorld = m_collisionObject.getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
			}
			m_hitNormalWorld.push_back(hitNormalWorld);
			btVector3 hitPointWorld;
			hitPointWorld.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);
			m_hitPointWorld.push_back(hitPointWorld);
			m_hitFractions.push_back(rayResult.m_hitFraction);
			return m_closestHitFraction;
		}
	};

	struct LocalConvexResult
	{
		this(/*const*/ btCollisionObject hitCollisionObject,
						  LocalShapeInfo* localShapeInfo,
						  ref const(btVector3) hitNormalLocal,
						  ref const(btVector3) hitPointLocal,
						  btScalar hitFraction)
		{
            m_hitCollisionObject = hitCollisionObject;
            m_localShapeInfo = localShapeInfo;
            m_hitNormalLocal = hitNormalLocal;
            m_hitPointLocal = hitPointLocal;
            m_hitFraction = hitFraction;
		}

		/*const*/ btCollisionObject m_hitCollisionObject;
		LocalShapeInfo* m_localShapeInfo;
		btVector3 m_hitNormalLocal;
		btVector3 m_hitPointLocal;
		btScalar m_hitFraction;
	};

	///RayResultCallback is used to report new raycast results
	extern (C++, struct)
	class ConvexResultCallback
	{
		btScalar m_closestHitFraction = 1.0;
		int m_collisionFilterGroup = btBroadphaseProxy.CollisionFilterGroups.DefaultFilter;
		int m_collisionFilterMask = btBroadphaseProxy.CollisionFilterGroups.AllFilter;

		/*virtual*/ ~this()
		{
		}

		bool hasHit() const
		{
			return (m_closestHitFraction < btScalar(1.));
		}

		/*virtual*/ bool needsCollision(btBroadphaseProxy proxy0) const
		{
			bool collides = (proxy0.m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0.m_collisionFilterMask);
			return collides;
		}

		abstract /*virtual*/ btScalar addSingleResult(ref LocalConvexResult convexResult, bool normalInWorldSpace);
	};

	extern (C++, struct)
	class ClosestConvexResultCallback : ConvexResultCallback
	{
		this(ref const(btVector3) convexFromWorld, ref const(btVector3) convexToWorld)
		{
            m_convexFromWorld = convexFromWorld;
            m_convexToWorld = convexToWorld;
            m_hitCollisionObject = null;
		}

		btVector3 m_convexFromWorld;  //used to calculate hitPointWorld from hitFraction
		btVector3 m_convexToWorld;

		btVector3 m_hitNormalWorld;
		btVector3 m_hitPointWorld;
		/*const*/ btCollisionObject m_hitCollisionObject;

		override /*virtual*/ btScalar addSingleResult(ref LocalConvexResult convexResult, bool normalInWorldSpace)
		{
			//caller already does the filter on the m_closestHitFraction
			btAssert(convexResult.m_hitFraction <= m_closestHitFraction);

			m_closestHitFraction = convexResult.m_hitFraction;
			m_hitCollisionObject = convexResult.m_hitCollisionObject;
			if (normalInWorldSpace)
			{
				m_hitNormalWorld = convexResult.m_hitNormalLocal;
			}
			else
			{
				///need to transform normal into worldspace
				m_hitNormalWorld = m_hitCollisionObject.getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
			}
			m_hitPointWorld = convexResult.m_hitPointLocal;
			return convexResult.m_hitFraction;
		}
	};

	///ContactResultCallback is used to report contact points
	extern (C++, struct)
	class ContactResultCallback
	{
		int m_collisionFilterGroup = btBroadphaseProxy.CollisionFilterGroups.DefaultFilter;
		int m_collisionFilterMask = btBroadphaseProxy.CollisionFilterGroups.AllFilter;
		btScalar m_closestDistanceThreshold = 0;

		/*virtual*/ ~this()
		{
		}

		/*virtual*/ bool needsCollision(btBroadphaseProxy proxy0) const
		{
			bool collides = (proxy0.m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0.m_collisionFilterMask);
			return collides;
		}

		abstract /*virtual*/ btScalar addSingleResult(ref btManifoldPoint cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1);
	};

	final int getNumCollisionObjects() const
	{
		return int(m_collisionObjects.size());
	}

	/// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
	/// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
	pragma(mangle, "?rayTest@btCollisionWorld@@UEBAXAEBVbtVector3@@0AEAURayResultCallback@1@@Z")
	/*virtual*/ void rayTest(ref const(btVector3) rayFromWorld, ref const(btVector3) rayToWorld, ref RayResultCallback resultCallback) const;

	/// convexTest performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
	/// This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
	final void convexSweepTest(const btConvexShape* castShape, ref const(btTransform) from, ref const(btTransform) to, ref ConvexResultCallback resultCallback, btScalar allowedCcdPenetration = btScalar(0.)) const;

	///contactTest performs a discrete collision test between colObj against all objects in the btCollisionWorld, and calls the resultCallback.
	///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
	final void contactTest(btCollisionObject colObj, ref ContactResultCallback resultCallback);

	///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
	///it reports one or more contact points (including the one with deepest penetration)
	pragma(mangle, "?contactPairTest@btCollisionWorld@@QEAAXPEAVbtCollisionObject@@0AEAUContactResultCallback@1@@Z")
	final void contactPairTest(btCollisionObject colObjA, btCollisionObject colObjB, /*ref*/ ContactResultCallback resultCallback);

	/// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
	/// In a future implementation, we consider moving the ray test as a /*virtual*/ method in btCollisionShape.
	/// This allows more customization.
	static void rayTestSingle(ref const(btTransform) rayFromTrans, ref const(btTransform) rayToTrans,
							  btCollisionObject collisionObject,
							  const btCollisionShape collisionShape,
							  ref const(btTransform) colObjWorldTransform,
							  ref RayResultCallback resultCallback);

	static void rayTestSingleInternal(ref const(btTransform) rayFromTrans, ref const(btTransform) rayToTrans,
									  const btCollisionObjectWrapper* collisionObjectWrap,
									  ref RayResultCallback resultCallback);

	/// objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
	static void objectQuerySingle(const btConvexShape* castShape, ref const(btTransform) rayFromTrans, ref const(btTransform) rayToTrans,
								  btCollisionObject collisionObject,
								  const btCollisionShape collisionShape,
								  ref const(btTransform) colObjWorldTransform,
								  ref ConvexResultCallback resultCallback, btScalar allowedPenetration);

	static void objectQuerySingleInternal(const btConvexShape* castShape, ref const(btTransform) convexFromTrans, ref const(btTransform) convexToTrans,
										  const btCollisionObjectWrapper* colObjWrap,
										  ref ConvexResultCallback resultCallback, btScalar allowedPenetration);

	/*virtual*/ void addCollisionObject(btCollisionObject collisionObject, int collisionFilterGroup = btBroadphaseProxy.CollisionFilterGroups.DefaultFilter, int collisionFilterMask = btBroadphaseProxy.CollisionFilterGroups.AllFilter);

	/*virtual*/ void refreshBroadphaseProxy(btCollisionObject collisionObject);

	final auto ref btCollisionObjectArray getCollisionObjectArray()
	{
		return m_collisionObjects;
	}

	final const(btCollisionObjectArray) getCollisionObjectArray() const
	{
		return m_collisionObjects;
	}

	/*virtual*/ void removeCollisionObject(btCollisionObject collisionObject);

	/*virtual*/ void performDiscreteCollisionDetection();

	final ref btDispatcherInfo getDispatchInfo()
	{
		return m_dispatchInfo;
	}

	final const(btDispatcherInfo) getDispatchInfo() const
	{
		return m_dispatchInfo;
	}

	final bool getForceUpdateAllAabbs() const
	{
		return m_forceUpdateAllAabbs;
	}
	final void setForceUpdateAllAabbs(bool forceUpdateAllAabbs)
	{
		m_forceUpdateAllAabbs = forceUpdateAllAabbs;
	}

	///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (Bullet/Demos/SerializeDemo)
	/*virtual*/ void serialize(btSerializer serializer);
};
