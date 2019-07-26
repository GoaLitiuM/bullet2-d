module bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCache;

extern (C++):

import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseInterface;
import bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCallback;

import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btScalar;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;

import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;

alias btBroadphasePairArray = btAlignedObjectArray!btBroadphasePair;

extern (C++, struct)
class btOverlapCallback
{
	/*virtual*/ ~this()
	{
	}
	//return true for deletion of the pair
	/*virtual*/ abstract bool processOverlap(ref btBroadphasePair pair);
};

extern (C++, struct)
class btOverlapFilterCallback
{
	/*virtual*/ ~this()
	{
	}
	// return true when pairs need collision
	/*virtual*/ abstract bool needBroadphaseCollision(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) const;
};

enum int BT_NULL_PAIR = 0xffffffff;

///The btOverlappingPairCache provides an interface for overlapping pair management (add, remove, storage), used by the btBroadphaseInterface broadphases.
///The btHashedOverlappingPairCache and btSortedOverlappingPairCache classes are two implementations.
extern (C++, class)
class btOverlappingPairCache : btOverlappingPairCallback
{
public:
	/*virtual*/ ~this() {}  // this is needed so we can get to the derived class destructor

	/*virtual*/ abstract btBroadphasePair* getOverlappingPairArrayPtr();

	/*virtual*/ abstract const(btBroadphasePair)* getOverlappingPairArrayPtr(); //const;

	/*virtual*/ abstract ref btBroadphasePairArray getOverlappingPairArray();

	/*virtual*/ abstract void cleanOverlappingPair(ref btBroadphasePair pair, btDispatcher dispatcher);

	/*virtual*/ abstract int getNumOverlappingPairs() const;

	/*virtual*/ abstract void cleanProxyFromPairs(btBroadphaseProxy proxy, btDispatcher dispatcher);

	/*virtual*/ abstract void setOverlapFilterCallback(btOverlapFilterCallback callback);

	/*virtual*/ abstract void processAllOverlappingPairs(btOverlapCallback, btDispatcher dispatcher);

	/*virtual*/ void processAllOverlappingPairs(btOverlapCallback callback, btDispatcher dispatcher, ref const btDispatcherInfo dispatchInfo)
	{
		processAllOverlappingPairs(callback, dispatcher);
	}
	/*virtual*/ abstract btBroadphasePair* findPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1);

	/*virtual*/ abstract bool hasDeferredRemoval();

	/*virtual*/ abstract void setInternalGhostPairCallback(btOverlappingPairCallback ghostPairCallback);

	/*virtual*/ abstract void sortOverlappingPairs(btDispatcher dispatcher);
};

/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com

//ATTRIBUTE_ALIGNED16(class)
extern (C++, class)
align(16) class btHashedOverlappingPairCache : btOverlappingPairCache
{
	btBroadphasePairArray m_overlappingPairArray;
	btOverlapFilterCallback m_overlapFilterCallback;

protected:
	btAlignedObjectArray!int m_hashTable;
	btAlignedObjectArray!int m_next;
	btOverlappingPairCallback m_ghostPairCallback;

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this();
	/*virtual*/ ~this();

	final override void removeOverlappingPairsContainingProxy(btBroadphaseProxy proxy, btDispatcher dispatcher);

	/*virtual*/ override void* removeOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher dispatcher);

	final/*SIMD_FORCE_INLINE*/ bool needsBroadphaseCollision(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) const
	{
		if (m_overlapFilterCallback)
			return m_overlapFilterCallback.needBroadphaseCollision(proxy0, proxy1);

		bool collides = (proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask) != 0;
		collides = collides && (proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask);

		return collides;
	}

	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	/*virtual*/ override btBroadphasePair* addOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1)
	{
		if (!needsBroadphaseCollision(proxy0, proxy1))
			return null;

		return internalAddPair(proxy0, proxy1);
	}

	final override void cleanProxyFromPairs(btBroadphaseProxy proxy, btDispatcher dispatcher);

	/*virtual*/ override void processAllOverlappingPairs(btOverlapCallback, btDispatcher dispatcher);

	/*virtual*/ override void processAllOverlappingPairs(btOverlapCallback callback, btDispatcher dispatcher, ref const btDispatcherInfo dispatchInfo);

	/*virtual*/ override btBroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}

	final override const(btBroadphasePair)* getOverlappingPairArrayPtr() //const
	{
		return &m_overlappingPairArray[0];
	}

	final override ref btBroadphasePairArray getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	/+const ref btBroadphasePairArray getOverlappingPairArray() //const
	{
		return m_overlappingPairArray;
	}+/

	final override void cleanOverlappingPair(ref btBroadphasePair pair, btDispatcher dispatcher);

	final override btBroadphasePair* findPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1);

	final int GetCount() const { return m_overlappingPairArray.size(); }
	//	btBroadphasePair* GetPairs() { return m_pairs; }

	final btOverlapFilterCallback getOverlapFilterCallback()
	{
		return m_overlapFilterCallback;
	}

	final override void setOverlapFilterCallback(btOverlapFilterCallback callback)
	{
		m_overlapFilterCallback = callback;
	}

	final override int getNumOverlappingPairs() const
	{
		return m_overlappingPairArray.size();
	}

private:
	pragma(mangle, "?internalAddPair@btHashedOverlappingPairCache@@AEAAPEAUbtBroadphasePair@@PEAUbtBroadphaseProxy@@0@Z") // bug?
	final btBroadphasePair* internalAddPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1);

	final void growTables();

	final /*SIMD_FORCE_INLINE*/ bool equalsPair(ref btBroadphasePair pair, int proxyId1, int proxyId2)
	{
		return pair.m_pProxy0.getUid() == proxyId1 && pair.m_pProxy1.getUid() == proxyId2;
	}

	/+
	// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
	// This assumes proxyId1 and proxyId2 are 16-bit.
	/*SIMD_FORCE_INLINE*/ int getHash(int proxyId1, int proxyId2)
	{
		int key = (proxyId2 << 16) | proxyId1;
		key = ~key + (key << 15);
		key = key ^ (key >> 12);
		key = key + (key << 2);
		key = key ^ (key >> 4);
		key = key * 2057;
		key = key ^ (key >> 16);
		return key;
	}
	+/

	final /*SIMD_FORCE_INLINE*/ uint getHash(uint proxyId1, uint proxyId2)
	{
		uint key = proxyId1 | (proxyId2 << 16);
		// Thomas Wang's hash

		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);
		return key;
	}

	final /*SIMD_FORCE_INLINE*/ btBroadphasePair* internalFindPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, int hash)
	{
		int proxyId1 = proxy0.getUid();
		int proxyId2 = proxy1.getUid();
/*#if 0  // wrong, 'equalsPair' use unsorted uids, copy-past devil striked again. Nat.
		if (proxyId1 > proxyId2)
			btSwap(proxyId1, proxyId2);
#endif*/

		int index = m_hashTable[hash];

		while (index != BT_NULL_PAIR && equalsPair(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
		{
			index = m_next[index];
		}

		if (index == BT_NULL_PAIR)
		{
			return null;
		}

		btAssert(index < m_overlappingPairArray.size());

		return &m_overlappingPairArray[index];
	}

	/*virtual*/ bool hasDeferredRemoval()
	{
		return false;
	}

	/*virtual*/ void setInternalGhostPairCallback(btOverlappingPairCallback ghostPairCallback)
	{
		m_ghostPairCallback = ghostPairCallback;
	}

	/*virtual*/ void sortOverlappingPairs(btDispatcher dispatcher);
};

///btSortedOverlappingPairCache maintains the objects with overlapping AABB
///Typically managed by the Broadphase, Axis3Sweep or btSimpleBroadphase
extern (C++, class)
class btSortedOverlappingPairCache : btOverlappingPairCache
{
protected:
	//avoid brute-force finding all the time
	btBroadphasePairArray m_overlappingPairArray;

	//during the dispatch, check that user doesn't destroy/create proxy
	bool m_blockedForChanges;

	///by default, do the removal during the pair traversal
	bool m_hasDeferredRemoval;

	//if set, use the callback instead of the built in filter in needBroadphaseCollision
	btOverlapFilterCallback m_overlapFilterCallback;

	btOverlappingPairCallback m_ghostPairCallback;

public:
	this();
	/*virtual*/ ~this();

	/*virtual*/ override void processAllOverlappingPairs(btOverlapCallback, btDispatcher dispatcher);

	override void* removeOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher dispatcher);

	pragma(mangle, "?cleanOverlappingPair@btSortedOverlappingPairCache@@UEAAXAEAUbtBroadphasePair@@PEAVbtDispatcher@@@Z") // bug?
	override void cleanOverlappingPair(ref btBroadphasePair pair, btDispatcher dispatcher);

	pragma(mangle, "?addOverlappingPair@btSortedOverlappingPairCache@@UEAAPEAUbtBroadphasePair@@PEAUbtBroadphaseProxy@@0@Z") // bug?
	override btBroadphasePair* addOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1);

	pragma(mangle, "?findPair@btSortedOverlappingPairCache@@UEAAPEAUbtBroadphasePair@@PEAUbtBroadphaseProxy@@0@Z") // bug?
	override btBroadphasePair* findPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1);

	override void cleanProxyFromPairs(btBroadphaseProxy proxy, btDispatcher dispatcher);

	override void removeOverlappingPairsContainingProxy(btBroadphaseProxy proxy, btDispatcher dispatcher);

	final /*inline*/ bool needsBroadphaseCollision(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1) const
	{
		if (m_overlapFilterCallback)
			return m_overlapFilterCallback.needBroadphaseCollision(proxy0, proxy1);

		bool collides = (proxy0.m_collisionFilterGroup & proxy1.m_collisionFilterMask) != 0;
		collides = collides && (proxy1.m_collisionFilterGroup & proxy0.m_collisionFilterMask);

		return collides;
	}

	override ref btBroadphasePairArray getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	/+const ref btBroadphasePairArray getOverlappingPairArray() //const
	{
		return m_overlappingPairArray;
	}+/

	final override btBroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}

	final override const(btBroadphasePair)* getOverlappingPairArrayPtr() //const
	{
		return &m_overlappingPairArray[0];
	}

	final override int getNumOverlappingPairs() const
	{
		return m_overlappingPairArray.size();
	}

	btOverlapFilterCallback getOverlapFilterCallback()
	{
		return m_overlapFilterCallback;
	}

	final override void setOverlapFilterCallback(btOverlapFilterCallback callback)
	{
		m_overlapFilterCallback = callback;
	}

	override /*virtual*/ bool hasDeferredRemoval()
	{
		return m_hasDeferredRemoval;
	}

	override /*virtual*/ void setInternalGhostPairCallback(btOverlappingPairCallback ghostPairCallback)
	{
		m_ghostPairCallback = ghostPairCallback;
	}

	override /*virtual*/ void sortOverlappingPairs(btDispatcher dispatcher);
};

///btNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.
extern (C++, class)
class btNullPairCache : btOverlappingPairCache
{
	btBroadphasePairArray m_overlappingPairArray;

public:
	override /*virtual*/ btBroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}
	final override const(btBroadphasePair)* getOverlappingPairArrayPtr() //const
	{
		return &m_overlappingPairArray[0];
	}
	final override ref btBroadphasePairArray getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	override /*virtual*/ void cleanOverlappingPair(ref btBroadphasePair /*pair*/, btDispatcher /*dispatcher*/)
	{
	}

	override /*virtual*/ int getNumOverlappingPairs() const
	{
		return 0;
	}

	override /*virtual*/ void cleanProxyFromPairs(btBroadphaseProxy /*proxy*/, btDispatcher /*dispatcher*/)
	{
	}

	override /*virtual*/ void setOverlapFilterCallback(btOverlapFilterCallback /*callback*/)
	{
	}

	override /*virtual*/ void processAllOverlappingPairs(btOverlapCallback, btDispatcher /*dispatcher*/)
	{
	}

	override /*virtual*/ btBroadphasePair* findPair(btBroadphaseProxy /*proxy0*/, btBroadphaseProxy /*proxy1*/)
	{
		return null;
	}

	override /*virtual*/ bool hasDeferredRemoval()
	{
		return true;
	}

	override /*virtual*/ void setInternalGhostPairCallback(btOverlappingPairCallback /* ghostPairCallback */)
	{
	}

	override /*virtual*/ btBroadphasePair* addOverlappingPair(btBroadphaseProxy /*proxy0*/, btBroadphaseProxy /*proxy1*/)
	{
		return null;
	}

	override /*virtual*/ void* removeOverlappingPair(btBroadphaseProxy /*proxy0*/, btBroadphaseProxy /*proxy1*/, btDispatcher /*dispatcher*/)
	{
		return null;
	}

	override /*virtual*/ void removeOverlappingPairsContainingProxy(btBroadphaseProxy /*proxy0*/, btDispatcher /*dispatcher*/)
	{
	}

	override /*virtual*/ void sortOverlappingPairs(btDispatcher dispatcher)
	{
		//(void)dispatcher;
	}
};
