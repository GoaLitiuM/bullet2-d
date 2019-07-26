module bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.BulletCollision.NarrowPhaseCollision.btManifoldPoint;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.LinearMath.btAlignedAllocator;
import bullet2.LinearMath.btSerializer;


///maximum contact breaking and merging threshold
extern btScalar gContactBreakingThreshold;

alias ContactDestroyedCallback = bool function(void* userPersistentData);
alias ContactProcessedCallback = bool function(ref btManifoldPoint cp, void* body0, void* body1);
alias ContactStartedCallback = void function(ref btPersistentManifold manifold);
alias ContactEndedCallback = void function(ref btPersistentManifold manifold);
extern ContactDestroyedCallback gContactDestroyedCallback;
extern ContactProcessedCallback gContactProcessedCallback;

pragma(mangle, "?gContactStartedCallback@@3P6AXAEBQEAVbtPersistentManifold@@@ZEA")
extern ContactStartedCallback gContactStartedCallback;
pragma(mangle, "?gContactEndedCallback@@3P6AXAEBQEAVbtPersistentManifold@@@ZEA")
extern ContactEndedCallback gContactEndedCallback;

//the enum starts at 1024 to avoid type conflicts with btTypedConstraint
enum btContactManifoldTypes
{
	MIN_CONTACT_MANIFOLD_TYPE = 1024,
	BT_PERSISTENT_MANIFOLD_TYPE
};

enum MANIFOLD_CACHE_SIZE = 4;

///btPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
///Those contact points are created by the collision narrow phase.
///The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
///updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
///reduces the cache to 4 points, when more then 4 points are added, using following rules:
///the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
///note that some pairs of objects might have more then one contact manifold.

//ATTRIBUTE_ALIGNED128( class) btPersistentManifold : public btTypedObject
//ATTRIBUTE_ALIGNED16(class)
//align(16) class btPersistentManifold : /*public*/ btTypedObject
extern (C++, class)
align(16) class btPersistentManifold : btTypedObject
{
private:
	btManifoldPoint[MANIFOLD_CACHE_SIZE] m_pointCache;

	/// this two body pointers can point to the physics rigidbody class.
	/*const*/btCollisionObject m_body0 = null;
	/*const*/btCollisionObject m_body1 = null;

	int m_cachedPoints = 0;

	btScalar m_contactBreakingThreshold;
	btScalar m_contactProcessingThreshold;

	/// sort cached points so most isolated points come first
	int sortCachedPoints(const ref btManifoldPoint pt);

	int findContactPoint(const btManifoldPoint* unUsed, int numUnused, const ref btManifoldPoint pt);

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	int m_companionIdA = 0;
	int m_companionIdB = 0;

	int m_index1a = 0;

	final this(btCollisionObject body0, btCollisionObject body1, int, btScalar contactBreakingThreshold, btScalar contactProcessingThreshold)
		/*: btTypedObject(BT_PERSISTENT_MANIFOLD_TYPE),
		  m_body0(body0),
		  m_body1(body1),
		  m_cachedPoints(0),
		  m_contactBreakingThreshold(contactBreakingThreshold),
		  m_contactProcessingThreshold(contactProcessingThreshold),
		  m_companionIdA(0),
		  m_companionIdB(0),
		  m_index1a(0)*/
	{
		super(btContactManifoldTypes.BT_PERSISTENT_MANIFOLD_TYPE);

        m_body0 = body0;
        m_body1 = body1;
        m_cachedPoints = 0;
        m_contactBreakingThreshold = contactBreakingThreshold;
        m_contactProcessingThreshold = contactProcessingThreshold;
        m_companionIdA = 0;
        m_companionIdB = 0;
        m_index1a = 0;
	}

	final /*SIMD_FORCE_INLINE*/ btCollisionObject getBody0() { return m_body0; }
	final /*SIMD_FORCE_INLINE*/ btCollisionObject getBody1() { return m_body1; }

	final void setBodies(btCollisionObject body0, btCollisionObject body1)
	{
		m_body0 = body0;
		m_body1 = body1;
	}

	final void clearUserCache(ref btManifoldPoint pt);

/*#ifdef DEBUG_PERSISTENCY
	void DebugPersistency();
#endif  //*/

	final /*SIMD_FORCE_INLINE*/ int getNumContacts() const
	{
		return m_cachedPoints;
	}
	/// the setNumContacts API is usually not used, except when you gather/fill all contacts manually
	final void setNumContacts(int cachedPoints)
	{
		m_cachedPoints = cachedPoints;
	}

	final /*SIMD_FORCE_INLINE*/ ref const(btManifoldPoint) getContactPoint(int index) const
	{
		btAssert(index < m_cachedPoints);
		return m_pointCache[index];
	}

	final /*SIMD_FORCE_INLINE*/ ref btManifoldPoint getContactPoint(int index)
	{
		btAssert(index < m_cachedPoints);
		return m_pointCache[index];
	}

	///@todo: get this margin from the current physics / collision environment
	final btScalar getContactBreakingThreshold() const;

	final btScalar getContactProcessingThreshold() const
	{
		return m_contactProcessingThreshold;
	}

	final void setContactBreakingThreshold(btScalar contactBreakingThreshold)
	{
		m_contactBreakingThreshold = contactBreakingThreshold;
	}

	final void setContactProcessingThreshold(btScalar contactProcessingThreshold)
	{
		m_contactProcessingThreshold = contactProcessingThreshold;
	}

	final int getCacheEntry(const ref btManifoldPoint newPoint) const;

	final int addManifoldPoint(const ref btManifoldPoint newPoint, bool isPredictive = false);

	final void removeContactPoint(int index)
	{
		clearUserCache(m_pointCache[index]);

		int lastUsedIndex = getNumContacts() - 1;
		//		m_pointCache[index] = m_pointCache[lastUsedIndex];
		if (index != lastUsedIndex)
		{
			m_pointCache[index] = m_pointCache[lastUsedIndex];
			//get rid of duplicated userPersistentData pointer
			m_pointCache[lastUsedIndex].m_userPersistentData = null;
			m_pointCache[lastUsedIndex].m_appliedImpulse = 0.0f;
			m_pointCache[lastUsedIndex].m_contactPointFlags = 0;
			m_pointCache[lastUsedIndex].m_appliedImpulseLateral1 = 0.0f;
			m_pointCache[lastUsedIndex].m_appliedImpulseLateral2 = 0.0f;
			m_pointCache[lastUsedIndex].m_lifeTime = 0;
		}

		btAssert(m_pointCache[lastUsedIndex].m_userPersistentData == null);
		m_cachedPoints--;

		if (gContactEndedCallback && m_cachedPoints == 0)
		{
			btPersistentManifold this_ = this;
			gContactEndedCallback(this_);
		}
	}
	final void replaceContactPoint(btManifoldPoint newPoint, int insertIndex)
	{
		btAssert(validContactDistance(newPoint));

/*#define MAINTAIN_PERSISTENCY 1
#ifdef MAINTAIN_PERSISTENCY*/
		int lifeTime = m_pointCache[insertIndex].getLifeTime();
		btScalar appliedImpulse = m_pointCache[insertIndex].m_appliedImpulse;
		btScalar appliedLateralImpulse1 = m_pointCache[insertIndex].m_appliedImpulseLateral1;
		btScalar appliedLateralImpulse2 = m_pointCache[insertIndex].m_appliedImpulseLateral2;

		bool replacePoint = true;
		///we keep existing contact points for friction anchors
		///if the friction force is within the Coulomb friction cone
		if (newPoint.m_contactPointFlags & btContactPointFlags.BT_CONTACT_FLAG_FRICTION_ANCHOR)
		{
			//   printf("appliedImpulse=%f\n", appliedImpulse);
			//   printf("appliedLateralImpulse1=%f\n", appliedLateralImpulse1);
			//   printf("appliedLateralImpulse2=%f\n", appliedLateralImpulse2);
			//   printf("mu = %f\n", m_pointCache[insertIndex].m_combinedFriction);
			btScalar mu = m_pointCache[insertIndex].m_combinedFriction;
			btScalar eps = 0;  //we could allow to enlarge or shrink the tolerance to check against the friction cone a bit, say 1e-7
			btScalar a = appliedLateralImpulse1 * appliedLateralImpulse1 + appliedLateralImpulse2 * appliedLateralImpulse2;
			btScalar b = eps + mu * appliedImpulse;
			b = b * b;
			replacePoint = (a) > (b);
		}

		if (replacePoint)
		{
			btAssert(lifeTime >= 0);
			void* cache = m_pointCache[insertIndex].m_userPersistentData;

			m_pointCache[insertIndex] = newPoint;
			m_pointCache[insertIndex].m_userPersistentData = cache;
			m_pointCache[insertIndex].m_appliedImpulse = appliedImpulse;
			m_pointCache[insertIndex].m_appliedImpulseLateral1 = appliedLateralImpulse1;
			m_pointCache[insertIndex].m_appliedImpulseLateral2 = appliedLateralImpulse2;
		}

		m_pointCache[insertIndex].m_lifeTime = lifeTime;
/*#else
		clearUserCache(m_pointCache[insertIndex]);
		m_pointCache[insertIndex] = newPoint;

#endif*/
	}

	final bool validContactDistance(const ref btManifoldPoint pt) const
	{
		return pt.m_distance1 <= getContactBreakingThreshold();
	}
	/// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
	final void refreshContactPoints(ref const(btTransform) trA, ref const(btTransform) trB);

	final /*SIMD_FORCE_INLINE*/ void clearManifold()
	{
		int i;
		for (i = 0; i < m_cachedPoints; i++)
		{
			clearUserCache(m_pointCache[i]);
		}

		if (gContactEndedCallback && m_cachedPoints)
		{
			btPersistentManifold this_ = this;
			gContactEndedCallback(this_);
		}
		m_cachedPoints = 0;
	}

	final int calculateSerializeBufferSize() const;
	final const(char*) serialize(const btPersistentManifold manifold, void* dataBuffer, btSerializer serializer); //const;
	final void deSerialize(const btPersistentManifoldDoubleData* manifoldDataPtr);
	final void deSerialize(const btPersistentManifoldFloatData* manifoldDataPtr);
};

// clang-format off

struct btPersistentManifoldDoubleData
{
	btVector3DoubleData[4] m_pointCacheLocalPointA;
	btVector3DoubleData[4] m_pointCacheLocalPointB;
	btVector3DoubleData[4] m_pointCachePositionWorldOnA;
	btVector3DoubleData[4] m_pointCachePositionWorldOnB;
	btVector3DoubleData[4] m_pointCacheNormalWorldOnB;
	btVector3DoubleData[4]	m_pointCacheLateralFrictionDir1;
	btVector3DoubleData[4]	m_pointCacheLateralFrictionDir2;
	double[4] m_pointCacheDistance;
	double[4] m_pointCacheAppliedImpulse;
	double[4] m_pointCacheCombinedFriction;
	double[4] m_pointCacheCombinedRollingFriction;
	double[4] m_pointCacheCombinedSpinningFriction;
	double[4] m_pointCacheCombinedRestitution;
	int[4]	m_pointCachePartId0;
	int[4]	m_pointCachePartId1;
	int[4]	m_pointCacheIndex0;
	int[4]	m_pointCacheIndex1;
	int[4] m_pointCacheContactPointFlags;
	double[4] m_pointCacheAppliedImpulseLateral1;
	double[4] m_pointCacheAppliedImpulseLateral2;
	double[4] m_pointCacheContactMotion1;
	double[4] m_pointCacheContactMotion2;
	double[4] m_pointCacheContactCFM;
	double[4] m_pointCacheCombinedContactStiffness1;
	double[4] m_pointCacheContactERP;
	double[4] m_pointCacheCombinedContactDamping1;
	double[4] m_pointCacheFrictionCFM;
	int[4] m_pointCacheLifeTime;

	int m_numCachedPoints;
	int m_companionIdA;
	int m_companionIdB;
	int m_index1a;

	int m_objectType;
	double	m_contactBreakingThreshold;
	double	m_contactProcessingThreshold;
	int m_padding;

	btCollisionObjectDoubleData *m_body0;
	btCollisionObjectDoubleData *m_body1;
};


struct btPersistentManifoldFloatData
{
	btVector3FloatData[4] m_pointCacheLocalPointA;
	btVector3FloatData[4] m_pointCacheLocalPointB;
	btVector3FloatData[4] m_pointCachePositionWorldOnA;
	btVector3FloatData[4] m_pointCachePositionWorldOnB;
	btVector3FloatData[4] m_pointCacheNormalWorldOnB;
	btVector3FloatData[4]	m_pointCacheLateralFrictionDir1;
	btVector3FloatData[4]	m_pointCacheLateralFrictionDir2;
	float[4] m_pointCacheDistance;
	float[4] m_pointCacheAppliedImpulse;
	float[4] m_pointCacheCombinedFriction;
	float[4] m_pointCacheCombinedRollingFriction;
	float[4] m_pointCacheCombinedSpinningFriction;
	float[4] m_pointCacheCombinedRestitution;
	int[4]	m_pointCachePartId0;
	int[4]	m_pointCachePartId1;
	int[4]	m_pointCacheIndex0;
	int[4]	m_pointCacheIndex1;
	int[4] m_pointCacheContactPointFlags;
	float[4] m_pointCacheAppliedImpulseLateral1;
	float[4] m_pointCacheAppliedImpulseLateral2;
	float[4] m_pointCacheContactMotion1;
	float[4] m_pointCacheContactMotion2;
	float[4] m_pointCacheContactCFM;
	float[4] m_pointCacheCombinedContactStiffness1;
	float[4] m_pointCacheContactERP;
	float[4] m_pointCacheCombinedContactDamping1;
	float[4] m_pointCacheFrictionCFM;
	int[4] m_pointCacheLifeTime;

	int m_numCachedPoints;
	int m_companionIdA;
	int m_companionIdB;
	int m_index1a;

	int m_objectType;
	float	m_contactBreakingThreshold;
	float	m_contactProcessingThreshold;
	int m_padding;

	btCollisionObjectFloatData *m_body0;
	btCollisionObjectFloatData *m_body1;
};

// clang-format on

version (BT_USE_DOUBLE_PRECISION)
{
    alias btPersistentManifoldData = btPersistentManifoldDoubleData;
    enum btPersistentManifoldDataName = "btPersistentManifoldDoubleData";
}
else
{
    alias btPersistentManifoldData = btPersistentManifoldFloatData;
    enum btPersistentManifoldDataName = "btPersistentManifoldFloatData";
}
