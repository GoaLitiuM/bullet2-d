module bullet2.BulletCollision.BroadphaseCollision.btDbvtBroadphase;

extern (C++):

import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseInterface;
import bullet2.BulletCollision.BroadphaseCollision.btDbvt;
import bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCache;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;

//
// Compile time config
//
/+
#define DBVT_BP_PROFILE 0
//#define DBVT_BP_SORTPAIRS				1
#define DBVT_BP_PREVENTFALSEUPDATE 0
#define DBVT_BP_ACCURATESLEEPING 0
#define DBVT_BP_ENABLE_BENCHMARK 0
//#define DBVT_BP_MARGIN					(btScalar)0.05
extern btScalar gDbvtMargin;

#if DBVT_BP_PROFILE
#define DBVT_BP_PROFILING_RATE 256
#include "LinearMath/btQuickprof.h"
#endif
+/

//
// btDbvtProxy
//

extern (C++, struct)
class btDbvtProxy : btBroadphaseProxy
{
    //btBroadphaseProxy base_;
	//alias base_ this;
	/* Fields		*/
	//btDbvtAabbMm	aabb;
	btDbvtNode* leaf;
	btDbvtProxy*[2] links;
	int stage;
	/* ctor			*/
	this(inout btVector3 aabbMin, inout btVector3 aabbMax, void* userPtr, int collisionFilterGroup, int collisionFilterMask)
	{
        super(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask);
        //base_ = btBroadphaseProxy(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask);
		links[0] = links[1] = null;
	}
};

alias btDbvtProxyArray = btAlignedObjectArray!(btDbvtProxy*);

///The btDbvtBroadphase implements a broadphase using two dynamic AABB bounding volume hierarchies/trees (see btDbvt).
///One tree is used for static/non-moving objects, and another tree is used for dynamic objects. Objects can move from one tree to the other.
///This is a very fast broadphase, especially for very dynamic worlds where many objects are moving. Its insert/add and remove of objects is generally faster than the sweep and prune broadphases btAxisSweep3 and bt32BitAxisSweep3.
extern (C++, struct)
class btDbvtBroadphase : btBroadphaseInterface
{
	/* Config		*/
	enum
	{
		DYNAMIC_SET = 0, /* Dynamic set index	*/
		FIXED_SET = 1,   /* Fixed set index		*/
		STAGECOUNT = 2   /* Number of stages		*/
	};
	/* Fields		*/
	btDbvt[2] m_sets;                           // Dbvt sets
	btDbvtProxy*[STAGECOUNT + 1] m_stageRoots;  // Stages list
	btOverlappingPairCache* m_paircache;        // Pair cache
	btScalar m_prediction;                      // Velocity prediction
	int m_stageCurrent;                         // Current stage
	int m_fupdates;                             // % of fixed updates per frame
	int m_dupdates;                             // % of dynamic updates per frame
	int m_cupdates;                             // % of cleanup updates per frame
	int m_newpairs;                             // Number of pairs created
	int m_fixedleft;                            // Fixed optimization left
	uint m_updates_call;                    // Number of updates call
	uint m_updates_done;                    // Number of updates done
	btScalar m_updates_ratio;                   // m_updates_done/m_updates_call
	int m_pid;                                  // Parse id
	int m_cid;                                  // Cleanup index
	int m_gid;                                  // Gen id
	bool m_releasepaircache;                    // Release pair cache on delete
	bool m_deferedcollide;                      // Defere dynamic/static collision to collide call
	bool m_needcleanup;                         // Need to run cleanup?
	btAlignedObjectArray!(btAlignedObjectArray!(const(btDbvtNode)*)) m_rayTestStacks;
/+#if DBVT_BP_PROFILE
	btClock m_clock;
	struct
	{
		unsigned long m_total;
		unsigned long m_ddcollide;
		unsigned long m_fdcollide;
		unsigned long m_cleanup;
		unsigned long m_jobcount;
	} m_profiling;
#endif+/
	/* Methods		*/
    /*this()
    {
        this(null);
    }*/
	final this(btOverlappingPairCache paircache = null);
	~this();
	final void collide(btDispatcher dispatcher);
	final void optimize();

	/* btBroadphaseInterface Implementation	*/
	final override btBroadphaseProxy createProxy(const ref btVector3 aabbMin, const ref btVector3 aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher dispatcher);
	override /*virtual*/ void destroyProxy(btBroadphaseProxy proxy, btDispatcher dispatcher);
	override /*virtual*/ void setAabb(btBroadphaseProxy proxy, const ref btVector3 aabbMin, const ref btVector3 aabbMax, btDispatcher dispatcher);
	override /*virtual*/ void rayTest(const ref btVector3 rayFrom, const ref btVector3 rayTo, ref btBroadphaseRayCallback rayCallback, const ref btVector3 aabbMin/* = btVector3(0, 0, 0)*/, const ref btVector3 aabbMax/* = btVector3(0, 0, 0)*/);
	override /*virtual*/ void aabbTest(const ref btVector3 aabbMin, const ref btVector3 aabbMax, ref btBroadphaseAabbCallback callback);

	override /*virtual*/ void getAabb(btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax) const;
	override /*virtual*/ void calculateOverlappingPairs(btDispatcher dispatcher);
	override /*virtual*/ btOverlappingPairCache getOverlappingPairCache();

	pragma(mangle, "?getOverlappingPairCache@btDbvtBroadphase@@UEBAPEBVbtOverlappingPairCache@@XZ")
	override /*virtual*/ const(btOverlappingPairCache) getOverlappingPairCache() const;
	override /*virtual*/ void getBroadphaseAabb(ref btVector3 aabbMin, ref btVector3 aabbMax) const;
	override /*virtual*/ void printStats();

	///reset broadphase internal structures, to ensure determinism/reproducability
	override /*virtual*/ void resetPool(btDispatcher dispatcher);

	final void performDeferredRemoval(btDispatcher dispatcher);

	final void setVelocityPrediction(btScalar prediction)
	{
		m_prediction = prediction;
	}
	final btScalar getVelocityPrediction() const
	{
		return m_prediction;
	}

	///this setAabbForceUpdate is similar to setAabb but always forces the aabb update.
	///it is not part of the btBroadphaseInterface but specific to btDbvtBroadphase.
	///it bypasses certain optimizations that prevent aabb updates (when the aabb shrinks), see
	///http://code.google.com/p/bullet/issues/detail?id=223
	final void setAabbForceUpdate(btBroadphaseProxy absproxy, const ref btVector3 aabbMin, const ref btVector3 aabbMax, btDispatcher /*dispatcher*/);

	final static void benchmark(btBroadphaseInterface*);
};

