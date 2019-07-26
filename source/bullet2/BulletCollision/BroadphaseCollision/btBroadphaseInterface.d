module bullet2.BulletCollision.BroadphaseCollision.btBroadphaseInterface;

extern (C++):

import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCache;

//struct btDispatcherInfo {};

extern (C++, struct)
struct btBroadphaseAabbCallback
{
	/*virtual*/ ~this() {}
	/*virtual*/ bool process(const(btBroadphaseProxy)* proxy);
};

extern (C++, struct)
struct btBroadphaseRayCallback //: btBroadphaseAabbCallback
{
	btBroadphaseAabbCallback base_;
	alias base_ this;
	///added some cached data to accelerate ray-AABB tests
	btVector3 m_rayDirectionInverse;
	uint[3] m_signs;
	btScalar m_lambda_max;

	/*virtual*/ ~this() { }

protected:
	//this() {}
};

///The btBroadphaseInterface class provides an interface to detect aabb-overlapping object pairs.
///Some implementations for this broadphase interface include btAxisSweep3, bt32BitAxisSweep3 and btDbvtBroadphase.
///The actual overlapping pair management, storage, adding and removing of pairs is dealt by the btOverlappingPairCache class.
extern (C++, class)
class btBroadphaseInterface
{
public:
	/*virtual*/ ~this() {}

	/*virtual*/ abstract btBroadphaseProxy createProxy(const ref btVector3 aabbMin, const ref btVector3 aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher dispatcher);
	/*virtual*/ abstract void destroyProxy(btBroadphaseProxy proxy, btDispatcher dispatcher);
	/*virtual*/ abstract void setAabb(btBroadphaseProxy proxy, const ref btVector3 aabbMin, const ref btVector3 aabbMax, btDispatcher dispatcher);
	/*virtual*/ abstract void getAabb(btBroadphaseProxy proxy, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

    /*final void rayTest(const ref btVector3 rayFrom, const ref btVector3 rayTo, ref btBroadphaseRayCallback rayCallback)
    {
        btVector3 zero = btVector3(0, 0, 0);
        rayTest(rayFrom, rayTo, rayCallback, zero, zero);
    }*/
	/*virtual*/ abstract void rayTest(const ref btVector3 rayFrom, const ref btVector3 rayTo, ref btBroadphaseRayCallback rayCallback, const ref btVector3 aabbMin, const ref btVector3 aabbMax);

	/*virtual*/ abstract void aabbTest(const ref btVector3 aabbMin, const ref btVector3 aabbMax, ref btBroadphaseAabbCallback callback);

	///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
	/*virtual*/ abstract void calculateOverlappingPairs(btDispatcher dispatcher);

	/*virtual*/ abstract btOverlappingPairCache getOverlappingPairCache();
	/*virtual*/ abstract const(btOverlappingPairCache) getOverlappingPairCache() const;

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	/*virtual*/ abstract void getBroadphaseAabb(ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	///reset broadphase internal structures, to ensure determinism/reproducability
	/*virtual*/ void resetPool(btDispatcher dispatcher) { /*(void)dispatcher;*/ };

	/*virtual*/ abstract void printStats();
};
