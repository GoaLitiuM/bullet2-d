module bullet2.BulletCollision.BroadphaseCollision.btOverlappingPairCallback;

extern (C++):

import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;

///The btOverlappingPairCallback class is an additional optional broadphase user callback for adding/removing overlapping pairs, similar interface to btOverlappingPairCache.
extern (C++, class)
class btOverlappingPairCallback
{
protected:
	final this() {}

public:
	/*virtual*/ ~this()
	{
	}

	/*virtual*/ abstract btBroadphasePair* addOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1);

	/*virtual*/ abstract void* removeOverlappingPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1, btDispatcher dispatcher);

	/*virtual*/ abstract void removeOverlappingPairsContainingProxy(btBroadphaseProxy proxy0, btDispatcher dispatcher);
};
