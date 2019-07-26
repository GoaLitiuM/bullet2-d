module bullet2.BulletCollision.CollisionDispatch.btDefaultCollisionConfiguration;

extern (C++):
@nogc:
nothrow:

import bullet2.BulletCollision.CollisionDispatch.btCollisionConfiguration;
import bullet2.BulletCollision.CollisionDispatch.btCollisionCreateFunc;
import bullet2.LinearMath.btPoolAllocator;

struct btVoronoiSimplexSolver {};
struct btConvexPenetrationDepthSolver {};

struct btDefaultCollisionConstructionInfo
{
	btPoolAllocator m_persistentManifoldPool = null;
	btPoolAllocator m_collisionAlgorithmPool = null;
	int m_defaultMaxPersistentManifoldPoolSize = 4096;
	int m_defaultMaxCollisionAlgorithmPoolSize = 4096;
	int m_customCollisionAlgorithmMaxElementSize = 0;
	int m_useEpaPenetrationAlgorithm = true;
};
private __gshared const btDefaultCollisionConstructionInfoInstance = btDefaultCollisionConstructionInfo();

///btCollisionConfiguration allows to configure Bullet collision detection
///stack allocator, pool memory allocators
///@todo: describe the meaning
class btDefaultCollisionConfiguration : btCollisionConfiguration
{
protected:
	int m_persistentManifoldPoolSize;

	btPoolAllocator m_persistentManifoldPool;
	bool m_ownsPersistentManifoldPool;

	btPoolAllocator m_collisionAlgorithmPool;
	bool m_ownsCollisionAlgorithmPool;

	//default penetration depth solver
	btConvexPenetrationDepthSolver* m_pdSolver;

	//default CreationFunctions, filling the m_doubleDispatch table
	btCollisionAlgorithmCreateFunc m_convexConvexCreateFunc;
	btCollisionAlgorithmCreateFunc m_convexConcaveCreateFunc;
	btCollisionAlgorithmCreateFunc m_swappedConvexConcaveCreateFunc;
	btCollisionAlgorithmCreateFunc m_compoundCreateFunc;
	btCollisionAlgorithmCreateFunc m_compoundCompoundCreateFunc;

	btCollisionAlgorithmCreateFunc m_swappedCompoundCreateFunc;
	btCollisionAlgorithmCreateFunc m_emptyCreateFunc;
	btCollisionAlgorithmCreateFunc m_sphereSphereCF;
	btCollisionAlgorithmCreateFunc m_sphereBoxCF;
	btCollisionAlgorithmCreateFunc m_boxSphereCF;

	btCollisionAlgorithmCreateFunc m_boxBoxCF;
	btCollisionAlgorithmCreateFunc m_sphereTriangleCF;
	btCollisionAlgorithmCreateFunc m_triangleSphereCF;
	btCollisionAlgorithmCreateFunc m_planeConvexCF;
	btCollisionAlgorithmCreateFunc m_convexPlaneCF;

public:
    final this()
    {
		btDefaultCollisionConstructionInfo dcci = btDefaultCollisionConstructionInfo();
        this(dcci);
    }
	final this(ref const(btDefaultCollisionConstructionInfo) constructionInfo);

	~this();

	///memory pools
	override btPoolAllocator getPersistentManifoldPool()
	{
		return m_persistentManifoldPool;
	}

	override btPoolAllocator getCollisionAlgorithmPool()
	{
		return m_collisionAlgorithmPool;
	}

	override btCollisionAlgorithmCreateFunc getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1);

	override btCollisionAlgorithmCreateFunc getClosestPointsAlgorithmCreateFunc(int proxyType0, int proxyType1);

	///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
	///By default, this feature is disabled for best performance.
	///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
	///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
	///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
	///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
	///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
	final void setConvexConvexMultipointIterations(int numPerturbationIterations = 3, int minimumPointsPerturbationThreshold = 3);

	final void setPlaneConvexMultipointIterations(int numPerturbationIterations = 3, int minimumPointsPerturbationThreshold = 3);
};
