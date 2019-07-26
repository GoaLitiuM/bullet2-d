module bullet2.BulletCollision.CollisionDispatch.btSimulationIslandManager;

extern (C++):

import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletCollision.CollisionDispatch.btUnionFind;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.CollisionDispatch.btCollisionWorld;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;

extern (C++, struct)
class IslandCallback
{
    /*virtual*/ ~this(){};

    /*virtual*/ abstract void processIsland(btCollisionObject* bodies, int numBodies, btPersistentManifold* manifolds, int numManifolds, int islandId);
};

///SimulationIslandManager creates and handles simulation islands, using btUnionFind
class btSimulationIslandManager
{
	btUnionFind m_unionFind;

	btAlignedObjectArray!btPersistentManifold m_islandmanifold;
	btAlignedObjectArray!btCollisionObject m_islandBodies;

	bool m_splitIslands;

public:
	this();
	/*virtual*/ ~this();

	final void initUnionFind(int n);

	ref btUnionFind getUnionFind() { return m_unionFind; }

	/*virtual*/ void updateActivationState(btCollisionWorld colWorld, btDispatcher dispatcher);
	/*virtual*/ void storeIslandActivationState(btCollisionWorld world);

	final void findUnions(btDispatcher dispatcher, btCollisionWorld colWorld);

	final void buildAndProcessIslands(btDispatcher dispatcher, btCollisionWorld collisionWorld, IslandCallback callback);

	final void buildIslands(btDispatcher dispatcher, btCollisionWorld colWorld);

	bool getSplitIslands()
	{
		return m_splitIslands;
	}
	void setSplitIslands(bool doSplitIslands)
	{
		m_splitIslands = doSplitIslands;
	}
};
