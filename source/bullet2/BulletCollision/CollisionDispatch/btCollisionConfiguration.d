module bullet2.BulletCollision.CollisionDispatch.btCollisionConfiguration;

extern (C++):

import bullet2.BulletCollision.CollisionDispatch.btCollisionCreateFunc;
import bullet2.LinearMath.btPoolAllocator;

///btCollisionConfiguration allows to configure Bullet collision detection
///stack allocator size, default collision algorithms and persistent manifold pool size
///@todo: describe the meaning
class btCollisionConfiguration
{
public:
	~this()
	{
	}

	///memory pools
	abstract btPoolAllocator getPersistentManifoldPool();

	abstract btPoolAllocator getCollisionAlgorithmPool();

	abstract btCollisionAlgorithmCreateFunc getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1);

	abstract btCollisionAlgorithmCreateFunc getClosestPointsAlgorithmCreateFunc(int proxyType0, int proxyType1);
};
