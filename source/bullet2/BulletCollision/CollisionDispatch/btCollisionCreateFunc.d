module bullet2.BulletCollision.CollisionDispatch.btCollisionCreateFunc;

import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.BulletCollision.BroadphaseCollision.btCollisionAlgorithm;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObjectWrapper;

extern (C++):

///Used by the btCollisionDispatcher to register and create instances for btCollisionAlgorithm
extern (C++, struct)
abstract class btCollisionAlgorithmCreateFunc
{
	bool m_swapped;// = false;

	final this()
	{
        m_swapped = false;
	}

	/*virtual*/ ~this(){};

	/*virtual*/ btCollisionAlgorithm* CreateCollisionAlgorithm(ref btCollisionAlgorithmConstructionInfo a, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap)
	{
		//(void)body0Wrap;
		//(void)body1Wrap;
		return null;
	}
};
