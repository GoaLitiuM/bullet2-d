module bullet2.LinearMath.btMotionState;

extern (C++):

import bullet2.LinearMath.btTransform;

///The btMotionState interface class allows the dynamics world to synchronize and interpolate the updated world transforms with graphics
///For optimizations, potentially only moving objects get synchronized (using setWorldPosition/setWorldOrientation)
class btMotionState
{
public:
	/*virtual*/ ~this()
	{
	}

	/*virtual*/ abstract void getWorldTransform(ref btTransform worldTrans) const;

	//Bullet only calls the update of worldtransform for active objects
	/*virtual*/ abstract void setWorldTransform(ref const(btTransform) worldTrans);
};
