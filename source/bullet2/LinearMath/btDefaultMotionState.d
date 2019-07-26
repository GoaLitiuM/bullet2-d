module bullet2.LinearMath.btDefaultMotionState;

extern (C++):

import bullet2.LinearMath.btMotionState;
import bullet2.LinearMath.btTransform;

///The btDefaultMotionState provides a common implementation to synchronize world transforms with offsets.
extern (C++, struct)
align(16) class btDefaultMotionState : btMotionState
{
	btTransform m_graphicsWorldTrans;
	btTransform m_centerOfMassOffset;
	btTransform m_startWorldTrans;
	void* m_userPointer;

	//BT_DECLARE_ALIGNED_ALLOCATOR();

	this(btTransform startTrans = btTransform.getIdentity(), btTransform centerOfMassOffset = btTransform.getIdentity())
	{
        m_graphicsWorldTrans = startTrans;
        m_centerOfMassOffset = centerOfMassOffset;
        m_startWorldTrans = startTrans;
        m_userPointer = null;
	}

	///synchronizes world transform from user to physics
	final override void getWorldTransform(ref btTransform centerOfMassWorldTrans) const
	{
		centerOfMassWorldTrans = m_graphicsWorldTrans * m_centerOfMassOffset.inverse();
	}

	///synchronizes world transform from physics to user
	///Bullet only calls the update of worldtransform for active objects
	final override void setWorldTransform(ref const(btTransform) centerOfMassWorldTrans)
	{
		m_graphicsWorldTrans = centerOfMassWorldTrans * m_centerOfMassOffset;
	}
};
