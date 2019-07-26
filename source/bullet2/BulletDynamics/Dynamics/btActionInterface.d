module bullet2.BulletDynamics.Dynamics.btActionInterface;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btIDebugDraw;
import bullet2.BulletDynamics.Dynamics.btRigidBody;
import bullet2.BulletCollision.CollisionDispatch.btCollisionWorld;

///Basic interface to allow actions such as vehicles and characters to be updated inside a btDynamicsWorld
class btActionInterface
{
protected:
	static btRigidBody getFixedBody();

public:
	/*virtual*/ ~this()
	{
	}

	abstract /*virtual*/ void updateAction(btCollisionWorld collisionWorld, btScalar deltaTimeStep);

	abstract /*virtual*/ void debugDraw(btIDebugDraw* debugDrawer);
};
