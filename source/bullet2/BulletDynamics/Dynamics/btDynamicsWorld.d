module bullet2.BulletDynamics.Dynamics.btDynamicsWorld;

extern (C++):

import bullet2.BulletDynamics.ConstraintSolver.btTypedConstraint;
import bullet2.BulletDynamics.ConstraintSolver.btConstraintSolver;
import bullet2.BulletCollision.CollisionDispatch.btCollisionWorld;
import bullet2.BulletCollision.CollisionDispatch.btCollisionConfiguration;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseInterface;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletDynamics.ConstraintSolver.btContactSolverInfo;
import bullet2.BulletDynamics.Dynamics.btActionInterface;
import bullet2.BulletDynamics.Dynamics.btRigidBody;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;

/// Type for the callback for each tick
//typedef void (*btInternalTickCallback)(btDynamicsWorld* world, btScalar timeStep);
alias btInternalTickCallback = void function(btDynamicsWorld world, btScalar timeStep);

enum btDynamicsWorldType
{
	BT_SIMPLE_DYNAMICS_WORLD = 1,
	BT_DISCRETE_DYNAMICS_WORLD = 2,
	BT_CONTINUOUS_DYNAMICS_WORLD = 3,
	BT_SOFT_RIGID_DYNAMICS_WORLD = 4,
	BT_GPU_DYNAMICS_WORLD = 5,
	BT_SOFT_MULTIBODY_DYNAMICS_WORLD = 6
};

///The btDynamicsWorld is the interface class for several dynamics implementation, basic, discrete, parallel, and continuous etc.
class btDynamicsWorld : btCollisionWorld
{
protected:
	btInternalTickCallback m_internalTickCallback;
	btInternalTickCallback m_internalPreTickCallback;
	void* m_worldUserInfo;

	btContactSolverInfo m_solverInfo;

public:
	final this(btDispatcher dispatcher, btBroadphaseInterface broadphase, btCollisionConfiguration collisionConfiguration)
	{
		super(dispatcher, broadphase, collisionConfiguration);
		m_internalTickCallback = null;
		m_internalPreTickCallback = null;
		m_worldUserInfo = null;
	}

	/*virtual*/ ~this()
	{
	}

	///stepSimulation proceeds the simulation over 'timeStep', units in preferably in seconds.
	///By default, Bullet will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
	///in order to keep the simulation real-time, the maximum number of substeps can be clamped to 'maxSubSteps'.
	///You can disable subdividing the timestep/substepping by passing maxSubSteps=0 as second argument to stepSimulation, but in that case you have to keep the timeStep constant.
	/*virtual*/ abstract int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = (1.0) / (60.0));

	/*virtual*/ override void debugDrawWorld();

	/*virtual*/ void addConstraint(btTypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies = false)
	{
		//(void)constraint;
		//(void)disableCollisionsBetweenLinkedBodies;
	}

	/*virtual*/ void removeConstraint(btTypedConstraint constraint) { /*(void)constraint;*/ }

	/*virtual*/ abstract void addAction(btActionInterface action);

	/*virtual*/ abstract void removeAction(btActionInterface action);

	//once a rigidbody is added to the dynamics world, it will get this gravity assigned
	//existing rigidbodies in the world get gravity assigned too, during this method
	/*virtual*/ abstract void setGravity(const ref btVector3 gravity);
	/*virtual*/ abstract btVector3 getGravity() const;

	/*virtual*/ abstract void synchronizeMotionStates();

	/*virtual*/ abstract void addRigidBody(btRigidBody body_);

	/*virtual*/ abstract void addRigidBody(btRigidBody body_, int group, int mask);

	/*virtual*/ abstract void removeRigidBody(btRigidBody body_);

	/*virtual*/ abstract void setConstraintSolver(btConstraintSolver solver);

	/*virtual*/ abstract btConstraintSolver getConstraintSolver();

	/*virtual*/ int getNumConstraints() const { return 0; }

	/*virtual*/ btTypedConstraint getConstraint(int index)
	{
		//(void)index;
		return null;
	}

	/*virtual*/ const(btTypedConstraint) getConstraint(int index) const
	{
		//(void)index;
		return null;
	}

	/*virtual*/ abstract btDynamicsWorldType getWorldType() const;

	/*virtual*/ abstract void clearForces();

	/// Set the callback for when an internal tick (simulation substep) happens, optional user info
	final void setInternalTickCallback(btInternalTickCallback cb, void* worldUserInfo = null, bool isPreTick = false)
	{
		if (isPreTick)
		{
			m_internalPreTickCallback = cb;
		}
		else
		{
			m_internalTickCallback = cb;
		}
		m_worldUserInfo = worldUserInfo;
	}

	final void setWorldUserInfo(void* worldUserInfo)
	{
		m_worldUserInfo = worldUserInfo;
	}

	final void* getWorldUserInfo() //const
	{
		return m_worldUserInfo;
	}

	/*final ref btContactSolverInfo getSolverInfo()
	{
		return m_solverInfo;
	}

	final ref const(btContactSolverInfo) getSolverInfo() const
	{
		return m_solverInfo;
	}*/

	final btContactSolverInfo* getSolverInfo()
	{
		return &m_solverInfo;
	}

	final const(btContactSolverInfo)* getSolverInfo() const
	{
		return &m_solverInfo;
	}

	///obsolete, use addAction instead.
	/*virtual*/ void addVehicle(btActionInterface vehicle) { /*(void)vehicle;*/ }
	///obsolete, use removeAction instead
	/*virtual*/ void removeVehicle(btActionInterface vehicle) { /*(void)vehicle;*/ }
	///obsolete, use addAction instead.
	/*virtual*/ void addCharacter(btActionInterface character) { /*(void)character;*/ }
	///obsolete, use removeAction instead
	/*virtual*/ void removeCharacter(btActionInterface character) { /*(void)character;*/ }
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btDynamicsWorldDoubleData
{
	btContactSolverInfoDoubleData m_solverInfo;
	btVector3DoubleData m_gravity;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btDynamicsWorldFloatData
{
	btContactSolverInfoFloatData m_solverInfo;
	btVector3FloatData m_gravity;
};
