module bullet2.BulletDynamics.Dynamics.btDiscreteDynamicsWorld;

extern (C++):

import bullet2.BulletDynamics.Dynamics.btDynamicsWorld;
import bullet2.BulletDynamics.Dynamics.btRigidBody;
import bullet2.BulletDynamics.ConstraintSolver.btTypedConstraint;
import bullet2.BulletDynamics.ConstraintSolver.btConstraintSolver;
import bullet2.BulletDynamics.ConstraintSolver.btContactSolverInfo;
import bullet2.BulletDynamics.Dynamics.btActionInterface;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseInterface;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.CollisionDispatch.btCollisionWorld;
import bullet2.BulletCollision.CollisionDispatch.btCollisionConfiguration;
import bullet2.BulletCollision.CollisionDispatch.btSimulationIslandManager;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btThreads;
import bullet2.LinearMath.btSerializer;

extern (C++, struct)
class InplaceSolverIslandCallback : /*btSimulationIslandManager.*/IslandCallback { }

///btDiscreteDynamicsWorld provides discrete rigid body simulation
///those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
//ATTRIBUTE_ALIGNED16(class)
align(16) class btDiscreteDynamicsWorld : btDynamicsWorld
{
protected:
	btAlignedObjectArray!btTypedConstraint m_sortedConstraints;
	InplaceSolverIslandCallback m_solverIslandCallback;

	btConstraintSolver m_constraintSolver;

	btSimulationIslandManager m_islandManager;

	btAlignedObjectArray!btTypedConstraint m_constraints;

	btAlignedObjectArray!btRigidBody m_nonStaticRigidBodies;

	btVector3 m_gravity;

	//for variable timesteps
	btScalar m_localTime;
	btScalar m_fixedTimeStep;
	//for variable timesteps

	bool m_ownsIslandManager;
	bool m_ownsConstraintSolver;
	bool m_synchronizeAllMotionStates;
	bool m_applySpeculativeContactRestitution;

	btAlignedObjectArray!btActionInterface m_actions;

	int m_profileTimings;

	bool m_latencyMotionStateInterpolation;

	btAlignedObjectArray!btPersistentManifold m_predictiveManifolds;
	btSpinMutex m_predictiveManifoldsMutex;  // used to synchronize threads creating predictive contacts

	/*virtual */void predictUnconstraintMotion(btScalar timeStep);

	final void integrateTransformsInternal(btRigidBody *bodies, int numBodies, btScalar timeStep);  // can be called in parallel
	/*virtual */void integrateTransforms(btScalar timeStep);

	/*virtual */void calculateSimulationIslands();

	/*virtual */void solveConstraints(ref btContactSolverInfo solverInfo);

	/*virtual */void updateActivationState(btScalar timeStep);

	final void updateActions(btScalar timeStep);

	final void startProfiling(btScalar timeStep);

	/*virtual */void internalSingleStepSimulation(btScalar timeStep);

	final void releasePredictiveContacts();
	final void createPredictiveContactsInternal(btRigidBody *bodies, int numBodies, btScalar timeStep);  // can be called in parallel
	/*virtual */void createPredictiveContacts(btScalar timeStep);

	/*virtual */void saveKinematicState(btScalar timeStep);

	final void serializeRigidBodies(btSerializer serializer);

	final void serializeDynamicsWorldInfo(btSerializer serializer);

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
	final this(btDispatcher dispatcher, btBroadphaseInterface pairCache, btConstraintSolver constraintSolver, btCollisionConfiguration collisionConfiguration);

	/*virtual */~this();

	///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
	override /*virtual */int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.));

	override /*virtual */void synchronizeMotionStates();

	///this can be useful to synchronize a single rigid body -> graphics object
	final void synchronizeSingleMotionState(btRigidBody body_);

	override /*virtual */void addConstraint(btTypedConstraint constraint, bool disableCollisionsBetweenLinkedBodies = false);

	override /*virtual */void removeConstraint(btTypedConstraint constraint);

	override /*virtual */void addAction(btActionInterface);

	override /*virtual */void removeAction(btActionInterface);

	final auto ref btSimulationIslandManager getSimulationIslandManager()
	{
		return m_islandManager;
	}

	final const(btSimulationIslandManager) getSimulationIslandManager() const
	{
		return m_islandManager;
	}

	final btCollisionWorld getCollisionWorld()
	{
		return this;
	}

	final void setGravity(const(btVector3) gravity)
	{
		setGravity(gravity);
	}
	override /*virtual */void setGravity(const ref btVector3 gravity);

	override /*virtual */btVector3 getGravity() const;

	override /*virtual */void addCollisionObject(btCollisionObject collisionObject, int collisionFilterGroup = btBroadphaseProxy.CollisionFilterGroups.StaticFilter, int collisionFilterMask = btBroadphaseProxy.CollisionFilterGroups.AllFilter ^ btBroadphaseProxy.CollisionFilterGroups.StaticFilter);

	override /*virtual */void addRigidBody(btRigidBody body_);

	override /*virtual */void addRigidBody(btRigidBody body_, int group, int mask);

	override /*virtual */void removeRigidBody(btRigidBody body_);

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
	override /*virtual */void removeCollisionObject(btCollisionObject collisionObject);

	/*virtual */void debugDrawConstraint(btTypedConstraint constraint);

	override /*virtual */void debugDrawWorld();

	override /*virtual */void setConstraintSolver(btConstraintSolver solver);

	override /*virtual */btConstraintSolver getConstraintSolver();

	override /*virtual */int getNumConstraints() const;

	override /*virtual */btTypedConstraint getConstraint(int index);

	pragma(mangle, "?getConstraint@btDiscreteDynamicsWorld@@UEBAPEBVbtTypedConstraint@@H@Z")
	override /*virtual */const(btTypedConstraint) getConstraint(int index) const;

	override /*virtual */btDynamicsWorldType getWorldType() const
	{
		return btDynamicsWorldType.BT_DISCRETE_DYNAMICS_WORLD;
	}

	///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
	override /*virtual */void clearForces();

	///apply gravity, call this once per timestep
	/*virtual */void applyGravity();

	/*virtual */void setNumTasks(int numTasks)
	{
		//(void)numTasks;
	}

	///obsolete, use updateActions instead
	/*virtual */void updateVehicles(btScalar timeStep)
	{
		updateActions(timeStep);
	}

	///obsolete, use addAction instead
	override /*virtual */void addVehicle(btActionInterface vehicle);
	///obsolete, use removeAction instead
	override /*virtual */void removeVehicle(btActionInterface vehicle);
	///obsolete, use addAction instead
	override /*virtual */void addCharacter(btActionInterface character);
	///obsolete, use removeAction instead
	override /*virtual */void removeCharacter(btActionInterface character);

	final void setSynchronizeAllMotionStates(bool synchronizeAll)
	{
		m_synchronizeAllMotionStates = synchronizeAll;
	}
	final bool getSynchronizeAllMotionStates() const
	{
		return m_synchronizeAllMotionStates;
	}

	final void setApplySpeculativeContactRestitution(bool enable)
	{
		m_applySpeculativeContactRestitution = enable;
	}

	final bool getApplySpeculativeContactRestitution() const
	{
		return m_applySpeculativeContactRestitution;
	}

	///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (see Bullet/Demos/SerializeDemo)
	override /*virtual */void serialize(btSerializer serializer);

	///Interpolate motion state between previous and current transform, instead of current and next transform.
	///This can relieve discontinuities in the rendering, due to penetrations
	final void setLatencyMotionStateInterpolation(bool latencyInterpolation)
	{
		m_latencyMotionStateInterpolation = latencyInterpolation;
	}
	final bool getLatencyMotionStateInterpolation() const
	{
		return m_latencyMotionStateInterpolation;
	}
};
