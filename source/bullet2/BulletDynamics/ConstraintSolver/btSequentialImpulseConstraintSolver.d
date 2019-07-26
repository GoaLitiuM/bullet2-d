module bullet2.BulletDynamics.ConstraintSolver.btSequentialImpulseConstraintSolver;

extern (C++):

import bullet2.BulletDynamics.ConstraintSolver.btTypedConstraint;
import bullet2.BulletDynamics.ConstraintSolver.btContactSolverInfo;
import bullet2.BulletDynamics.ConstraintSolver.btSolverBody;
import bullet2.BulletDynamics.ConstraintSolver.btSolverConstraint;
import bullet2.BulletDynamics.ConstraintSolver.btConstraintSolver;

import bullet2.BulletCollision.NarrowPhaseCollision.btManifoldPoint;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btIDebugDraw;
import bullet2.LinearMath.btAlignedObjectArray;

import core.stdc.config : c_ulong;

import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;

alias btSingleConstraintRowSolver = btScalar function(ref btSolverBody, ref btSolverBody, ref const(btSolverConstraint));
//typedef btScalar (*btSingleConstraintRowSolver)(btSolverBody&, btSolverBody&, const ref btSolverConstraint);

struct btSISolverSingleIterationData
{
	btAlignedObjectArray!(btSolverBody)* m_tmpSolverBodyPool;
	btConstraintArray* m_tmpSolverContactConstraintPool;
	btConstraintArray* m_tmpSolverNonContactConstraintPool;
	btConstraintArray* m_tmpSolverContactFrictionConstraintPool;
	btConstraintArray* m_tmpSolverContactRollingFrictionConstraintPool;

	btAlignedObjectArray!(int)* m_orderTmpConstraintPool;
	btAlignedObjectArray!(int)* m_orderNonContactConstraintPool;
	btAlignedObjectArray!(int)* m_orderFrictionConstraintPool;
	btAlignedObjectArray!(btTypedConstraint.btConstraintInfo1)* m_tmpConstraintSizesPool;
	c_ulong* m_seed;

	btSingleConstraintRowSolver* m_resolveSingleConstraintRowGeneric;
	btSingleConstraintRowSolver* m_resolveSingleConstraintRowLowerLimit;
	btSingleConstraintRowSolver* m_resolveSplitPenetrationImpulse;
	btAlignedObjectArray!(int)* m_kinematicBodyUniqueIdToSolverBodyTable;
	int* m_fixedBodyId;
	int* m_maxOverrideNumSolverIterations;
	int getOrInitSolverBody(ref btCollisionObject body_, btScalar timeStep);
	static void initSolverBody(btSolverBody * solverBody, btCollisionObject * collisionObject, btScalar timeStep);
	int getSolverBody(ref btCollisionObject body_) const;


	this(btAlignedObjectArray!(btSolverBody)* tmpSolverBodyPool,
		btConstraintArray* tmpSolverContactConstraintPool,
		btConstraintArray* tmpSolverNonContactConstraintPool,
		btConstraintArray* tmpSolverContactFrictionConstraintPool,
		btConstraintArray* tmpSolverContactRollingFrictionConstraintPool,
		btAlignedObjectArray!(int)* orderTmpConstraintPool,
		btAlignedObjectArray!(int)* orderNonContactConstraintPool,
		btAlignedObjectArray!(int)* orderFrictionConstraintPool,
		btAlignedObjectArray!(btTypedConstraint.btConstraintInfo1)* tmpConstraintSizesPool,
		btSingleConstraintRowSolver* resolveSingleConstraintRowGeneric,
		btSingleConstraintRowSolver* resolveSingleConstraintRowLowerLimit,
		btSingleConstraintRowSolver* resolveSplitPenetrationImpulse,
		btAlignedObjectArray!(int)* kinematicBodyUniqueIdToSolverBodyTable,
		c_ulong* seed,
		int* fixedBodyId,
		int* maxOverrideNumSolverIterations
	)
	{
        m_tmpSolverBodyPool = tmpSolverBodyPool;
		m_tmpSolverContactConstraintPool = tmpSolverContactConstraintPool;
		m_tmpSolverNonContactConstraintPool = tmpSolverNonContactConstraintPool;
		m_tmpSolverContactFrictionConstraintPool = tmpSolverContactFrictionConstraintPool;
		m_tmpSolverContactRollingFrictionConstraintPool = tmpSolverContactRollingFrictionConstraintPool;
		m_orderTmpConstraintPool = orderTmpConstraintPool;
		m_orderNonContactConstraintPool = orderNonContactConstraintPool;
		m_orderFrictionConstraintPool = orderFrictionConstraintPool;
		m_tmpConstraintSizesPool = tmpConstraintSizesPool;
		m_resolveSingleConstraintRowGeneric = resolveSingleConstraintRowGeneric;
		m_resolveSingleConstraintRowLowerLimit = resolveSingleConstraintRowLowerLimit;
		m_resolveSplitPenetrationImpulse = resolveSplitPenetrationImpulse;
		m_kinematicBodyUniqueIdToSolverBodyTable = kinematicBodyUniqueIdToSolverBodyTable;
		m_seed = seed;
		m_fixedBodyId = fixedBodyId;
		m_maxOverrideNumSolverIterations = maxOverrideNumSolverIterations;
	}
};

struct btSolverAnalyticsData
{
	/*btSolverAnalyticsData()
	{
		m_numSolverCalls = 0;
		m_numIterationsUsed = -1;
		m_remainingLeastSquaresResidual = -1;
		m_islandId = -2;
	}*/
	int m_islandId = -2;
	int m_numBodies;
	int m_numContactManifolds;
	int m_numSolverCalls = 0;
	int m_numIterationsUsed = -1;
	double m_remainingLeastSquaresResidual = -1;
};

///The btSequentialImpulseConstraintSolver is a fast SIMD implementation of the Projected Gauss Seidel (iterative LCP) method.
//ATTRIBUTE_ALIGNED16(class)
align(16) class btSequentialImpulseConstraintSolver : btConstraintSolver
{


protected:
	btAlignedObjectArray!(btSolverBody) m_tmpSolverBodyPool;
	btConstraintArray m_tmpSolverContactConstraintPool;
	btConstraintArray m_tmpSolverNonContactConstraintPool;
	btConstraintArray m_tmpSolverContactFrictionConstraintPool;
	btConstraintArray m_tmpSolverContactRollingFrictionConstraintPool;

	btAlignedObjectArray!(int) m_orderTmpConstraintPool;
	btAlignedObjectArray!(int) m_orderNonContactConstraintPool;
	btAlignedObjectArray!(int) m_orderFrictionConstraintPool;
	btAlignedObjectArray!(btTypedConstraint.btConstraintInfo1) m_tmpConstraintSizesPool;
	int m_maxOverrideNumSolverIterations;
	int m_fixedBodyId;
	// When running solvers on multiple threads, a race condition exists for Kinematic objects that
	// participate in more than one solver.
	// The getOrInitSolverBody() function writes the companionId of each body (storing the index of the solver body
	// for the current solver). For normal dynamic bodies it isn't an issue because they can only be in one island
	// (and therefore one thread) at a time. But kinematic bodies can be in multiple islands at once.
	// To avoid this race condition, this solver does not write the companionId, instead it stores the solver body
	// index in this solver-local table, indexed by the uniqueId of the body.
	btAlignedObjectArray!(int) m_kinematicBodyUniqueIdToSolverBodyTable;  // only used for multithreading

	btSingleConstraintRowSolver m_resolveSingleConstraintRowGeneric;
	btSingleConstraintRowSolver m_resolveSingleConstraintRowLowerLimit;
	btSingleConstraintRowSolver m_resolveSplitPenetrationImpulse;
	int m_cachedSolverMode;  // used to check if SOLVER_SIMD flag has been changed
	final void setupSolverFunctions(bool useSimd);

	btScalar m_leastSquaresResidual;

	final void setupFrictionConstraint(ref btSolverConstraint solverConstraint, const ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB,
		ref btManifoldPoint cp, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2,
		btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation,
		ref const(btContactSolverInfo) infoGlobal,
		btScalar desiredVelocity = 0., btScalar cfmSlip = 0.);

	final void setupTorsionalFrictionConstraint(ref btSolverConstraint solverConstraint, const ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB,
		ref btManifoldPoint cp, btScalar combinedTorsionalFriction, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2,
		btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation,
		btScalar desiredVelocity = 0., btScalar cfmSlip = 0.);

	final ref btSolverConstraint addFrictionConstraint(const ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, ref btManifoldPoint cp, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, ref const(btContactSolverInfo) infoGlobal, btScalar desiredVelocity = 0, btScalar cfmSlip = 0);
	final ref btSolverConstraint addTorsionalFrictionConstraint(const ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, ref btManifoldPoint cp, btScalar torsionalFriction, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, btScalar desiredVelocity = 0, btScalar cfmSlip = 0);

	final void setupContactConstraint(ref btSolverConstraint solverConstraint, int solverBodyIdA, int solverBodyIdB, ref btManifoldPoint cp,
		ref const(btContactSolverInfo) infoGlobal, ref btScalar relaxation, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2);

	final static void applyAnisotropicFriction(btCollisionObject * colObj, ref btVector3 frictionDirection, int frictionMode);

	final void setFrictionConstraintImpulse(ref btSolverConstraint solverConstraint, int solverBodyIdA, int solverBodyIdB,
		ref btManifoldPoint cp, ref const(btContactSolverInfo) infoGlobal);

	///m_btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
	c_ulong m_btSeed2;

	final btScalar restitutionCurve(btScalar rel_vel, btScalar restitution, btScalar velocityThreshold);

	/*virtual*/ void convertContacts(btPersistentManifold* manifoldPtr, int numManifolds, ref const(btContactSolverInfo) infoGlobal);

	final void convertContact(btPersistentManifold * manifold, ref const(btContactSolverInfo) infoGlobal);

	/*virtual*/ void convertJoints(btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal);
	final void convertJoint(btSolverConstraint * currentConstraintRow, btTypedConstraint * constraint, const ref btTypedConstraint.btConstraintInfo1 info1, int solverBodyIdA, int solverBodyIdB, ref const(btContactSolverInfo) infoGlobal);


	/*virtual*/ void convertBodies(btCollisionObject *bodies, int numBodies, ref const(btContactSolverInfo) infoGlobal);

	final btScalar resolveSplitPenetrationSIMD(ref btSolverBody bodyA, ref btSolverBody bodyB, const ref btSolverConstraint contactConstraint)
	{
		return m_resolveSplitPenetrationImpulse(bodyA, bodyB, contactConstraint);
	}

	final btScalar resolveSplitPenetrationImpulseCacheFriendly(ref btSolverBody bodyA, ref btSolverBody bodyB, const ref btSolverConstraint contactConstraint)
	{
		return m_resolveSplitPenetrationImpulse(bodyA, bodyB, contactConstraint);
	}

	//internal method
	final int getOrInitSolverBody(ref btCollisionObject body_, btScalar timeStep);
	final void initSolverBody(btSolverBody * solverBody, btCollisionObject * collisionObject, btScalar timeStep);

	final btScalar resolveSingleConstraintRowGeneric(ref btSolverBody bodyA, ref btSolverBody bodyB, const ref btSolverConstraint contactConstraint);
	final btScalar resolveSingleConstraintRowGenericSIMD(ref btSolverBody bodyA, ref btSolverBody bodyB, const ref btSolverConstraint contactConstraint);
	final btScalar resolveSingleConstraintRowLowerLimit(ref btSolverBody bodyA, ref btSolverBody bodyB, const ref btSolverConstraint contactConstraint);
	final btScalar resolveSingleConstraintRowLowerLimitSIMD(ref btSolverBody bodyA, ref btSolverBody bodyB, const ref btSolverConstraint contactConstraint);
	final btScalar resolveSplitPenetrationImpulse(ref btSolverBody bodyA, ref btSolverBody bodyB, const ref btSolverConstraint contactConstraint)
	{
		return m_resolveSplitPenetrationImpulse(bodyA, bodyB, contactConstraint);
	}

public:

	final void writeBackContacts(int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
	final void writeBackJoints(int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
	final void writeBackBodies(int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
	/*virtual*/ void solveGroupCacheFriendlySplitImpulseIterations(btCollisionObject *bodies, int numBodies, btPersistentManifold* manifoldPtr, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal, btIDebugDraw* debugDrawer);
	/*virtual*/ btScalar solveGroupCacheFriendlyFinish(btCollisionObject *bodies, int numBodies, ref const(btContactSolverInfo) infoGlobal);
	/*virtual*/ btScalar solveSingleIteration(int iteration, btCollisionObject* bodies, int numBodies, btPersistentManifold* manifoldPtr, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal, btIDebugDraw* debugDrawer);


	/*virtual*/ btScalar solveGroupCacheFriendlySetup(btCollisionObject *bodies, int numBodies, btPersistentManifold* manifoldPtr, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal, btIDebugDraw* debugDrawer);
	/*virtual*/ btScalar solveGroupCacheFriendlyIterations(btCollisionObject *bodies, int numBodies, btPersistentManifold* manifoldPtr, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal, btIDebugDraw* debugDrawer);

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this();
	/*virtual*/ ~this();

	/*virtual*/ override btScalar solveGroup(btCollisionObject* bodies, int numBodies, btPersistentManifold* manifold, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) info, btIDebugDraw* debugDrawer, btDispatcher dispatcher);

	final static btScalar solveSingleIterationInternal(ref btSISolverSingleIterationData siData, int iteration, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal);
	final static void convertBodiesInternal(ref btSISolverSingleIterationData siData, btCollisionObject* bodies, int numBodies, ref const(btContactSolverInfo) infoGlobal);
	final static void convertJointsInternal(ref btSISolverSingleIterationData siData, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal);
	final static void convertContactInternal(ref btSISolverSingleIterationData siData, btPersistentManifold * manifold, ref const(btContactSolverInfo) infoGlobal);
	final static void setupContactConstraintInternal(ref btSISolverSingleIterationData siData, ref btSolverConstraint solverConstraint, int solverBodyIdA, int solverBodyIdB, ref btManifoldPoint cp, ref const(btContactSolverInfo) infoGlobal, ref btScalar relaxation,
		const ref btVector3 rel_pos1, const ref btVector3 rel_pos2);
	final static btScalar restitutionCurveInternal(btScalar rel_vel, btScalar restitution, btScalar velocityThreshold);
	final static ref btSolverConstraint addTorsionalFrictionConstraintInternal(ref btAlignedObjectArray!(btSolverBody) tmpSolverBodyPool, ref btConstraintArray tmpSolverContactRollingFrictionConstraintPool, const ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, ref btManifoldPoint cp, btScalar combinedTorsionalFriction, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, btScalar desiredVelocity = 0, btScalar cfmSlip = 0.);
	final static void setupTorsionalFrictionConstraintInternal(ref btAlignedObjectArray!(btSolverBody) tmpSolverBodyPool, ref btSolverConstraint solverConstraint, const ref btVector3 normalAxis1, int solverBodyIdA, int solverBodyIdB,
		ref btManifoldPoint cp, btScalar combinedTorsionalFriction, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2,
		btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation,
		btScalar desiredVelocity, btScalar cfmSlip);
	final static void setupFrictionConstraintInternal(ref btAlignedObjectArray!(btSolverBody) tmpSolverBodyPool, ref btSolverConstraint solverConstraint, const ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB, ref btManifoldPoint cp, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, ref const(btContactSolverInfo) infoGlobal, btScalar desiredVelocity, btScalar cfmSlip);
	final static ref btSolverConstraint addFrictionConstraintInternal(ref btAlignedObjectArray!(btSolverBody) tmpSolverBodyPool, ref btConstraintArray tmpSolverContactFrictionConstraintPool, const ref btVector3 normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, ref btManifoldPoint cp, const ref btVector3 rel_pos1, const ref btVector3 rel_pos2, btCollisionObject* colObj0, btCollisionObject* colObj1, btScalar relaxation, ref const(btContactSolverInfo) infoGlobal, btScalar desiredVelocity = 0., btScalar cfmSlip = 0.);
	final static void setFrictionConstraintImpulseInternal(ref btAlignedObjectArray!(btSolverBody) tmpSolverBodyPool, ref btConstraintArray tmpSolverContactFrictionConstraintPool,
		ref btSolverConstraint solverConstraint,
		int solverBodyIdA, int solverBodyIdB,
		ref btManifoldPoint cp, ref const(btContactSolverInfo) infoGlobal);
	final static void convertJointInternal(ref btAlignedObjectArray!(btSolverBody) tmpSolverBodyPool,
		ref int maxOverrideNumSolverIterations,
		btSolverConstraint* currentConstraintRow,
		btTypedConstraint* constraint,
		const ref btTypedConstraint.btConstraintInfo1 info1,
		int solverBodyIdA,
		int solverBodyIdB,
		ref const(btContactSolverInfo) infoGlobal);

	final static btScalar solveGroupCacheFriendlyFinishInternal(ref btSISolverSingleIterationData siData, btCollisionObject* bodies, int numBodies, ref const(btContactSolverInfo) infoGlobal);

	final static void writeBackContactsInternal(ref btConstraintArray tmpSolverContactConstraintPool, ref btConstraintArray tmpSolverContactFrictionConstraintPool, int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);

	final static void writeBackJointsInternal(ref btConstraintArray tmpSolverNonContactConstraintPool, int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
	final static void writeBackBodiesInternal(ref btAlignedObjectArray!(btSolverBody) tmpSolverBodyPool, int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
	final static void solveGroupCacheFriendlySplitImpulseIterationsInternal(ref btSISolverSingleIterationData siData, btCollisionObject* bodies, int numBodies, btPersistentManifold* manifoldPtr, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal, btIDebugDraw* debugDrawer);


	///clear internal cached data and reset random seed
	/*virtual*/ override void reset();

	final c_ulong btRand2();
	final int btRandInt2(int n);

	final static c_ulong btRand2a(ref c_ulong seed);
	final static int btRandInt2a(int n, ref c_ulong seed);

	final void setRandSeed(c_ulong seed)
	{
		m_btSeed2 = seed;
	}
	final c_ulong getRandSeed() const
	{
		return m_btSeed2;
	}

	/*virtual*/ override btConstraintSolverType getSolverType() const
	{
		return btConstraintSolverType.BT_SEQUENTIAL_IMPULSE_SOLVER;
	}

	final btSingleConstraintRowSolver getActiveConstraintRowSolverGeneric()
	{
		return m_resolveSingleConstraintRowGeneric;
	}
	final void setConstraintRowSolverGeneric(btSingleConstraintRowSolver rowSolver)
	{
		m_resolveSingleConstraintRowGeneric = rowSolver;
	}
	final btSingleConstraintRowSolver getActiveConstraintRowSolverLowerLimit()
	{
		return m_resolveSingleConstraintRowLowerLimit;
	}
	final void setConstraintRowSolverLowerLimit(btSingleConstraintRowSolver rowSolver)
	{
		m_resolveSingleConstraintRowLowerLimit = rowSolver;
	}



	///Various implementations of solving a single constraint row using a generic equality constraint, using scalar reference, SSE2 or SSE4
	final static btSingleConstraintRowSolver getScalarConstraintRowSolverGeneric();
	final static btSingleConstraintRowSolver getSSE2ConstraintRowSolverGeneric();
	final static btSingleConstraintRowSolver getSSE4_1ConstraintRowSolverGeneric();

	///Various implementations of solving a single constraint row using an inequality (lower limit) constraint, using scalar reference, SSE2 or SSE4
	final static btSingleConstraintRowSolver getScalarConstraintRowSolverLowerLimit();
	final static btSingleConstraintRowSolver getSSE2ConstraintRowSolverLowerLimit();
	final static btSingleConstraintRowSolver getSSE4_1ConstraintRowSolverLowerLimit();

	final static btSingleConstraintRowSolver getScalarSplitPenetrationImpulseGeneric();
	final static btSingleConstraintRowSolver getSSE2SplitPenetrationImpulseGeneric();

	btSolverAnalyticsData m_analyticsData;
};
