module bullet2.BulletDynamics.ConstraintSolver.btSequentialImpulseConstraintSolverMt;

extern (C++):

import bullet2.BulletDynamics.ConstraintSolver.btSequentialImpulseConstraintSolver;
import bullet2.BulletDynamics.ConstraintSolver.btTypedConstraint;
import bullet2.BulletDynamics.ConstraintSolver.btContactSolverInfo;
import bullet2.BulletDynamics.ConstraintSolver.btBatchedConstraints;

import bullet2.BulletCollision.NarrowPhaseCollision.btManifoldPoint;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btThreads;
import bullet2.LinearMath.btIDebugDraw;
import bullet2.LinearMath.btAlignedObjectArray;

import core.stdc.config : c_ulong;

//import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;

///
/// btSequentialImpulseConstraintSolverMt
///
///  A multithreaded variant of the sequential impulse constraint solver. The constraints to be solved are grouped into
///  batches and phases where each batch of constraints within a given phase can be solved in parallel with the rest.
///  Ideally we want as few phases as possible, and each phase should have many batches, and all of the batches should
///  have about the same number of constraints.
///  This method works best on a large island of many constraints.
///
///  Supports all of the features of the normal sequential impulse solver such as:
///    - split penetration impulse
///    - rolling friction
///    - interleaving constraints
///    - warmstarting
///    - 2 friction directions
///    - randomized constraint ordering
///    - early termination when leastSquaresResidualThreshold is satisfied
///
///  When the SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS flag is enabled, unlike the normal SequentialImpulse solver,
///  the rolling friction is interleaved as well.
///  Interleaving the contact penetration constraints with friction reduces the number of parallel loops that need to be done,
///  which reduces threading overhead so it can be a performance win, however, it does seem to produce a less stable simulation,
///  at least on stacks of blocks.
///
///  When the SOLVER_RANDMIZE_ORDER flag is enabled, the ordering of phases, and the ordering of constraints within each batch
///  is randomized, however it does not swap constraints between batches.
///  This is to avoid regenerating the batches for each solver iteration which would be quite costly in performance.
///
///  Note that a non-zero leastSquaresResidualThreshold could possibly affect the determinism of the simulation
///  if the task scheduler's parallelSum operation is non-deterministic. The parallelSum operation can be non-deterministic
///  because floating point addition is not associative due to rounding errors.
///  The task scheduler can and should ensure that the result of any parallelSum operation is deterministic.
///
align(16) class btSequentialImpulseConstraintSolverMt : btSequentialImpulseConstraintSolver
{
public:
	override /*virtual*/ void solveGroupCacheFriendlySplitImpulseIterations(btCollisionObject *bodies, int numBodies, btPersistentManifold* manifoldPtr, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal, btIDebugDraw* debugDrawer);
	override /*virtual*/ btScalar solveSingleIteration(int iteration, btCollisionObject* bodies, int numBodies, btPersistentManifold* manifoldPtr, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal, btIDebugDraw* debugDrawer);
	override /*virtual*/ btScalar solveGroupCacheFriendlySetup(btCollisionObject *bodies, int numBodies, btPersistentManifold* manifoldPtr, int numManifolds, btTypedConstraint* constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal, btIDebugDraw* debugDrawer);
	override /*virtual*/ btScalar solveGroupCacheFriendlyFinish(btCollisionObject *bodies, int numBodies, ref const(btContactSolverInfo) infoGlobal);

	// temp struct used to collect info from persistent manifolds into a cache-friendly struct using multiple threads
	struct btContactManifoldCachedInfo
	{
		enum int MAX_NUM_CONTACT_POINTS = 4;

		int numTouchingContacts;
		int[2] solverBodyIds;
		int contactIndex;
		int rollingFrictionIndex;
		bool[MAX_NUM_CONTACT_POINTS] contactHasRollingFriction;
		btManifoldPoint*[MAX_NUM_CONTACT_POINTS] contactPoints;
	};
	// temp struct used for setting up joint constraints in parallel
	struct JointParams
	{
		int m_solverConstraint;
		int m_solverBodyA;
		int m_solverBodyB;
	};
	final void internalInitMultipleJoints(btTypedConstraint *constraints, int iBegin, int iEnd);
	final void internalConvertMultipleJoints(ref const(btAlignedObjectArray!JointParams) jointParamsArray, btTypedConstraint* constraints, int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);

	// parameters to control batching
	extern static __gshared bool s_allowNestedParallelForLoops;        // whether to allow nested parallel operations
	extern static __gshared int s_minimumContactManifoldsForBatching;  // don't even try to batch if fewer manifolds than this
	extern static __gshared btBatchedConstraints.BatchingMethod s_contactBatchingMethod;
	extern static __gshared btBatchedConstraints.BatchingMethod s_jointBatchingMethod;
	extern static __gshared int s_minBatchSize;  // desired number of constraints per batch
	extern static __gshared int s_maxBatchSize;

protected:
	enum int CACHE_LINE_SIZE = 64;

	btBatchedConstraints m_batchedContactConstraints;
	btBatchedConstraints m_batchedJointConstraints;
	int m_numFrictionDirections;
	bool m_useBatching;
	bool m_useObsoleteJointConstraints;
	btAlignedObjectArray!btContactManifoldCachedInfo m_manifoldCachedInfoArray;
	btAlignedObjectArray!int m_rollingFrictionIndexTable;  // lookup table mapping contact index to rolling friction index
	btSpinMutex m_bodySolverArrayMutex;
	char[CACHE_LINE_SIZE] m_antiFalseSharingPadding;  // padding to keep mutexes in separate cachelines
	btSpinMutex m_kinematicBodyUniqueIdToSolverBodyTableMutex;
	btAlignedObjectArray!char m_scratchMemory;

	/*virtual*/ void randomizeConstraintOrdering(int iteration, int numIterations);
	/*virtual*/ btScalar resolveAllJointConstraints(int iteration);
	/*virtual*/ btScalar resolveAllContactConstraints();
	/*virtual*/ btScalar resolveAllContactFrictionConstraints();
	/*virtual*/ btScalar resolveAllContactConstraintsInterleaved();
	/*virtual*/ btScalar resolveAllRollingFrictionConstraints();

	/*virtual*/ void setupBatchedContactConstraints();
	/*virtual*/ void setupBatchedJointConstraints();
	override /*virtual*/ void convertJoints(btTypedConstraint *constraints, int numConstraints, ref const(btContactSolverInfo) infoGlobal);
	override /*virtual*/ void convertContacts(btPersistentManifold *manifoldPtr, int numManifolds, ref const(btContactSolverInfo) infoGlobal);
	override /*virtual*/ void convertBodies(btCollisionObject *bodies, int numBodies, ref const(btContactSolverInfo) infoGlobal);

	final int getOrInitSolverBodyThreadsafe(ref btCollisionObject body_, btScalar timeStep);
	final void allocAllContactConstraints(btPersistentManifold *manifoldPtr, int numManifolds, ref const(btContactSolverInfo) infoGlobal);
	final void setupAllContactConstraints(ref const(btContactSolverInfo) infoGlobal);
	final void randomizeBatchedConstraintOrdering(btBatchedConstraints * batchedConstraints);

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this();
	/*virtual*/ ~this();

	final btScalar resolveMultipleJointConstraints(ref const(btAlignedObjectArray!int) consIndices, int batchBegin, int batchEnd, int iteration);
	final btScalar resolveMultipleContactConstraints(ref const(btAlignedObjectArray!int) consIndices, int batchBegin, int batchEnd);
	final btScalar resolveMultipleContactSplitPenetrationImpulseConstraints(ref const(btAlignedObjectArray!int) consIndices, int batchBegin, int batchEnd);
	final btScalar resolveMultipleContactFrictionConstraints(ref const(btAlignedObjectArray!int) consIndices, int batchBegin, int batchEnd);
	final btScalar resolveMultipleContactRollingFrictionConstraints(ref const(btAlignedObjectArray!int) consIndices, int batchBegin, int batchEnd);
	final btScalar resolveMultipleContactConstraintsInterleaved(ref const(btAlignedObjectArray!int) contactIndices, int batchBegin, int batchEnd);

	final void internalCollectContactManifoldCachedInfo(btContactManifoldCachedInfo * cachedInfoArray, btPersistentManifold *manifoldPtr, int numManifolds, ref const(btContactSolverInfo) infoGlobal);
	final void internalAllocContactConstraints(const btContactManifoldCachedInfo* cachedInfoArray, int numManifolds);
	final void internalSetupContactConstraints(int iContactConstraint, ref const(btContactSolverInfo) infoGlobal);
	final void internalConvertBodies(btCollisionObject *bodies, int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
	final void internalWriteBackContacts(int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
	final void internalWriteBackJoints(int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
	final void internalWriteBackBodies(int iBegin, int iEnd, ref const(btContactSolverInfo) infoGlobal);
};
