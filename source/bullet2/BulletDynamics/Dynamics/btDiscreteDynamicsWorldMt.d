module bullet2.BulletDynamics.Dynamics.btDiscreteDynamicsWorldMt;

extern (C++):

import bullet2.BulletDynamics.Dynamics.btDiscreteDynamicsWorld;
import bullet2.BulletCollision.CollisionDispatch.btSimulationIslandManager;

import bullet2.BulletDynamics.ConstraintSolver.btConstraintSolver;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btIDebugDraw;
import bullet2.LinearMath.btThreads;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletDynamics.ConstraintSolver.btContactSolverInfo;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseInterface;
import bullet2.BulletCollision.CollisionDispatch.btCollisionConfiguration;
import bullet2.BulletDynamics.ConstraintSolver.btTypedConstraint;
import bullet2.LinearMath.btAlignedObjectArray;


///
/// btConstraintSolverPoolMt - masquerades as a constraint solver, but really it is a threadsafe pool of them.
///
///  Each solver in the pool is protected by a mutex.  When solveGroup is called from a thread,
///  the pool looks for a solver that isn't being used by another thread, locks it, and dispatches the
///  call to the solver.
///  So long as there are at least as many solvers as there are hardware threads, it should never need to
///  spin wait.
///
class btConstraintSolverPoolMt : btConstraintSolver
{
public:
	// create the solvers for me
	final /*explicit*/ this(int numSolvers);

	// pass in fully constructed solvers (destructor will delete them)
	final this(btConstraintSolver* solvers, int numSolvers);

	/*virtual*/ ~this();

	///solve a group of constraints
	override /*virtual*/ btScalar solveGroup(btCollisionObject* bodies,
								int numBodies,
								btPersistentManifold* manifolds,
								int numManifolds,
								btTypedConstraint* constraints,
								int numConstraints,
								ref const(btContactSolverInfo) info,
								btIDebugDraw* debugDrawer,
								btDispatcher dispatcher);

	override /*virtual*/ void reset();
	override /*virtual*/ btConstraintSolverType getSolverType() const { return m_solverType; }

private:
	const __gshared static size_t kCacheLineSize = 128;
	struct ThreadSolver
	{
		btConstraintSolver solver;
		btSpinMutex mutex;
		char[kCacheLineSize - btSpinMutex.sizeof - (void*).sizeof] _cachelinePadding;  // keep mutexes from sharing a cache line
	};
	btAlignedObjectArray!ThreadSolver m_solvers;
	btConstraintSolverType m_solverType;

	final ThreadSolver* getAndLockThreadSolver();
	final void init(btConstraintSolver* solvers, int numSolvers);
};

///
/// btDiscreteDynamicsWorldMt -- a version of DiscreteDynamicsWorld with some minor changes to support
///                              solving simulation islands on multiple threads.
///
///  Should function exactly like btDiscreteDynamicsWorld.
///  Also 3 methods that iterate over all of the rigidbodies can run in parallel:
///     - predictUnconstraintMotion
///     - integrateTransforms
///     - createPredictiveContacts
///
align(16) class btDiscreteDynamicsWorldMt : btDiscreteDynamicsWorld
{
protected:
	btConstraintSolver m_constraintSolverMt;

	override /*virtual*/ void solveConstraints(ref btContactSolverInfo solverInfo);

	override /*virtual*/ void predictUnconstraintMotion(btScalar timeStep);

	/*struct UpdaterCreatePredictiveContacts : btIParallelForBody
	{
		btScalar timeStep;
		btRigidBody* rigidBodies;
		btDiscreteDynamicsWorldMt* world;

		override void forLoop(int iBegin, int iEnd) const
		{
			world->createPredictiveContactsInternal(&rigidBodies[iBegin], iEnd - iBegin, timeStep);
		}
	};*/
	override /*virtual*/ void createPredictiveContacts(btScalar timeStep);

	/*struct UpdaterIntegrateTransforms : btIParallelForBody
	{
		btScalar timeStep;
		btRigidBody* rigidBodies;
		btDiscreteDynamicsWorldMt* world;

		override void forLoop(int iBegin, int iEnd) const
		{
			world->integrateTransformsInternal(&rigidBodies[iBegin], iEnd - iBegin, timeStep);
		}
	};*/
	override /*virtual*/ void integrateTransforms(btScalar timeStep);

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	this(btDispatcher dispatcher,
							  btBroadphaseInterface pairCache,
							  btConstraintSolverPoolMt solverPool,        // Note this should be a solver-pool for multi-threading
							  btConstraintSolver constraintSolverMt,      // single multi-threaded solver for large islands (or NULL)
							  btCollisionConfiguration collisionConfiguration);
	/*virtual*/ ~this();

	override /*virtual*/ int stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep);
};
