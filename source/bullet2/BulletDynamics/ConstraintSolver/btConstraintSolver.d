module bullet2.BulletDynamics.ConstraintSolver.btConstraintSolver;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btIDebugDraw;

import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;
import bullet2.BulletDynamics.ConstraintSolver.btTypedConstraint;
import bullet2.BulletDynamics.ConstraintSolver.btContactSolverInfo;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;

/// btConstraintSolver provides solver interface

enum btConstraintSolverType
{
	BT_SEQUENTIAL_IMPULSE_SOLVER = 1,
	BT_MLCP_SOLVER = 2,
	BT_NNCG_SOLVER = 4,
	BT_MULTIBODY_SOLVER = 8,
	BT_BLOCK_SOLVER = 16,
};

class btConstraintSolver
{
public:
	/*virtual*/ ~this() {}

	/*virtual*/ void prepareSolve(int /* numBodies */, int /* numManifolds */) {}

	///solve a group of constraints
	/*virtual*/ abstract btScalar solveGroup(btCollisionObject* bodies, int numBodies, btPersistentManifold* manifold, int numManifolds, btTypedConstraint* constraints, int numConstraints, const ref btContactSolverInfo info, btIDebugDraw* debugDrawer, btDispatcher dispatcher);

	/*virtual*/ void allSolved(const ref btContactSolverInfo /* info */, btIDebugDraw* /* debugDrawer */) {}

	///clear internal cached data and reset random seed
	/*virtual*/ abstract void reset();

	/*virtual*/ abstract btConstraintSolverType getSolverType() const;
};
