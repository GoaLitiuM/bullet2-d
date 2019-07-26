module bullet2.BulletDynamics.ConstraintSolver.btSolverConstraint;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btMatrix3x3;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.BulletDynamics.ConstraintSolver.btSolverBody;

//#include "btJacobianEntry.h"

//#define NO_FRICTION_TANGENTIALS 1
//#include "btSolverBody.h"

///1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.
//ATTRIBUTE_ALIGNED16(struct)
align(16) struct
btSolverConstraint
{
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	btVector3 m_relpos1CrossNormal;
	btVector3 m_contactNormal1;

	btVector3 m_relpos2CrossNormal;
	btVector3 m_contactNormal2;  //usually m_contactNormal2 == -m_contactNormal1, but not always

	btVector3 m_angularComponentA;
	btVector3 m_angularComponentB;

	/*mutable*/ btSimdScalar m_appliedPushImpulse;
	/*mutable*/ btSimdScalar m_appliedImpulse;

	btScalar m_friction;
	btScalar m_jacDiagABInv;
	btScalar m_rhs;
	btScalar m_cfm;

	btScalar m_lowerLimit;
	btScalar m_upperLimit;
	btScalar m_rhsPenetration;
	union {
		void* m_originalContactPoint;
		btScalar m_unusedPadding4;
		int m_numRowsForNonContactConstraint;
	};

	int m_overrideNumSolverIterations;
	int m_frictionIndex;
	int m_solverBodyIdA;
	int m_solverBodyIdB;

	enum btSolverConstraintType
	{
		BT_SOLVER_CONTACT_1D = 0,
		BT_SOLVER_FRICTION_1D
	};
};

alias btConstraintArray = btAlignedObjectArray!btSolverConstraint;
