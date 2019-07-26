module bullet2.BulletDynamics.ConstraintSolver.btContactSolverInfo;

extern (C++):

import bullet2.LinearMath.btScalar;

public enum btSolverMode : int
{
	SOLVER_RANDMIZE_ORDER = 1,
	SOLVER_FRICTION_SEPARATE = 2,
	SOLVER_USE_WARMSTARTING = 4,
	SOLVER_USE_2_FRICTION_DIRECTIONS = 16,
	SOLVER_ENABLE_FRICTION_DIRECTION_CACHING = 32,
	SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION = 64,
	SOLVER_CACHE_FRIENDLY = 128,
	SOLVER_SIMD = 256,
	SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS = 512,
	SOLVER_ALLOW_ZERO_LENGTH_FRICTION_DIRECTIONS = 1024,
	SOLVER_DISABLE_IMPLICIT_CONE_FRICTION = 2048
};

struct btContactSolverInfoData
{
	btScalar m_tau;
	btScalar m_damping;  //global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
	btScalar m_friction;
	btScalar m_timeStep;
	btScalar m_restitution;
	int m_numIterations;
	btScalar m_maxErrorReduction;
	btScalar m_sor;          //successive over-relaxation term
	btScalar m_erp;          //error reduction for non-contact constraints
	btScalar m_erp2;         //error reduction for contact constraints
	btScalar m_globalCfm;    //constraint force mixing for contacts and non-contacts
	btScalar m_frictionERP;  //error reduction for friction constraints
	btScalar m_frictionCFM;  //constraint force mixing for friction constraints

	int m_splitImpulse;
	btScalar m_splitImpulsePenetrationThreshold;
	btScalar m_splitImpulseTurnErp;
	btScalar m_linearSlop;
	btScalar m_warmstartingFactor;

	btSolverMode m_solverMode;
	int m_restingContactRestitutionThreshold;
	int m_minimumSolverBatchSize;
	btScalar m_maxGyroscopicForce;
	btScalar m_singleAxisRollingFrictionThreshold;
	btScalar m_leastSquaresResidualThreshold;
	btScalar m_restitutionVelocityThreshold;
	bool m_jointFeedbackInWorldSpace;
	bool m_jointFeedbackInJointFrame;
	int m_reportSolverAnalytics;
};

struct btContactSolverInfo// : public btContactSolverInfoData
{
    btContactSolverInfoData base_ = btContactSolverInfoData
    (
        /*m_tau = */ (0.6),
		/*m_damping = */(1.0),
		/*m_fiction = */(0.3),
		/*m_timeStep = */(1.0f / 60.0f),
		/*m_restitution = */(0.),
        /*m_numIterations = */10,
		/*m_maxErrorReduction = */(20.),
		/*m_sor = */(1.),
		/*m_erp = */(0.2),
		/*m_erp2 = */(0.2),
		/*m_globalCfm = */(0.),
		/*m_frictionERP = */(0.2),  //positional friction 'anchors' are disabled by default
		/*m_frictionCFM = */(0.),

		/*m_splitImpulse = */true,
		/*m_splitImpulsePenetrationThreshold = */-0.04f,
		/*m_splitImpulseTurnErp = */0.1f,
		/*m_linearSlop = */(0.0),
		/*m_warmstartingFactor = */(0.85),
		/*m_solverMode = *///SOLVER_USE_WARMSTARTING |  SOLVER_SIMD | SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION|SOLVER_USE_2_FRICTION_DIRECTIONS|SOLVER_ENABLE_FRICTION_DIRECTION_CACHING,// | SOLVER_RANDMIZE_ORDER,
		/*m_solverMode = */btSolverMode.SOLVER_USE_WARMSTARTING | btSolverMode.SOLVER_SIMD,  // | SOLVER_RANDMIZE_ORDER,
		/*m_restingContactRestitutionThreshold = */2,              //unused as of 2.81
		/*m_minimumSolverBatchSize = */128,                        //try to combine islands until the amount of constraints reaches this limit
		/*m_maxGyroscopicForce = */100.0f,                          ///it is only used for 'explicit' version of gyroscopic force
		/*m_singleAxisRollingFrictionThreshold = */1e30f,          ///if the velocity is above this threshold, it will use a single constraint row (axis), otherwise 3 rows.
		/*m_leastSquaresResidualThreshold = */0.0f,
		/*m_restitutionVelocityThreshold = */0.2f,  //if the relative velocity is below this threshold, there is zero restitution
		/*m_jointFeedbackInWorldSpace = */false,
		/*m_jointFeedbackInJointFrame = */false,
		/*m_reportSolverAnalytics = */0,
    );
    alias base_ this;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btContactSolverInfoDoubleData
{
	double m_tau;
	double m_damping;  //global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
	double m_friction;
	double m_timeStep;
	double m_restitution;
	double m_maxErrorReduction;
	double m_sor;
	double m_erp;        //used as Baumgarte factor
	double m_erp2;       //used in Split Impulse
	double m_globalCfm;  //constraint force mixing
	double m_splitImpulsePenetrationThreshold;
	double m_splitImpulseTurnErp;
	double m_linearSlop;
	double m_warmstartingFactor;
	double m_maxGyroscopicForce;  ///it is only used for 'explicit' version of gyroscopic force
	double m_singleAxisRollingFrictionThreshold;

	int m_numIterations;
	int m_solverMode;
	int m_restingContactRestitutionThreshold;
	int m_minimumSolverBatchSize;
	int m_splitImpulse;
	char[4] m_padding;
};
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btContactSolverInfoFloatData
{
	float m_tau;
	float m_damping;  //global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
	float m_friction;
	float m_timeStep;

	float m_restitution;
	float m_maxErrorReduction;
	float m_sor;
	float m_erp;  //used as Baumgarte factor

	float m_erp2;       //used in Split Impulse
	float m_globalCfm;  //constraint force mixing
	float m_splitImpulsePenetrationThreshold;
	float m_splitImpulseTurnErp;

	float m_linearSlop;
	float m_warmstartingFactor;
	float m_maxGyroscopicForce;
	float m_singleAxisRollingFrictionThreshold;

	int m_numIterations;
	int m_solverMode;
	int m_restingContactRestitutionThreshold;
	int m_minimumSolverBatchSize;

	int m_splitImpulse;
	char[4] m_padding;
};
