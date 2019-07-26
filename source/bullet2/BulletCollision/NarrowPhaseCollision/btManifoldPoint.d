module bullet2.BulletCollision.NarrowPhaseCollision.btManifoldPoint;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;

// Don't change following order of parameters
//ATTRIBUTE_ALIGNED16(struct)
align(16) struct btConstraintRow
{
	btScalar[3] m_normal;
	btScalar m_rhs;
	btScalar m_jacDiagInv;
	btScalar m_lowerLimit;
	btScalar m_upperLimit;
	btScalar m_accumImpulse;
};
alias PfxConstraintRow = btConstraintRow;
//#endif  //PFX_USE_FREE_VECTORMATH

enum btContactPointFlags
{
	BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED = 1,
	BT_CONTACT_FLAG_HAS_CONTACT_CFM = 2,
	BT_CONTACT_FLAG_HAS_CONTACT_ERP = 4,
	BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING = 8,
	BT_CONTACT_FLAG_FRICTION_ANCHOR = 16,
};

/// ManifoldContactPoint collects and maintains persistent contactpoints.
/// used to improve stability and performance of rigidbody dynamics response.
extern (C++, class)
struct btManifoldPoint
{
public:
	/*this()
	{
        m_userPersistentData = null;
        m_contactPointFlags = 0;
        m_appliedImpulse = 0.0f;
        m_appliedImpulseLateral1 = 0.0f;
        m_appliedImpulseLateral2 = 0.0f;
        m_contactMotion1 = 0.0f;
        m_contactMotion2 = 0.0f;
        m_contactCFM = 0.0f;
        m_contactERP = 0.0f;
        m_frictionCFM = 0.0f;
        m_lifeTime = 0;
	}*/

	this(ref btVector3 pointA, ref btVector3 pointB,
					ref btVector3 normal,
					btScalar distance)
	{
        m_localPointA = pointA;
        m_localPointB = pointB;
        m_normalWorldOnB = normal;
        m_distance1 = distance;
        m_combinedFriction = btScalar(0.);
        m_combinedRollingFriction = btScalar(0.);
        m_combinedSpinningFriction = btScalar(0.);
        m_combinedRestitution = btScalar(0.);
        m_userPersistentData = null;
        m_contactPointFlags = 0;
        m_appliedImpulse = 0.0f;
        m_appliedImpulseLateral1 = 0.0f;
        m_appliedImpulseLateral2 = 0.0f;
        m_contactMotion1 = 0.0f;
        m_contactMotion2 = 0.0f;
        m_contactCFM = 0.0f;
        m_contactERP = 0.0f;
        m_frictionCFM = 0.0f;
        m_lifeTime = 0;
	}

	btVector3 m_localPointA;
	btVector3 m_localPointB;
	btVector3 m_positionWorldOnB;
	///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
	btVector3 m_positionWorldOnA;
	btVector3 m_normalWorldOnB;

	btScalar m_distance1;
	btScalar m_combinedFriction;
	btScalar m_combinedRollingFriction;   //torsional friction orthogonal to contact normal, useful to make spheres stop rolling forever
	btScalar m_combinedSpinningFriction;  //torsional friction around contact normal, useful for grasping objects
	btScalar m_combinedRestitution;

	//BP mod, store contact triangles.
	int m_partId0;
	int m_partId1;
	int m_index0;
	int m_index1;

	/*mutable*/ void* m_userPersistentData = null;
	//bool			m_lateralFrictionInitialized;
	int m_contactPointFlags = 0;

	btScalar m_appliedImpulse = 0.0f;
	btScalar m_appliedImpulseLateral1 = 0.0f;
	btScalar m_appliedImpulseLateral2 = 0.0f;
	btScalar m_contactMotion1 = 0.0f;
	btScalar m_contactMotion2 = 0.0f;

	union {
		btScalar m_contactCFM = 0.0f;
		btScalar m_combinedContactStiffness1;
	};

	union {
		btScalar m_contactERP = 0.0f;
		btScalar m_combinedContactDamping1;
	};

	btScalar m_frictionCFM = 0.0f;

	int m_lifeTime = 0;  //lifetime of the contactpoint in frames

	btVector3 m_lateralFrictionDir1;
	btVector3 m_lateralFrictionDir2;

	btScalar getDistance() const
	{
		return m_distance1;
	}
	int getLifeTime() const
	{
		return m_lifeTime;
	}

	/*const */ref btVector3 getPositionWorldOnA() return// const
	{
		return m_positionWorldOnA;
		//				return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
	}

	/*const */ref btVector3 getPositionWorldOnB() return// const
	{
		return m_positionWorldOnB;
	}

	void setDistance(btScalar dist)
	{
		m_distance1 = dist;
	}

	///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
	btScalar getAppliedImpulse() const
	{
		return m_appliedImpulse;
	}
};
