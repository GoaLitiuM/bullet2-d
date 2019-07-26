module bullet2.BulletDynamics.ConstraintSolver.btSolverBody;

extern (C++):

import bullet2.BulletDynamics.Dynamics.btRigidBody;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btMatrix3x3;
import bullet2.LinearMath.btAlignedAllocator;
import bullet2.LinearMath.btTransformUtil;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btScalar;

///Until we get other contributions, only use SIMD on Windows, when using Visual Studio 2008 or later, and not double precision
/+#ifdef BT_USE_SSE
#define USE_SIMD 1
#endif  //

#ifdef USE_SIMD

struct btSimdScalar
{
	/*SIMD_FORCE_INLINE*/ btSimdScalar()
	{
	}

	/*SIMD_FORCE_INLINE*/ btSimdScalar(float fl)
		: m_vec128(_mm_set1_ps(fl))
	{
	}

	/*SIMD_FORCE_INLINE*/ btSimdScalar(__m128 v128)
		: m_vec128(v128)
	{
	}
	union {
		__m128 m_vec128;
		float m_floats[4];
		int m_ints[4];
		btScalar m_unusedPadding;
	};
	/*SIMD_FORCE_INLINE*/ __m128 get128()
	{
		return m_vec128;
	}

	/*SIMD_FORCE_INLINE*/ const __m128 get128() const
	{
		return m_vec128;
	}

	/*SIMD_FORCE_INLINE*/ void set128(__m128 v128)
	{
		m_vec128 = v128;
	}

	/*SIMD_FORCE_INLINE*/ operator __m128()
	{
		return m_vec128;
	}
	/*SIMD_FORCE_INLINE*/ operator const __m128() const
	{
		return m_vec128;
	}

	/*SIMD_FORCE_INLINE*/ operator float() const
	{
		return m_floats[0];
	}
};

///@brief Return the elementwise product of two btSimdScalar
/*SIMD_FORCE_INLINE*/ btSimdScalar
operator*(const btSimdScalar& v1, const btSimdScalar& v2)
{
	return btSimdScalar(_mm_mul_ps(v1.get128(), v2.get128()));
}

///@brief Return the elementwise product of two btSimdScalar
/*SIMD_FORCE_INLINE*/ btSimdScalar
operator+(const btSimdScalar& v1, const btSimdScalar& v2)
{
	return btSimdScalar(_mm_add_ps(v1.get128(), v2.get128()));
}

#else+/
//#define btSimdScalar btScalar
alias btSimdScalar = btScalar;
//#endif

///The btSolverBody is an internal datastructure for the constraint solver. Only necessary data is packed to increase cache coherence/performance.
//ATTRIBUTE_ALIGNED16(struct)
align(16) struct btSolverBody
{
	//BT_DECLARE_ALIGNED_ALLOCATOR();
	btTransform m_worldTransform;
	btVector3 m_deltaLinearVelocity;
	btVector3 m_deltaAngularVelocity;
	btVector3 m_angularFactor;
	btVector3 m_linearFactor;
	btVector3 m_invMass;
	btVector3 m_pushVelocity;
	btVector3 m_turnVelocity;
	btVector3 m_linearVelocity;
	btVector3 m_angularVelocity;
	btVector3 m_externalForceImpulse;
	btVector3 m_externalTorqueImpulse;

	btRigidBody* m_originalBody;
	void setWorldTransform(const ref btTransform worldTransform)
	{
		m_worldTransform = worldTransform;
	}

	ref btTransform getWorldTransform() return //const
	{
		return m_worldTransform;
	}

	/*SIMD_FORCE_INLINE*/ void getVelocityInLocalPointNoDelta(const ref btVector3 rel_pos, ref btVector3 velocity) //const
	{
		if (m_originalBody)
			velocity = m_linearVelocity + m_externalForceImpulse + (m_angularVelocity + m_externalTorqueImpulse).cross(rel_pos);
		else
			velocity.setValue(0, 0, 0);
	}

	/*SIMD_FORCE_INLINE*/ void getVelocityInLocalPointObsolete(const ref btVector3 rel_pos, ref btVector3 velocity) //const
	{
		if (m_originalBody)
			velocity = m_linearVelocity + m_deltaLinearVelocity + (m_angularVelocity + m_deltaAngularVelocity).cross(rel_pos);
		else
			velocity.setValue(0, 0, 0);
	}

	/*SIMD_FORCE_INLINE*/ void getAngularVelocity(ref btVector3 angVel) //const
	{
		if (m_originalBody)
			angVel = m_angularVelocity + m_deltaAngularVelocity;
		else
			angVel.setValue(0, 0, 0);
	}

	//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	/*SIMD_FORCE_INLINE*/ void applyImpulse(const ref btVector3 linearComponent, const ref btVector3 angularComponent, const btScalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_deltaLinearVelocity += linearComponent * impulseMagnitude * m_linearFactor;
			m_deltaAngularVelocity += angularComponent * (impulseMagnitude * m_angularFactor);
		}
	}

	/*SIMD_FORCE_INLINE*/ void internalApplyPushImpulse(const ref btVector3 linearComponent, const ref btVector3 angularComponent, btScalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_pushVelocity += linearComponent * impulseMagnitude * m_linearFactor;
			m_turnVelocity += angularComponent * (impulseMagnitude * m_angularFactor);
		}
	}

	ref btVector3 getDeltaLinearVelocity() return //const
	{
		return m_deltaLinearVelocity;
	}

	ref btVector3 getDeltaAngularVelocity() return //const
	{
		return m_deltaAngularVelocity;
	}

	ref btVector3 getPushVelocity() return //const
	{
		return m_pushVelocity;
	}

	ref btVector3 getTurnVelocity() return //const
	{
		return m_turnVelocity;
	}

	////////////////////////////////////////////////
	///some internal methods, don't use them

	ref btVector3 internalGetDeltaLinearVelocity() return
	{
		return m_deltaLinearVelocity;
	}

	ref btVector3 internalGetDeltaAngularVelocity() return
	{
		return m_deltaAngularVelocity;
	}

	ref btVector3 internalGetAngularFactor() return //const
	{
		return m_angularFactor;
	}

	ref btVector3 internalGetInvMass() return //const
	{
		return m_invMass;
	}

	void internalSetInvMass(const ref btVector3 invMass)
	{
		m_invMass = invMass;
	}

	ref btVector3 internalGetPushVelocity() return
	{
		return m_pushVelocity;
	}

	ref btVector3 internalGetTurnVelocity() return
	{
		return m_turnVelocity;
	}

	/*SIMD_FORCE_INLINE*/ void internalGetVelocityInLocalPointObsolete(const ref btVector3 rel_pos, ref btVector3 velocity) //const
	{
		velocity = m_linearVelocity + m_deltaLinearVelocity + (m_angularVelocity + m_deltaAngularVelocity).cross(rel_pos);
	}

	/*SIMD_FORCE_INLINE*/ void internalGetAngularVelocity(ref btVector3 angVel) //const
	{
		angVel = m_angularVelocity + m_deltaAngularVelocity;
	}

	//Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
	/*SIMD_FORCE_INLINE*/ void internalApplyImpulse(const ref btVector3 linearComponent, const ref btVector3 angularComponent, const btScalar impulseMagnitude)
	{
		if (m_originalBody)
		{
			m_deltaLinearVelocity += linearComponent * impulseMagnitude * m_linearFactor;
			m_deltaAngularVelocity += angularComponent * (impulseMagnitude * m_angularFactor);
		}
	}

	void writebackVelocity()
	{
		if (m_originalBody)
		{
			m_linearVelocity += m_deltaLinearVelocity;
			m_angularVelocity += m_deltaAngularVelocity;

			//m_originalBody->setCompanionId(-1);
		}
	}

	void writebackVelocityAndTransform(btScalar timeStep, btScalar splitImpulseTurnErp)
	{
		//(void)timeStep;
		if (m_originalBody)
		{
			m_linearVelocity += m_deltaLinearVelocity;
			m_angularVelocity += m_deltaAngularVelocity;

			//correct the position/orientation based on push/turn recovery
			btTransform newTransform;
			if (m_pushVelocity[0] != 0.0f || m_pushVelocity[1] != 0 || m_pushVelocity[2] != 0 || m_turnVelocity[0] != 0.0f || m_turnVelocity[1] != 0 || m_turnVelocity[2] != 0)
			{
				//	btQuaternion orn = m_worldTransform.getRotation();
				btTransformUtil.integrateTransform(m_worldTransform, m_pushVelocity, m_turnVelocity * splitImpulseTurnErp, timeStep, newTransform);
				m_worldTransform = newTransform;
			}
			//m_worldTransform.setRotation(orn);
			//m_originalBody->setCompanionId(-1);
		}
	}
};
