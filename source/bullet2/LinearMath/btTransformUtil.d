module bullet2.LinearMath.btTransformUtil;

extern (C++):

import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btQuaternion;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btMatrix3x3;

//#define ANGULAR_MOTION_THRESHOLD btScalar(0.5) * SIMD_HALF_PI
enum ANGULAR_MOTION_THRESHOLD = 0.5 * SIMD_HALF_PI;

/*SIMD_FORCE_INLINE*/ btVector3 btAabbSupport(const ref btVector3 halfExtents, const ref btVector3 supportDir)
{
	return btVector3(supportDir.x() < btScalar(0.0) ? -halfExtents.x() : halfExtents.x(),
					 supportDir.y() < btScalar(0.0) ? -halfExtents.y() : halfExtents.y(),
					 supportDir.z() < btScalar(0.0) ? -halfExtents.z() : halfExtents.z());
}

/// Utils related to temporal transforms
class btTransformUtil
{
public:
	static void integrateTransform(btTransform curTrans, btVector3 linvel, btVector3 angvel, btScalar timeStep, ref btTransform predictedTransform)
	{
		predictedTransform.setOrigin(curTrans.getOrigin() + linvel * timeStep);
		//	#define QUATERNION_DERIVATIVE
/+#ifdef QUATERNION_DERIVATIVE
		btQuaternion predictedOrn = curTrans.getRotation();
		predictedOrn += (angvel * predictedOrn) * (timeStep * btScalar(0.5));
		predictedOrn.safeNormalize();
#else+/
		//Exponential map
		//google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

		btVector3 axis;
		btScalar fAngle2 = angvel.length2();
		btScalar fAngle = 0;
		if (fAngle2 > SIMD_EPSILON)
		{
			fAngle = btSqrt(fAngle2);
		}

		//limit the angular motion
		if (fAngle * timeStep > ANGULAR_MOTION_THRESHOLD)
		{
			fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
		}

		if (fAngle < btScalar(0.001))
		{
			// use Taylor's expansions of sync function
			axis = angvel * (btScalar(0.5) * timeStep - (timeStep * timeStep * timeStep) * (btScalar(0.020833333333)) * fAngle * fAngle);
		}
		else
		{
			// sync(fAngle) = sin(c*fAngle)/t
			axis = angvel * (btSin(btScalar(0.5) * fAngle * timeStep) / fAngle);
		}
		btQuaternion dorn = btQuaternion(axis.x(), axis.y(), axis.z(), btCos(fAngle * timeStep * btScalar(0.5)));
		btQuaternion orn0 = curTrans.getRotation();

		btQuaternion predictedOrn = dorn * orn0;
		predictedOrn.safeNormalize();
//#endif
		if (predictedOrn.length2() > SIMD_EPSILON)
		{
			predictedTransform.setRotation(predictedOrn);
		}
		else
		{
			auto b = curTrans.getBasis();
			predictedTransform.setBasis(b);
		}
	}

	static void calculateVelocityQuaternion(const ref btVector3 pos0, const ref btVector3 pos1, const ref btQuaternion orn0, const ref btQuaternion orn1, btScalar timeStep, ref btVector3 linVel, ref btVector3 angVel)
	{
		linVel = (pos1 - pos0) / timeStep;
		btVector3 axis;
		btScalar angle;
		if (orn0 != orn1)
		{
			calculateDiffAxisAngleQuaternion(orn0, orn1, axis, angle);
			angVel = axis * angle / timeStep;
		}
		else
		{
			angVel.setValue(0, 0, 0);
		}
	}

	static void calculateDiffAxisAngleQuaternion(const ref btQuaternion orn0, const ref btQuaternion orn1a, ref btVector3 axis, ref btScalar angle)
	{
		btQuaternion orn1 = orn0.nearest(orn1a);
		btQuaternion dorn = orn1 * orn0.inverse();
		angle = dorn.getAngle();
		axis = btVector3(dorn.x(), dorn.y(), dorn.z());
		axis[3] = /*btScalar*/(0.0);
		//check for axis length
		btScalar len = axis.length2();
		if (len < SIMD_EPSILON * SIMD_EPSILON)
			axis = btVector3(btScalar(1.), btScalar(0.), btScalar(0.));
		else
			axis /= btSqrt(len);
	}

	static void calculateVelocity(const ref btTransform transform0, const ref btTransform transform1, btScalar timeStep, ref btVector3 linVel, ref btVector3 angVel)
	{
		linVel = (transform1.getOrigin() - transform0.getOrigin()) / timeStep;
		btVector3 axis;
		btScalar angle;
		calculateDiffAxisAngle(transform0, transform1, axis, angle);
		angVel = axis * angle / timeStep;
	}

	static void calculateDiffAxisAngle(const ref btTransform transform0, const ref btTransform transform1, ref btVector3 axis, ref btScalar angle)
	{
		btMatrix3x3 dmat = transform1.getBasis() * transform0.getBasis().inverse();
		btQuaternion dorn;
		dmat.getRotation(dorn);

		///floating point inaccuracy can lead to w component > 1..., which breaks
		dorn.normalize();

		angle = dorn.getAngle();
		axis = btVector3(dorn.x(), dorn.y(), dorn.z());
		axis[3] = /*btScalar*/(0.0);
		//check for axis length
		btScalar len = axis.length2();
		if (len < SIMD_EPSILON * SIMD_EPSILON)
			axis = btVector3(btScalar(1.), btScalar(0.), btScalar(0.));
		else
			axis /= btSqrt(len);
	}
};

///The btConvexSeparatingDistanceUtil can help speed up convex collision detection
///by conservatively updating a cached separating distance/vector instead of re-calculating the closest distance
extern (C++, class)
struct btConvexSeparatingDistanceUtil
{
	btQuaternion m_ornA;
	btQuaternion m_ornB;
	btVector3 m_posA;
	btVector3 m_posB;

	btVector3 m_separatingNormal;

	btScalar m_boundingRadiusA;
	btScalar m_boundingRadiusB;
	btScalar m_separatingDistance;

public:
	this(btScalar boundingRadiusA, btScalar boundingRadiusB)
	{
        m_boundingRadiusA = boundingRadiusA;
        m_boundingRadiusB = boundingRadiusB;
        m_separatingDistance = 0.0f;
	}

	btScalar getConservativeSeparatingDistance()
	{
		return m_separatingDistance;
	}

	void updateSeparatingDistance(const ref btTransform transA, const ref btTransform transB)
	{
		btVector3 toPosA = transA.getOrigin();
		btVector3 toPosB = transB.getOrigin();
		btQuaternion toOrnA = transA.getRotation();
		btQuaternion toOrnB = transB.getRotation();

		if (m_separatingDistance > 0.0f)
		{
			btVector3 linVelA, angVelA, linVelB, angVelB;
			btTransformUtil.calculateVelocityQuaternion(m_posA, toPosA, m_ornA, toOrnA, 1.0, linVelA, angVelA);
			btTransformUtil.calculateVelocityQuaternion(m_posB, toPosB, m_ornB, toOrnB, 1.0, linVelB, angVelB);
			btScalar maxAngularProjectedVelocity = angVelA.length() * m_boundingRadiusA + angVelB.length() * m_boundingRadiusB;
			btVector3 relLinVel = (linVelB - linVelA);
			btScalar relLinVelocLength = relLinVel.dot(m_separatingNormal);
			if (relLinVelocLength < 0.0f)
			{
				relLinVelocLength = 0.0f;
			}

			btScalar projectedMotion = maxAngularProjectedVelocity + relLinVelocLength;
			m_separatingDistance -= projectedMotion;
		}

		m_posA = toPosA;
		m_posB = toPosB;
		m_ornA = toOrnA;
		m_ornB = toOrnB;
	}

	void initSeparatingDistance(const ref btVector3 separatingVector, btScalar separatingDistance, const ref btTransform transA, const ref btTransform transB)
	{
		m_separatingDistance = separatingDistance;

		if (m_separatingDistance > 0.0f)
		{
			m_separatingNormal = separatingVector;

			btVector3 toPosA = transA.getOrigin();
			btVector3 toPosB = transB.getOrigin();
			btQuaternion toOrnA = transA.getRotation();
			btQuaternion toOrnB = transB.getRotation();
			m_posA = toPosA;
			m_posB = toPosB;
			m_ornA = toOrnA;
			m_ornB = toOrnB;
		}
	}
};
