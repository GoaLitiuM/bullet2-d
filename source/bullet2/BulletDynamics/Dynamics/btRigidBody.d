module bullet2.BulletDynamics.Dynamics.btRigidBody;

extern (C++):

import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btMatrix3x3;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btMotionState;
import bullet2.LinearMath.btQuaternion;
import bullet2.LinearMath.btSerializer;

import bullet2.BulletDynamics.ConstraintSolver.btTypedConstraint;
import bullet2.BulletDynamics.ConstraintSolver.btSolverBody;

extern btScalar gDeactivationTime;
extern bool gDisableDeactivation;

version (BT_USE_DOUBLE_PRECISION)
{
    alias btRigidBodyData = btRigidBodyDoubleData;
    enum btRigidBodyDataName = "btRigidBodyDoubleData";
}
else
{
    alias btRigidBodyData = btRigidBodyFloatData;
    enum btRigidBodyDataName = "btRigidBodyFloatData";
}

enum btRigidBodyFlags
{
	BT_DISABLE_WORLD_GRAVITY = 1,
	///BT_ENABLE_GYROPSCOPIC_FORCE flags is enabled by default in Bullet 2.83 and onwards.
	///and it BT_ENABLE_GYROPSCOPIC_FORCE becomes equivalent to BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY
	///See Demos/GyroscopicDemo and computeGyroscopicImpulseImplicit
	BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT = 2,
	BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD = 4,
	BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY = 8,
	BT_ENABLE_GYROPSCOPIC_FORCE = BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY,
};

///The btRigidBody is the main class for rigid body objects. It is derived from btCollisionObject, so it keeps a pointer to a btCollisionShape.
///It is recommended for performance and memory use to share btCollisionShape objects whenever possible.
///There are 3 types of rigid bodies:
///- A) Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.
///- B) Fixed objects with zero mass. They are not moving (basically collision objects)
///- C) Kinematic objects, which are objects without mass, but the user can move them. There is one-way interaction, and Bullet calculates a velocity based on the timestep and previous and current world transform.
///Bullet automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a given time.
///Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection impact (to allow active objects to activate/wake up sleeping objects)
class btRigidBody : btCollisionObject
{
	btMatrix3x3 m_invInertiaTensorWorld;
	btVector3 m_linearVelocity;
	btVector3 m_angularVelocity;
	btScalar m_inverseMass;
	btVector3 m_linearFactor;

	btVector3 m_gravity;
	btVector3 m_gravity_acceleration;
	btVector3 m_invInertiaLocal;
	btVector3 m_totalForce;
	btVector3 m_totalTorque;

	btScalar m_linearDamping;
	btScalar m_angularDamping;

	bool m_additionalDamping;
	btScalar m_additionalDampingFactor;
	btScalar m_additionalLinearDampingThresholdSqr;
	btScalar m_additionalAngularDampingThresholdSqr;
	btScalar m_additionalAngularDampingFactor;

	btScalar m_linearSleepingThreshold;
	btScalar m_angularSleepingThreshold;

	//m_optionalMotionState allows to automatic synchronize the world transform for active objects
	btMotionState m_optionalMotionState;

	//keep track of typed constraints referencing this rigid body, to disable collision between linked bodies
	btAlignedObjectArray!(btTypedConstraint*) m_constraintRefs;

	int m_rigidbodyFlags;

	int m_debugBodyId;

protected:
	//ATTRIBUTE_ALIGNED16(btVector3 m_deltaLinearVelocity);
    align(16) btVector3 m_deltaLinearVelocity;

	btVector3 m_deltaAngularVelocity;
	btVector3 m_angularFactor;
	btVector3 m_invMass;
	btVector3 m_pushVelocity;
	btVector3 m_turnVelocity;

public:
	///The btRigidBodyConstructionInfo structure provides information to create a rigid body. Setting mass to zero creates a fixed (non-dynamic) rigid body.
	///For dynamic objects, you can use the collision shape to approximate the local inertia tensor, otherwise use the zero vector (default argument)
	///You can use the motion state to synchronize the world transform between physics and graphics objects.
	///And if the motion state is provided, the rigid body will initialize its initial world transform from the motion state,
	///m_startWorldTransform is only used when you don't provide a motion state.
	struct btRigidBodyConstructionInfo
	{
		btScalar m_mass;

		///When a motionState is provided, the rigid body will initialize its world transform from the motion state
		///In this case, m_startWorldTransform is ignored.
		btMotionState m_motionState;
		btTransform m_startWorldTransform;

		btCollisionShape m_collisionShape;
		btVector3 m_localInertia;
		btScalar m_linearDamping;
		btScalar m_angularDamping;

		///best simulation results when friction is non-zero
		btScalar m_friction;
		///the m_rollingFriction prevents rounded shapes, such as spheres, cylinders and capsules from rolling forever.
		///See Bullet/Demos/RollingFrictionDemo for usage
		btScalar m_rollingFriction;
		btScalar m_spinningFriction;  //torsional friction around contact normal

		///best simulation results using zero restitution.
		btScalar m_restitution;

		btScalar m_linearSleepingThreshold;
		btScalar m_angularSleepingThreshold;

		//Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		//Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
		bool m_additionalDamping;
		btScalar m_additionalDampingFactor;
		btScalar m_additionalLinearDampingThresholdSqr;
		btScalar m_additionalAngularDampingThresholdSqr;
		btScalar m_additionalAngularDampingFactor;

		this(btScalar mass, btMotionState motionState, btCollisionShape collisionShape, /*const ref*/ btVector3 localInertia = btVector3(0, 0, 0))
		{
			m_startWorldTransform.setIdentity();

            m_mass = mass;
            m_motionState = motionState;
            m_collisionShape = collisionShape;
            m_localInertia = localInertia;
            m_linearDamping = 0.0;
            m_angularDamping = 0.0;
            m_friction = 0.5;
            m_rollingFriction = 0;
            m_spinningFriction = 0;
            m_restitution = 0.0;
            m_linearSleepingThreshold = 0.8;
            m_angularSleepingThreshold = 1.0f;
            m_additionalDamping = false;
            m_additionalDampingFactor = 0.005;
            m_additionalLinearDampingThresholdSqr = 0.01;
            m_additionalAngularDampingThresholdSqr = 0.01;
            m_additionalAngularDampingFactor = 0.01;
		}
	};

	///btRigidBody constructor using construction info
	final this(const ref btRigidBodyConstructionInfo constructionInfo);

	///btRigidBody constructor for backwards compatibility.
	///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
	final this(btScalar mass, btMotionState motionState, btCollisionShape collisionShape, const ref btVector3 localInertia/* = btVector3(0, 0, 0)*/);

	/*virtual*/ ~this()
	{
		//No constraints should point to this rigidbody
		//Remove constraints from the dynamics world before you delete the related rigidbodies.
		btAssert(m_constraintRefs.size() == 0);
	}

protected:
	///setupRigidBody is only used internally by the constructor
	final void setupRigidBody(const ref btRigidBodyConstructionInfo constructionInfo);

public:
	final void proceedToTransform(const ref btTransform newTrans);

	///to keep collision detection and dynamics separate we don't store a rigidbody pointer
	///but a rigidbody is derived from btCollisionObject, so we can safely perform an upcast
	final static const(btRigidBody) upcast(const btCollisionObject colObj)
	{
		if (colObj.getInternalType() & btCollisionObject.CollisionObjectTypes.CO_RIGID_BODY)
			return cast(const(btRigidBody))colObj;
		return null;
	}
	final static btRigidBody upcast(btCollisionObject colObj)
	{
		if (colObj.getInternalType() & btCollisionObject.CollisionObjectTypes.CO_RIGID_BODY)
			return cast(btRigidBody)colObj;
		return null;
	}

	/// continuous collision detection needs prediction
	final void predictIntegratedTransform(btScalar step, ref btTransform predictedTransform);

	final void saveKinematicState(btScalar step);

	final void applyGravity();

	final void setGravity(const ref btVector3 acceleration);

	final ref btVector3 getGravity() //const
	{
		return m_gravity_acceleration;
	}

	final void setDamping(btScalar lin_damping, btScalar ang_damping);

	final btScalar getLinearDamping() const
	{
		return m_linearDamping;
	}

	final btScalar getAngularDamping() const
	{
		return m_angularDamping;
	}

	final btScalar getLinearSleepingThreshold() const
	{
		return m_linearSleepingThreshold;
	}

	final btScalar getAngularSleepingThreshold() const
	{
		return m_angularSleepingThreshold;
	}

	final void applyDamping(btScalar timeStep);

	final override /*SIMD_FORCE_INLINE*/ const(btCollisionShape) getCollisionShape() const
	{
		return m_collisionShape;
	}

	final override /*SIMD_FORCE_INLINE*/ btCollisionShape getCollisionShape()
	{
		return m_collisionShape;
	}

	final void setMassProps(btScalar mass, const ref btVector3 inertia);

	final ref btVector3 getLinearFactor()// const
	{
		return m_linearFactor;
	}
	final void setLinearFactor(const ref btVector3 linearFactor)
	{
		m_linearFactor = linearFactor;
		m_invMass = m_linearFactor * m_inverseMass;
	}
	final btScalar getInvMass() const { return m_inverseMass; }

	final ref btMatrix3x3 getInvInertiaTensorWorld() //const
	{
		return m_invInertiaTensorWorld;
	}
	ref const(btMatrix3x3) getInvInertiaTensorWorld() const
	{
		return m_invInertiaTensorWorld;
	}

	final void integrateVelocities(btScalar step);

	final void setCenterOfMassTransform(const ref btTransform xform);

	final void applyCentralForce(ref btVector3 force)
	{
		m_totalForce += force * m_linearFactor;
	}

	final ref btVector3 getTotalForce() // const
	{
		return m_totalForce;
	};

	final ref btVector3 getTotalTorque() // const
	{
		return m_totalTorque;
	};

	final ref btVector3 getInvInertiaDiagLocal() //const
	{
		return m_invInertiaLocal;
	};

	final void setInvInertiaDiagLocal(const ref btVector3 diagInvInertia)
	{
		m_invInertiaLocal = diagInvInertia;
	}

	final void setSleepingThresholds(btScalar linear, btScalar angular)
	{
		m_linearSleepingThreshold = linear;
		m_angularSleepingThreshold = angular;
	}

	final void applyTorque(btVector3 torque)
	{
		m_totalTorque += torque * m_angularFactor;
	}

	final void applyForce(btVector3 force, btVector3 rel_pos)
	{
		applyCentralForce(force);
		applyTorque(rel_pos.cross(force * m_linearFactor));
	}

	final void applyCentralImpulse(btVector3 impulse)
	{
		m_linearVelocity += impulse * m_linearFactor * m_inverseMass;
	}

	final void applyTorqueImpulse(btVector3 torque)
	{
		m_angularVelocity += m_invInertiaTensorWorld * torque * m_angularFactor;
	}

	final void applyImpulse(btVector3 impulse, btVector3 rel_pos)
	{
		if (m_inverseMass != btScalar(0.))
		{
			applyCentralImpulse(impulse);
			//if (m_angularFactor)
			{
				applyTorqueImpulse(rel_pos.cross(impulse * m_linearFactor));
			}
		}
	}

	final void clearForces()
	{
		m_totalForce.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
		m_totalTorque.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	}

	final void updateInertiaTensor();

	final auto ref btVector3 getCenterOfMassPosition() const
	{
		return m_worldTransform.getOrigin();
	}
	final btQuaternion getOrientation() const;

	final auto ref btTransform getCenterOfMassTransform() const
	{
		return m_worldTransform;
	}
	auto ref btVector3 getLinearVelocity() const
	{
		return m_linearVelocity;
	}
	auto ref btVector3 getAngularVelocity() const
	{
		return m_angularVelocity;
	}

	final /*inline*/ void setLinearVelocity(const ref btVector3 lin_vel)
	{
		m_updateRevision++;
		m_linearVelocity = lin_vel;
	}

	final /*inline*/ void setAngularVelocity(const ref btVector3 ang_vel)
	{
		m_updateRevision++;
		m_angularVelocity = ang_vel;
	}

	final btVector3 getVelocityInLocalPoint(const ref btVector3 rel_pos) //const
	{
		//we also calculate lin/ang velocity for kinematic objects
		return m_linearVelocity + m_angularVelocity.cross(rel_pos);

		//for kinematic objects, we could also use use:
		//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	}

	final void translate(const ref btVector3 v)
	{
		m_worldTransform.getOrigin() += v;
	}

	final void getAabb(ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	final /*SIMD_FORCE_INLINE*/ btScalar computeImpulseDenominator(ref btVector3 pos, ref btVector3 normal) //const
	{
		btVector3 r0 = pos - getCenterOfMassPosition();

		btVector3 c0 = (r0).cross(normal);

		//btVector3 vec = (c0 * getInvInertiaTensorWorld()).cross(r0);
		auto m = getInvInertiaTensorWorld();
		btVector3 vec = (btVector3(m.tdotx(c0), m.tdoty(c0), m.tdotz(c0))).cross(r0);

		return m_inverseMass + normal.dot(vec);
	}

	final /*SIMD_FORCE_INLINE*/ btScalar computeAngularImpulseDenominator(ref btVector3 axis) const
	{
		btVector3 vec = axis * getInvInertiaTensorWorld();
		return axis.dot(vec);
	}

	final /*SIMD_FORCE_INLINE*/ void updateDeactivation(btScalar timeStep)
	{
		if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == DISABLE_DEACTIVATION))
			return;

		if ((getLinearVelocity().length2() < m_linearSleepingThreshold * m_linearSleepingThreshold) &&
			(getAngularVelocity().length2() < m_angularSleepingThreshold * m_angularSleepingThreshold))
		{
			m_deactivationTime += timeStep;
		}
		else
		{
			m_deactivationTime = btScalar(0.);
			setActivationState(0);
		}
	}

	final /*SIMD_FORCE_INLINE*/ bool wantsSleeping()
	{
		if (getActivationState() == DISABLE_DEACTIVATION)
			return false;

		//disable deactivation
		if (gDisableDeactivation || (gDeactivationTime == btScalar(0.)))
			return false;

		if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == WANTS_DEACTIVATION))
			return true;

		if (m_deactivationTime > gDeactivationTime)
		{
			return true;
		}
		return false;
	}

	final const(btBroadphaseProxy) getBroadphaseProxy() const
	{
		return m_broadphaseHandle;
	}
	final btBroadphaseProxy getBroadphaseProxy()
	{
		return m_broadphaseHandle;
	}
	final void setNewBroadphaseProxy(btBroadphaseProxy broadphaseProxy)
	{
		m_broadphaseHandle = broadphaseProxy;
	}

	//btMotionState allows to automatic synchronize the world transform for active objects
	final btMotionState getMotionState()
	{
		return m_optionalMotionState;
	}
	final const(btMotionState) getMotionState() const
	{
		return m_optionalMotionState;
	}
	final void setMotionState(btMotionState motionState)
	{
		m_optionalMotionState = motionState;
		if (m_optionalMotionState)
			motionState.getWorldTransform(m_worldTransform);
	}

	//for experimental overriding of friction/contact solver func
	int m_contactSolverType;
	int m_frictionSolverType;

	final void setAngularFactor(const ref btVector3 angFac)
	{
		m_updateRevision++;
		m_angularFactor = angFac;
	}

	final void setAngularFactor(btScalar angFac)
	{
		m_updateRevision++;
		m_angularFactor.setValue(angFac, angFac, angFac);
	}
	final ref btVector3 getAngularFactor() //const
	{
		return m_angularFactor;
	}

	//is this rigidbody added to a btCollisionWorld/btDynamicsWorld/btBroadphase?
	final bool isInWorld() const
	{
		return (getBroadphaseProxy() !is null);
	}

	final void addConstraintRef(btTypedConstraint* c);
	final void removeConstraintRef(btTypedConstraint* c);

	final btTypedConstraint* getConstraintRef(int index)
	{
		return m_constraintRefs[index];
	}

	final int getNumConstraintRefs() const
	{
		return m_constraintRefs.size();
	}

	final void setFlags(int flags)
	{
		m_rigidbodyFlags = flags;
	}

	final int getFlags() const
	{
		return m_rigidbodyFlags;
	}

	///perform implicit force computation in world space
	final btVector3 computeGyroscopicImpulseImplicit_World(btScalar dt) const;

	///perform implicit force computation in body space (inertial frame)
	final btVector3 computeGyroscopicImpulseImplicit_Body(btScalar step) const;

	///explicit version is best avoided, it gains energy
	final btVector3 computeGyroscopicForceExplicit(btScalar maxGyroscopicForce) const;
	final btVector3 getLocalInertia() const;

	///////////////////////////////////////////////

	/*virtual*/ override int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	pragma(mangle, "?serialize@btRigidBody@@UEBAPEBDPEAXPEAVbtSerializer@@@Z")
	/*virtual*/ override const(char*) serialize(void* dataBuffer, btSerializer serializer) const;

	/*virtual*/ override void serializeSingleObject(btSerializer serializer) const;
};

//@todo add m_optionalMotionState and m_constraintRefs to btRigidBodyData
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btRigidBodyFloatData
{
	btCollisionObjectFloatData m_collisionObjectData;
	btMatrix3x3FloatData m_invInertiaTensorWorld;
	btVector3FloatData m_linearVelocity;
	btVector3FloatData m_angularVelocity;
	btVector3FloatData m_angularFactor;
	btVector3FloatData m_linearFactor;
	btVector3FloatData m_gravity;
	btVector3FloatData m_gravity_acceleration;
	btVector3FloatData m_invInertiaLocal;
	btVector3FloatData m_totalForce;
	btVector3FloatData m_totalTorque;
	float m_inverseMass;
	float m_linearDamping;
	float m_angularDamping;
	float m_additionalDampingFactor;
	float m_additionalLinearDampingThresholdSqr;
	float m_additionalAngularDampingThresholdSqr;
	float m_additionalAngularDampingFactor;
	float m_linearSleepingThreshold;
	float m_angularSleepingThreshold;
	int m_additionalDamping;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btRigidBodyDoubleData
{
	btCollisionObjectDoubleData m_collisionObjectData;
	btMatrix3x3DoubleData m_invInertiaTensorWorld;
	btVector3DoubleData m_linearVelocity;
	btVector3DoubleData m_angularVelocity;
	btVector3DoubleData m_angularFactor;
	btVector3DoubleData m_linearFactor;
	btVector3DoubleData m_gravity;
	btVector3DoubleData m_gravity_acceleration;
	btVector3DoubleData m_invInertiaLocal;
	btVector3DoubleData m_totalForce;
	btVector3DoubleData m_totalTorque;
	double m_inverseMass;
	double m_linearDamping;
	double m_angularDamping;
	double m_additionalDampingFactor;
	double m_additionalLinearDampingThresholdSqr;
	double m_additionalAngularDampingThresholdSqr;
	double m_additionalAngularDampingFactor;
	double m_linearSleepingThreshold;
	double m_angularSleepingThreshold;
	int m_additionalDamping;
	char[4] m_padding;
};
