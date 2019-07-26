module bullet2.BulletCollision.CollisionDispatch.btCollisionObject;

extern (C++):

import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btAlignedAllocator;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btSerializer;

//island management, m_activationState1
enum ACTIVE_TAG = 1;
enum ISLAND_SLEEPING = 2;
enum WANTS_DEACTIVATION = 3;
enum DISABLE_DEACTIVATION = 4;
enum DISABLE_SIMULATION = 5;

alias btCollisionObjectArray = btAlignedObjectArray!(btCollisionObject);

version (BT_USE_DOUBLE_PRECISION)
{
    alias btCollisionObjectData = btCollisionObjectDoubleData;
    enum btCollisionObjectDataName = "btCollisionObjectDoubleData";
}
else
{
    alias btCollisionObjectData = btCollisionObjectFloatData;
    enum btCollisionObjectDataName = "btCollisionObjectFloatData";
}

/// btCollisionObject can be used to manage collision detection objects.
/// btCollisionObject maintains all information that is needed for a collision detection: Shape, Transform and AABB proxy.
/// They can be added to the btCollisionWorld.
extern (C++, class)
align(16) class btCollisionObject
{
protected:
	btTransform m_worldTransform;

	///m_interpolationWorldTransform is used for CCD and interpolation
	///it can be either previous or future (predicted) transform
	btTransform m_interpolationWorldTransform;
	//those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying velocities)
	//without destroying the continuous interpolated motion (which uses this interpolation velocities)
	btVector3 m_interpolationLinearVelocity;
	btVector3 m_interpolationAngularVelocity;

	btVector3 m_anisotropicFriction;
	int m_hasAnisotropicFriction;
	btScalar m_contactProcessingThreshold;

	btBroadphaseProxy m_broadphaseHandle;
	btCollisionShape m_collisionShape;
	///m_extensionPointer is used by some internal low-level Bullet extensions.
	void* m_extensionPointer;

	///m_rootCollisionShape is temporarily used to store the original collision shape
	///The m_collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
	///If it is NULL, the m_collisionShape is not temporarily replaced.
	btCollisionShape m_rootCollisionShape;

	int m_collisionFlags;

	int m_islandTag1;
	int m_companionId;
	int m_worldArrayIndex;  // index of object in world's collisionObjects array

	/*mutable*/ int m_activationState1;
	/*mutable*/ btScalar m_deactivationTime;

	btScalar m_friction;
	btScalar m_restitution;
	btScalar m_rollingFriction;   //torsional friction orthogonal to contact normal (useful to stop spheres rolling forever)
	btScalar m_spinningFriction;  // torsional friction around the contact normal (useful for grasping)
	btScalar m_contactDamping;
	btScalar m_contactStiffness;

	///m_internalType is reserved to distinguish Bullet's btCollisionObject, btRigidBody, btSoftBody, btGhostObject etc.
	///do not assign your own m_internalType unless you write a new dynamics object class.
	int m_internalType;

	///users can point to their objects, m_userPointer is not used by Bullet, see setUserPointer/getUserPointer

	void* m_userObjectPointer;

	int m_userIndex2;

	int m_userIndex;

	int m_userIndex3;

	///time of impact calculation
	btScalar m_hitFraction;

	///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
	btScalar m_ccdSweptSphereRadius;

	/// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
	btScalar m_ccdMotionThreshold;

	/// If some object should have elaborate collision filtering by sub-classes
	int m_checkCollideWith;

	btAlignedObjectArray!(/*const*/ btCollisionObject*) m_objectsWithoutCollisionCheck;

	///internal update revision number. It will be increased when the object changes. This allows some subsystems to perform lazy evaluation.
	int m_updateRevision;

	btVector3 m_customDebugColorRGB;

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	enum CollisionFlags
	{
		CF_STATIC_OBJECT = 1,
		CF_KINEMATIC_OBJECT = 2,
		CF_NO_CONTACT_RESPONSE = 4,
		CF_CUSTOM_MATERIAL_CALLBACK = 8,  //this allows per-triangle material (friction/restitution)
		CF_CHARACTER_OBJECT = 16,
		CF_DISABLE_VISUALIZE_OBJECT = 32,          //disable debug drawing
		CF_DISABLE_SPU_COLLISION_PROCESSING = 64,  //disable parallel/SPU processing
		CF_HAS_CONTACT_STIFFNESS_DAMPING = 128,
		CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR = 256,
		CF_HAS_FRICTION_ANCHOR = 512,
		CF_HAS_COLLISION_SOUND_TRIGGER = 1024
	};

	enum CollisionObjectTypes
	{
		CO_COLLISION_OBJECT = 1,
		CO_RIGID_BODY = 2,
		///CO_GHOST_OBJECT keeps track of all objects overlapping its AABB and that pass its collision filter
		///It is useful for collision sensors, explosion objects, character controller etc.
		CO_GHOST_OBJECT = 4,
		CO_SOFT_BODY = 8,
		CO_HF_FLUID = 16,
		CO_USER_TYPE = 32,
		CO_FEATHERSTONE_LINK = 64
	};

	enum AnisotropicFrictionFlags
	{
		CF_ANISOTROPIC_FRICTION_DISABLED = 0,
		CF_ANISOTROPIC_FRICTION = 1,
		CF_ANISOTROPIC_ROLLING_FRICTION = 2
	};

	final /*SIMD_FORCE_INLINE*/ bool mergesSimulationIslands() const
	{
		///static objects, kinematic and object without contact response don't merge islands
		return ((m_collisionFlags & (CollisionFlags.CF_STATIC_OBJECT | CollisionFlags.CF_KINEMATIC_OBJECT | CollisionFlags.CF_NO_CONTACT_RESPONSE)) == 0);
	}

	final ref btVector3 getAnisotropicFriction() //const
	{
		return m_anisotropicFriction;
	}
	final void setAnisotropicFriction(ref btVector3 anisotropicFriction, int frictionMode = AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION)
	{
		m_anisotropicFriction = anisotropicFriction;
		bool isUnity = (anisotropicFriction[0] != 1.0f) || (anisotropicFriction[1] != 1.0f) || (anisotropicFriction[2] != 1.0f);
		m_hasAnisotropicFriction = isUnity ? frictionMode : 0;
	}
	final bool hasAnisotropicFriction(int frictionMode = AnisotropicFrictionFlags.CF_ANISOTROPIC_FRICTION) const
	{
		return (m_hasAnisotropicFriction & frictionMode) != 0;
	}

	///the constraint solver can discard solving contacts, if the distance is above this threshold. 0 by default.
	///Note that using contacts with positive distance can improve stability. It increases, however, the chance of colliding with degerate contacts, such as 'interior' triangle edges
	final void setContactProcessingThreshold(btScalar contactProcessingThreshold)
	{
		m_contactProcessingThreshold = contactProcessingThreshold;
	}
	final btScalar getContactProcessingThreshold() const
	{
		return m_contactProcessingThreshold;
	}

	final /*SIMD_FORCE_INLINE*/ bool isStaticObject() const
	{
		return (m_collisionFlags & CollisionFlags.CF_STATIC_OBJECT) != 0;
	}

	final /*SIMD_FORCE_INLINE*/ bool isKinematicObject() const
	{
		return (m_collisionFlags & CollisionFlags.CF_KINEMATIC_OBJECT) != 0;
	}

	final /*SIMD_FORCE_INLINE*/ bool isStaticOrKinematicObject() const
	{
		return (m_collisionFlags & (CollisionFlags.CF_KINEMATIC_OBJECT | CollisionFlags.CF_STATIC_OBJECT)) != 0;
	}

	final /*SIMD_FORCE_INLINE*/ bool hasContactResponse() const
	{
		return (m_collisionFlags & CollisionFlags.CF_NO_CONTACT_RESPONSE) == 0;
	}

	final this();

	/*virtual*/ ~this();

	/*virtual*/ void setCollisionShape(btCollisionShape collisionShape)
	{
		m_updateRevision++;
		m_collisionShape = collisionShape;
		m_rootCollisionShape = collisionShape;
	}

	const(btCollisionShape) getCollisionShape() const
	{
		return m_collisionShape;
	}

	/*SIMD_FORCE_INLINE*/ btCollisionShape getCollisionShape()
	{
		return m_collisionShape;
	}

	final void setIgnoreCollisionCheck(btCollisionObject* co, bool ignoreCollisionCheck)
	{
		if (ignoreCollisionCheck)
		{
			//We don't check for duplicates. Is it ok to leave that up to the user of this API?
			//int index = m_objectsWithoutCollisionCheck.findLinearSearch(co);
			//if (index == m_objectsWithoutCollisionCheck.size())
			//{
			m_objectsWithoutCollisionCheck.push_back(co);
			//}
		}
		else
		{
			m_objectsWithoutCollisionCheck.remove(co);
		}
		m_checkCollideWith = m_objectsWithoutCollisionCheck.size() > 0;
	}

	/*virtual*/ bool checkCollideWithOverride(const btCollisionObject* co) const
	{
		int index = m_objectsWithoutCollisionCheck.findLinearSearch(co);
		if (index < m_objectsWithoutCollisionCheck.size())
		{
			return false;
		}
		return true;
	}

	///Avoid using this internal API call, the extension pointer is used by some Bullet extensions.
	///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
	final void* internalGetExtensionPointer() //const
	{
		return m_extensionPointer;
	}
	///Avoid using this internal API call, the extension pointer is used by some Bullet extensions
	///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
	final void internalSetExtensionPointer(void* pointer)
	{
		m_extensionPointer = pointer;
	}

	final /*SIMD_FORCE_INLINE*/ int getActivationState() const { return m_activationState1; }

	final void setActivationState(int newState) const;

	final void setDeactivationTime(btScalar time)
	{
		m_deactivationTime = time;
	}
	final btScalar getDeactivationTime() const
	{
		return m_deactivationTime;
	}

	final void forceActivationState(int newState) const;

	final void activate(bool forceActivation = false) const;

	final /*SIMD_FORCE_INLINE*/ bool isActive() const
	{
		return ((getActivationState() != ISLAND_SLEEPING) && (getActivationState() != DISABLE_SIMULATION));
	}

	final void setRestitution(btScalar rest)
	{
		m_updateRevision++;
		m_restitution = rest;
	}
	final btScalar getRestitution() const
	{
		return m_restitution;
	}
	final void setFriction(btScalar frict)
	{
		m_updateRevision++;
		m_friction = frict;
	}
	final btScalar getFriction() const
	{
		return m_friction;
	}

	final void setRollingFriction(btScalar frict)
	{
		m_updateRevision++;
		m_rollingFriction = frict;
	}
	final btScalar getRollingFriction() const
	{
		return m_rollingFriction;
	}
	final void setSpinningFriction(btScalar frict)
	{
		m_updateRevision++;
		m_spinningFriction = frict;
	}
	final btScalar getSpinningFriction() const
	{
		return m_spinningFriction;
	}
	final void setContactStiffnessAndDamping(btScalar stiffness, btScalar damping)
	{
		m_updateRevision++;
		m_contactStiffness = stiffness;
		m_contactDamping = damping;

		m_collisionFlags |= CollisionFlags.CF_HAS_CONTACT_STIFFNESS_DAMPING;

		//avoid divisions by zero...
		if (m_contactStiffness < SIMD_EPSILON)
		{
			m_contactStiffness = SIMD_EPSILON;
		}
	}

	final btScalar getContactStiffness() const
	{
		return m_contactStiffness;
	}

	final btScalar getContactDamping() const
	{
		return m_contactDamping;
	}

	///reserved for Bullet internal usage
	final int getInternalType() const
	{
		return m_internalType;
	}

	final ref btTransform getWorldTransform()
	{
		return m_worldTransform;
	}

	final ref const(btTransform) getWorldTransform() const
	{
		return m_worldTransform;
	}

	final void setWorldTransform(const ref btTransform worldTrans)
	{
		m_updateRevision++;
		m_worldTransform = worldTrans;
	}

	final /*SIMD_FORCE_INLINE*/ btBroadphaseProxy getBroadphaseHandle()
	{
		return m_broadphaseHandle;
	}

	final const(btBroadphaseProxy) getBroadphaseHandle() const
	{
		return m_broadphaseHandle;
	}

	final void setBroadphaseHandle(btBroadphaseProxy handle)
	{
		m_broadphaseHandle = handle;
	}

	final ref const(btTransform) getInterpolationWorldTransform() const
	{
		return m_interpolationWorldTransform;
	}

	final ref btTransform getInterpolationWorldTransform()
	{
		return m_interpolationWorldTransform;
	}

	final void setInterpolationWorldTransform(const ref btTransform trans)
	{
		m_updateRevision++;
		m_interpolationWorldTransform = trans;
	}

	final void setInterpolationLinearVelocity(const ref btVector3 linvel)
	{
		m_updateRevision++;
		m_interpolationLinearVelocity = linvel;
	}

	final void setInterpolationAngularVelocity(const ref btVector3 angvel)
	{
		m_updateRevision++;
		m_interpolationAngularVelocity = angvel;
	}

	final /*const*/ ref btVector3 getInterpolationLinearVelocity() //const
	{
		return m_interpolationLinearVelocity;
	}

	final /*const*/ ref btVector3 getInterpolationAngularVelocity() //const
	{
		return m_interpolationAngularVelocity;
	}

	final /*SIMD_FORCE_INLINE*/ int getIslandTag() const
	{
		return m_islandTag1;
	}

	final void setIslandTag(int tag)
	{
		m_islandTag1 = tag;
	}

	final /*SIMD_FORCE_INLINE*/ int getCompanionId() const
	{
		return m_companionId;
	}

	final void setCompanionId(int id)
	{
		m_companionId = id;
	}

	final /*SIMD_FORCE_INLINE*/ int getWorldArrayIndex() const
	{
		return m_worldArrayIndex;
	}

	// only should be called by CollisionWorld
	final void setWorldArrayIndex(int ix)
	{
		m_worldArrayIndex = ix;
	}

	final /*SIMD_FORCE_INLINE*/ btScalar getHitFraction() const
	{
		return m_hitFraction;
	}

	final void setHitFraction(btScalar hitFraction)
	{
		m_hitFraction = hitFraction;
	}

	final /*SIMD_FORCE_INLINE*/ int getCollisionFlags() const
	{
		return m_collisionFlags;
	}

	final void setCollisionFlags(int flags)
	{
		m_collisionFlags = flags;
	}

	///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
	final btScalar getCcdSweptSphereRadius() const
	{
		return m_ccdSweptSphereRadius;
	}

	///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
	final void setCcdSweptSphereRadius(btScalar radius)
	{
		m_ccdSweptSphereRadius = radius;
	}

	final btScalar getCcdMotionThreshold() const
	{
		return m_ccdMotionThreshold;
	}

	final btScalar getCcdSquareMotionThreshold() const
	{
		return m_ccdMotionThreshold * m_ccdMotionThreshold;
	}

	/// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
	final void setCcdMotionThreshold(btScalar ccdMotionThreshold)
	{
		m_ccdMotionThreshold = ccdMotionThreshold;
	}

	///users can point to their objects, userPointer is not used by Bullet
	final void* getUserPointer() //const
	{
		return m_userObjectPointer;
	}

	final int getUserIndex() const
	{
		return m_userIndex;
	}

	final int getUserIndex2() const
	{
		return m_userIndex2;
	}

	final int getUserIndex3() const
	{
		return m_userIndex3;
	}

	///users can point to their objects, userPointer is not used by Bullet
	final void setUserPointer(void* userPointer)
	{
		m_userObjectPointer = userPointer;
	}

	///users can point to their objects, userPointer is not used by Bullet
	final void setUserIndex(int index)
	{
		m_userIndex = index;
	}

	final void setUserIndex2(int index)
	{
		m_userIndex2 = index;
	}

	final void setUserIndex3(int index)
	{
		m_userIndex3 = index;
	}

	final int getUpdateRevisionInternal() const
	{
		return m_updateRevision;
	}

	final void setCustomDebugColor(const ref btVector3 colorRGB)
	{
		m_customDebugColorRGB = colorRGB;
		m_collisionFlags |= CollisionFlags.CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR;
	}

	final void removeCustomDebugColor()
	{
		m_collisionFlags &= ~CollisionFlags.CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR;
	}

	final bool getCustomDebugColor(ref btVector3 colorRGB) const
	{
		bool hasCustomColor = (0 != (m_collisionFlags & CollisionFlags.CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR));
		if (hasCustomColor)
		{
			colorRGB = m_customDebugColorRGB;
		}
		return hasCustomColor;
	}

	final /*inline*/ bool checkCollideWith(const btCollisionObject* co) const
	{
		if (m_checkCollideWith)
			return checkCollideWithOverride(co);

		return true;
	}

	/*virtual*/ int calculateSerializeBufferSize() const
    {
        return btCollisionObjectData.sizeof;
    }

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	pragma(mangle, "?serialize@btCollisionObject@@UEBAPEBDPEAXPEAVbtSerializer@@@Z")
	/*virtual*/ const(char*) serialize(void* dataBuffer, btSerializer serializer) const;

	/*virtual*/ void serializeSingleObject(btSerializer serializer) const;
};

// clang-format off

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btCollisionObjectDoubleData
{
	void					*m_broadphaseHandle;
	void					*m_collisionShape;
	btCollisionShapeData	*m_rootCollisionShape;
	char					*m_name;

	btTransformDoubleData	m_worldTransform;
	btTransformDoubleData	m_interpolationWorldTransform;
	btVector3DoubleData		m_interpolationLinearVelocity;
	btVector3DoubleData		m_interpolationAngularVelocity;
	btVector3DoubleData		m_anisotropicFriction;
	double					m_contactProcessingThreshold;
	double					m_deactivationTime;
	double					m_friction;
	double					m_rollingFriction;
	double                  m_contactDamping;
	double                  m_contactStiffness;
	double					m_restitution;
	double					m_hitFraction;
	double					m_ccdSweptSphereRadius;
	double					m_ccdMotionThreshold;
	int						m_hasAnisotropicFriction;
	int						m_collisionFlags;
	int						m_islandTag1;
	int						m_companionId;
	int						m_activationState1;
	int						m_internalType;
	int						m_checkCollideWith;
	int						m_collisionFilterGroup;
	int						m_collisionFilterMask;
	int						m_uniqueId;//m_uniqueId is introduced for paircache. could get rid of this, by calculating the address offset etc.
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btCollisionObjectFloatData
{
	void					*m_broadphaseHandle;
	void					*m_collisionShape;
	btCollisionShapeData	*m_rootCollisionShape;
	char					*m_name;

	btTransformFloatData	m_worldTransform;
	btTransformFloatData	m_interpolationWorldTransform;
	btVector3FloatData		m_interpolationLinearVelocity;
	btVector3FloatData		m_interpolationAngularVelocity;
	btVector3FloatData		m_anisotropicFriction;
	float					m_contactProcessingThreshold;
	float					m_deactivationTime;
	float					m_friction;
	float					m_rollingFriction;
	float                   m_contactDamping;
    float                   m_contactStiffness;
	float					m_restitution;
	float					m_hitFraction;
	float					m_ccdSweptSphereRadius;
	float					m_ccdMotionThreshold;
	int						m_hasAnisotropicFriction;
	int						m_collisionFlags;
	int						m_islandTag1;
	int						m_companionId;
	int						m_activationState1;
	int						m_internalType;
	int						m_checkCollideWith;
	int						m_collisionFilterGroup;
	int						m_collisionFilterMask;
	int						m_uniqueId;
};
// clang-format on

