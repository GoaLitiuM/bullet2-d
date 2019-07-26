module bullet2.BulletCollision.CollisionDispatch.btManifoldResult;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btSerializer;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObjectWrapper;
import bullet2.BulletCollision.NarrowPhaseCollision.btManifoldPoint;
import bullet2.BulletCollision.NarrowPhaseCollision.btPersistentManifold;

alias ContactAddedCallback = bool function(ref btManifoldPoint cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1);
extern ContactAddedCallback gContactAddedCallback;

//#define DEBUG_PART_INDEX 1

/// These callbacks are used to customize the algorith that combine restitution, friction, damping, Stiffness
alias CalculateCombinedCallback = btScalar function(const btCollisionObject* body0, const btCollisionObject* body1);

extern CalculateCombinedCallback gCalculateCombinedRestitutionCallback;
extern CalculateCombinedCallback gCalculateCombinedFrictionCallback;
extern CalculateCombinedCallback gCalculateCombinedRollingFrictionCallback;
extern CalculateCombinedCallback gCalculateCombinedSpinningFrictionCallback;
extern CalculateCombinedCallback gCalculateCombinedContactDampingCallback;
extern CalculateCombinedCallback gCalculateCombinedContactStiffnessCallback;

///btManifoldResult is a helper class to manage  contact results.
struct btManifoldResult// : btDiscreteCollisionDetectorInterface.Result
{
	//btDiscreteCollisionDetectorInterface.Result
	//{
		///*virtual*/ ~Result() {}

		///setShapeIdentifiersA/B provides experimental support for per-triangle material / custom material combiner
		/*abstract void setShapeIdentifiersA(int partId0, int index0);
		abstract void setShapeIdentifiersB(int partId1, int index1);
		abstract void addContactPoint(const ref btVector3 normalOnBInWorld, const ref btVector3 pointInWorld, btScalar depth);*/
	//};

protected:
	btPersistentManifold m_manifoldPtr;

	const(btCollisionObjectWrapper)* m_body0Wrap;
	const(btCollisionObjectWrapper)* m_body1Wrap;
	int m_partId0;
	int m_partId1;
	int m_index0;
	int m_index1;

public:
	/*btManifoldResult()
		:
#ifdef DEBUG_PART_INDEX

		  m_partId0(-1),
		  m_partId1(-1),
		  m_index0(-1),
		  m_index1(-1)
#endif  //DEBUG_PART_INDEX
	{
	}*/

	this(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap);

	/*virtual*/ ~this(){};

	void setPersistentManifold(btPersistentManifold manifoldPtr)
	{
		m_manifoldPtr = manifoldPtr;
	}

	/*const btPersistentManifold* getPersistentManifold()
	{
		return m_manifoldPtr;
	}*/
	btPersistentManifold getPersistentManifold()
	{
		return m_manifoldPtr;
	}

	/*virtual*/ void setShapeIdentifiersA(int partId0, int index0)
	{
		m_partId0 = partId0;
		m_index0 = index0;
	}

	/*virtual*/ void setShapeIdentifiersB(int partId1, int index1)
	{
		m_partId1 = partId1;
		m_index1 = index1;
	}

	/*virtual*/ void addContactPoint(const ref btVector3 normalOnBInWorld, const ref btVector3 pointInWorld, btScalar depth);

	/*SIMD_FORCE_INLINE*/ void refreshContactPoints()
	{
		btAssert(m_manifoldPtr);
		if (!m_manifoldPtr.getNumContacts())
			return;

		bool isSwapped = m_manifoldPtr.getBody0() != m_body0Wrap.getCollisionObject();

		if (isSwapped)
		{
			m_manifoldPtr.refreshContactPoints(m_body1Wrap.getCollisionObject().getWorldTransform(), m_body0Wrap.getCollisionObject().getWorldTransform());
		}
		else
		{
			m_manifoldPtr.refreshContactPoints(m_body0Wrap.getCollisionObject().getWorldTransform(), m_body1Wrap.getCollisionObject().getWorldTransform());
		}
	}

	const (btCollisionObjectWrapper*) getBody0Wrap()
	{
		return m_body0Wrap;
	}
	const (btCollisionObjectWrapper*) getBody1Wrap()
	{
		return m_body1Wrap;
	}

	void setBody0Wrap(btCollisionObjectWrapper* obj0Wrap)
	{
		m_body0Wrap = obj0Wrap;
	}

	void setBody1Wrap(btCollisionObjectWrapper* obj1Wrap)
	{
		m_body1Wrap = obj1Wrap;
	}

	const(btCollisionObject) getBody0Internal()
	{
		return m_body0Wrap.getCollisionObject();
	}

	const(btCollisionObject) getBody1Internal()
	{
		return m_body1Wrap.getCollisionObject();
	}

	btScalar m_closestPointDistanceThreshold = 0;

	/// in the future we can let the user override the methods to combine restitution and friction
	static btScalar calculateCombinedRestitution(const btCollisionObject* body0, const btCollisionObject* body1);
	static btScalar calculateCombinedFriction(const btCollisionObject* body0, const btCollisionObject* body1);
	static btScalar calculateCombinedRollingFriction(const btCollisionObject* body0, const btCollisionObject* body1);
	static btScalar calculateCombinedSpinningFriction(const btCollisionObject* body0, const btCollisionObject* body1);
	static btScalar calculateCombinedContactDamping(const btCollisionObject* body0, const btCollisionObject* body1);
	static btScalar calculateCombinedContactStiffness(const btCollisionObject* body0, const btCollisionObject* body1);
};

