module bullet2.BulletCollision.CollisionDispatch.btCollisionObjectWrapper;

extern (C++):

import bullet2.LinearMath.btScalar; // for SIMD_FORCE_INLINE definition
import bullet2.LinearMath.btTransform;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;

struct btCollisionObjectWrapper
{
	//BT_DECLARE_STACK_ONLY_OBJECT

/*private:
	this(const ref btCollisionObjectWrapper);  // not implemented. Not allowed.
	btCollisionObjectWrapper* operator=(const ref btCollisionObjectWrapper);
*/
public:
	const(btCollisionObjectWrapper)* m_parent;
	const btCollisionShape m_shape;
	const btCollisionObject m_collisionObject;
	const(btTransform)* m_worldTransform;
	int m_partId;
	int m_index;

	this(btCollisionObjectWrapper* parent, btCollisionShape shape, btCollisionObject collisionObject, btTransform* worldTransform, int partId, int index)
	{
        m_parent = parent;
        m_shape = shape;
        m_collisionObject = collisionObject;
        m_worldTransform = worldTransform;
        m_partId = partId;
        m_index = index;
	}

	/*SIMD_FORCE_INLINE*/ ref const(btTransform) getWorldTransform() const { return *m_worldTransform; }
	/*SIMD_FORCE_INLINE*/ const(btCollisionObject) getCollisionObject() const { return m_collisionObject; }
	/*SIMD_FORCE_INLINE*/ const(btCollisionShape) getCollisionShape() const { return m_shape; }
};
