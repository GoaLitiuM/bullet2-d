module bullet2.BulletCollision.CollisionShapes.btBoxShape;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btPolyhedralConvexShape;
import bullet2.BulletCollision.CollisionShapes.btCollisionMargin;
import bullet2.BulletCollision.CollisionShapes.btConvexInternalShape;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseProxy;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btMinMax;
import bullet2.LinearMath.btTransform;

///The btBoxShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a btCollisionObject or btRigidBody it will be an oriented box in world space.
align(16) class btBoxShape : btPolyhedralConvexShape
{
	//btVector3	m_boxHalfExtents1; //use m_implicitShapeDimensions instead

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final btVector3 getHalfExtentsWithMargin() const
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();
		btVector3 margin = btVector3(getMargin(), getMargin(), getMargin());
		halfExtents += margin;
		return halfExtents;
	}

	final ref const(btVector3) getHalfExtentsWithoutMargin() const
	{
		return m_implicitShapeDimensions;  //scaling is included, margin is not
	}

    final btVector3 localGetSupportingVertex(btVector3 vec) const
    {
        return localGetSupportingVertex(vec);
    }
	override /*virtual*/ btVector3 localGetSupportingVertex(ref const(btVector3) vec) const
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();
		btVector3 margin = btVector3(getMargin(), getMargin(), getMargin());
		halfExtents += margin;

		return btVector3(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
						 btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
						 btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	final override /*SIMD_FORCE_INLINE*/ btVector3 localGetSupportingVertexWithoutMargin(ref const(btVector3) vec) const
	{
		const(btVector3) halfExtents = getHalfExtentsWithoutMargin();

		return btVector3(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
						 btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
						 btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	override /*virtual*/ void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		const(btVector3) halfExtents = getHalfExtentsWithoutMargin();

		for (int i = 0; i < numVectors; i++)
		{
			const (btVector3*) vec = &vectors[i];
			supportVerticesOut[i].setValue(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
										   btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
										   btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
		}
	}

	final this(const(btVector3) boxHalfExtents)
	{
		this(boxHalfExtents);
	}
	final this(ref const(btVector3) boxHalfExtents);

	override /*virtual*/ void setMargin(btScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		btVector3 oldMargin = btVector3(getMargin(), getMargin(), getMargin());
		btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

		btConvexInternalShape.setMargin(collisionMargin);
		btVector3 newMargin = btVector3(getMargin(), getMargin(), getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
	}
	override /*virtual*/ void setLocalScaling(ref const(btVector3) scaling)
	{
		btVector3 oldMargin = btVector3(getMargin(), getMargin(), getMargin());
		btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;
		btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		btConvexInternalShape.setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
	}

	override /*virtual*/ void getAabb(const ref btTransform t, ref btVector3 aabbMin, ref btVector3 aabbMax) const;

	override /*virtual*/ void calculateLocalInertia(btScalar mass, ref btVector3 inertia) const;

	override /*virtual*/ void getPlane(ref btVector3 planeNormal, ref btVector3 planeSupport, int i) const
	{
		//this plane might not be aligned...
		btVector4 plane;
		getPlaneEquation(plane, i);
		planeNormal = btVector3(plane.getX(), plane.getY(), plane.getZ());
		planeSupport = localGetSupportingVertex(-planeNormal);
	}

	override /*virtual*/ int getNumPlanes() const
	{
		return 6;
	}

	override /*virtual*/ int getNumVertices() const
	{
		return 8;
	}

	override /*virtual*/ int getNumEdges() const
	{
		return 12;
	}

	override /*virtual*/ void getVertex(int i, ref btVector3 vtx) const
	{
		btVector3 halfExtents = getHalfExtentsWithMargin();

		vtx = btVector3(
			halfExtents.x() * (1 - (i & 1)) - halfExtents.x() * (i & 1),
			halfExtents.y() * (1 - ((i & 2) >> 1)) - halfExtents.y() * ((i & 2) >> 1),
			halfExtents.z() * (1 - ((i & 4) >> 2)) - halfExtents.z() * ((i & 4) >> 2));
	}

	/*virtual*/ void getPlaneEquation(ref btVector4 plane, int i) const
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();

		switch (i)
		{
			case 0:
				plane.setValue(btScalar(1.), btScalar(0.), btScalar(0.), -halfExtents.x());
				break;
			case 1:
				plane.setValue(btScalar(-1.), btScalar(0.), btScalar(0.), -halfExtents.x());
				break;
			case 2:
				plane.setValue(btScalar(0.), btScalar(1.), btScalar(0.), -halfExtents.y());
				break;
			case 3:
				plane.setValue(btScalar(0.), btScalar(-1.), btScalar(0.), -halfExtents.y());
				break;
			case 4:
				plane.setValue(btScalar(0.), btScalar(0.), btScalar(1.), -halfExtents.z());
				break;
			case 5:
				plane.setValue(btScalar(0.), btScalar(0.), btScalar(-1.), -halfExtents.z());
				break;
			default:
				btAssert(0);
		}
	}

	override /*virtual*/ void getEdge(int i, ref btVector3 pa, ref btVector3 pb) const
	//final override /*virtual*/ void getEdge(int i,Edge& edge) const
	{
		int edgeVert0 = 0;
		int edgeVert1 = 0;

		switch (i)
		{
			case 0:
				edgeVert0 = 0;
				edgeVert1 = 1;
				break;
			case 1:
				edgeVert0 = 0;
				edgeVert1 = 2;
				break;
			case 2:
				edgeVert0 = 1;
				edgeVert1 = 3;

				break;
			case 3:
				edgeVert0 = 2;
				edgeVert1 = 3;
				break;
			case 4:
				edgeVert0 = 0;
				edgeVert1 = 4;
				break;
			case 5:
				edgeVert0 = 1;
				edgeVert1 = 5;

				break;
			case 6:
				edgeVert0 = 2;
				edgeVert1 = 6;
				break;
			case 7:
				edgeVert0 = 3;
				edgeVert1 = 7;
				break;
			case 8:
				edgeVert0 = 4;
				edgeVert1 = 5;
				break;
			case 9:
				edgeVert0 = 4;
				edgeVert1 = 6;
				break;
			case 10:
				edgeVert0 = 5;
				edgeVert1 = 7;
				break;
			case 11:
				edgeVert0 = 6;
				edgeVert1 = 7;
				break;
			default:
				btAssert(0);
		}

		getVertex(edgeVert0, pa);
		getVertex(edgeVert1, pb);
	}

	override /*virtual*/ bool isInside(ref const(btVector3) pt, btScalar tolerance) const
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();

		//btScalar minDist = 2*tolerance;

		bool result = (pt.x() <= (halfExtents.x() + tolerance)) &&
					  (pt.x() >= (-halfExtents.x() - tolerance)) &&
					  (pt.y() <= (halfExtents.y() + tolerance)) &&
					  (pt.y() >= (-halfExtents.y() - tolerance)) &&
					  (pt.z() <= (halfExtents.z() + tolerance)) &&
					  (pt.z() >= (-halfExtents.z() - tolerance));

		return result;
	}

	//debugging
	override /*virtual*/ const(char*) getName() const
	{
		return "Box";
	}

	override /*virtual*/ int getNumPreferredPenetrationDirections() const
	{
		return 6;
	}

	override /*virtual*/ void getPreferredPenetrationDirection(int index, ref btVector3 penetrationVector) const
	{
		switch (index)
		{
			case 0:
				penetrationVector.setValue(btScalar(1.), btScalar(0.), btScalar(0.));
				break;
			case 1:
				penetrationVector.setValue(btScalar(-1.), btScalar(0.), btScalar(0.));
				break;
			case 2:
				penetrationVector.setValue(btScalar(0.), btScalar(1.), btScalar(0.));
				break;
			case 3:
				penetrationVector.setValue(btScalar(0.), btScalar(-1.), btScalar(0.));
				break;
			case 4:
				penetrationVector.setValue(btScalar(0.), btScalar(0.), btScalar(1.));
				break;
			case 5:
				penetrationVector.setValue(btScalar(0.), btScalar(0.), btScalar(-1.));
				break;
			default:
				btAssert(0);
		}
	}
};
