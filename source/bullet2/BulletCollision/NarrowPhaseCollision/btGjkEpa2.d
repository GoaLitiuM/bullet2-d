module bullet2.BulletCollision.NarrowPhaseCollision.btGjkEpa2;

extern (C++):

import bullet2.BulletCollision.CollisionShapes.btConvexShape;
import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;

///btGjkEpaSolver contributed under zlib by Nathanael Presson
struct btGjkEpaSolver2
{
	struct sResults
	{
		enum eStatus
		{
			Separated,   /* Shapes doesnt penetrate												*/
			Penetrating, /* Shapes are penetrating												*/
			GJK_Failed,  /* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
			EPA_Failed   /* EPA phase fail, bigger problem, need to save parameters, and debug	*/
		};
        eStatus status;
		btVector3[2] witnesses;
		btVector3 normal;
		btScalar distance;
	};

	static int StackSizeRequirement();

	static bool Distance(const(btConvexShape) shape0, ref const(btTransform) wtrs0,
						 const(btConvexShape) shape1, ref const(btTransform) wtrs1,
						 ref const(btVector3) guess,
						 ref sResults results);

	static bool Penetration(const(btConvexShape) shape0, ref const(btTransform) wtrs0,
							const(btConvexShape) shape1, ref const(btTransform) wtrs1,
							ref const(btVector3) guess,
							ref sResults results,
							bool usemargins = true);
//#ifndef __SPU__
    pragma(mangle, "?SignedDistance@btGjkEpaSolver2@@SAMAEBVbtVector3@@MPEBVbtConvexShape@@AEBVbtTransform@@AEAUsResults@1@@Z")
	static btScalar SignedDistance(ref const(btVector3) position,
								   btScalar margin,
								   const(btConvexShape) shape,
								   ref const(btTransform) wtrs,
								   ref sResults results);

	static bool SignedDistance(const(btConvexShape) shape0, ref const(btTransform) wtrs0,
							   const(btConvexShape) shape1, ref const(btTransform) wtrs1,
							   ref const(btVector3) guess,
							   ref sResults results);
//#endif  //__SPU__
};
