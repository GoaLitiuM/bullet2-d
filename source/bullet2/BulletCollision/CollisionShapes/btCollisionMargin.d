module bullet2.BulletCollision.CollisionShapes.btCollisionMargin;

extern (C++):

///The CONVEX_DISTANCE_MARGIN is a default collision margin for convex collision shapes derived from btConvexInternalShape.
///This collision margin is used by Gjk and some other algorithms
///Note that when creating small objects, you need to make sure to set a smaller collision margin, using the 'setMargin' API
enum CONVEX_DISTANCE_MARGIN = 0.04;  // btScalar(0.1)//;//btScalar(0.01)
