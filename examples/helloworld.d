module bullet2._Examples.helloworld;

import bullet2;

void bulletHelloWorld()
{
    ///-----includes_end-----

	///-----initialization_start-----

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher dispatcher = new btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver solver = new btSequentialImpulseConstraintSolver();

	btDiscreteDynamicsWorld dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	dynamicsWorld.setGravity(btVector3(0, -10, 0));

	///-----initialization_end-----

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray!btCollisionShape collisionShapes;

	///create a few basic rigid bodies

	//the ground is a cube of side 100 at position y = -56.
	//the sphere will hit it at y = -6, with center at -5
	if (true)
	{
		btCollisionShape groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

		collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -56, 0));

		btScalar mass = btScalar(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = mass != 0.0f;

		btVector3 localInertia = btVector3(0, 0, 0);
		if (isDynamic)
			groundShape.calculateLocalInertia(mass, localInertia);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody.btRigidBodyConstructionInfo rbInfo = btRigidBody.btRigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody body_ = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		dynamicsWorld.addRigidBody(body_);
	}

	if (true)
	{
		//create a dynamic rigidbody

		btCollisionShape colShape = new btSphereShape(btScalar(1.0));
		collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass = btScalar(1.0f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.0f);

		btVector3 localInertia = btVector3(0, 0, 0);
		if (isDynamic)
			colShape.calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(2, 10, 0));

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody.btRigidBodyConstructionInfo rbInfo = btRigidBody.btRigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia);
		btRigidBody body_ = new btRigidBody(rbInfo);

		dynamicsWorld.addRigidBody(body_);
	}

	/// Do some simulation

	///-----stepsimulation_start-----
	for (int i = 0; i < 150; i++)
	{
		dynamicsWorld.stepSimulation(1.0f / 60.0f, 10);

		//print positions of all objects
		for (int j = dynamicsWorld.getNumCollisionObjects() - 1; j >= 0; j--)
		{
			btCollisionObject obj = dynamicsWorld.getCollisionObjectArray()[j];
			btRigidBody body_ = btRigidBody.upcast(obj);
			btTransform trans;
			if (body_ && body_.getMotionState())
			{
				body_.getMotionState().getWorldTransform(trans);
			}
			else
			{
				trans = obj.getWorldTransform();
			}
			import core.stdc.stdio;
			printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
		}
	}

	///-----stepsimulation_end-----

	//cleanup in the reverse order of creation/initialization

	///-----cleanup_start-----

	//remove the rigidbodies from the dynamics world and delete them
	for (int i = dynamicsWorld.getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject obj = dynamicsWorld.getCollisionObjectArray()[i];
		btRigidBody body_ = btRigidBody.upcast(obj);
		if (body_ && body_.getMotionState())
		{
			destroy(body_.getMotionState());
		}
		dynamicsWorld.removeCollisionObject(obj);
		destroy(obj);
	}

	//delete collision shapes
	for (int j = 0; j < collisionShapes.size(); j++)
	{
		btCollisionShape shape = collisionShapes[j];
		destroy(shape);

		collisionShapes[j] = null;
		assert(collisionShapes[j] is null);
	}

	//delete dynamics world
	destroy(dynamicsWorld);

	//delete solver
	destroy(solver);

	//delete broadphase
	destroy(overlappingPairCache);

	//delete dispatcher
	destroy(dispatcher);

	destroy(collisionConfiguration);

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();
}
