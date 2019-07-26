module bullet2.BulletDynamics.ConstraintSolver.btBatchedConstraints;

import bullet2.LinearMath.btThreads;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btIDebugDraw;
import bullet2.BulletDynamics.ConstraintSolver.btSolverBody;
import bullet2.BulletDynamics.ConstraintSolver.btSolverConstraint;

struct btBatchedConstraints
{
	enum BatchingMethod
	{
		BATCHING_METHOD_SPATIAL_GRID_2D,
		BATCHING_METHOD_SPATIAL_GRID_3D,
		BATCHING_METHOD_COUNT
	};
	struct Range
	{
		int begin = 0;
		int end = 0;

		//Range() : begin(0), end(0) {}
		this(int _beg, int _end) { begin = _beg; end = _end; }
	};

	btAlignedObjectArray!int m_constraintIndices;
	btAlignedObjectArray!Range m_batches;        // each batch is a range of indices in the m_constraintIndices array
	btAlignedObjectArray!Range m_phases;         // each phase is range of indices in the m_batches array
	btAlignedObjectArray!char m_phaseGrainSize;  // max grain size for each phase
	btAlignedObjectArray!int m_phaseOrder;       // phases can be done in any order, so we can randomize the order here
	btIDebugDraw* m_debugDrawer = null;

	static bool s_debugDrawBatches;

	//this() { m_debugDrawer = NULL; }
	void setup(btConstraintArray* constraints,
			   ref const(btAlignedObjectArray!btSolverBody) bodies,
			   BatchingMethod batchingMethod,
			   int minBatchSize,
			   int maxBatchSize,
			   btAlignedObjectArray!(char*) scratchMemory);
	bool validate(btConstraintArray* constraints, ref const(btAlignedObjectArray!btSolverBody) bodies) const;
};
