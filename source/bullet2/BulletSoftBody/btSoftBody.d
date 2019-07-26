module bullet2.BulletSoftBody.btSoftBody;

extern (C++):

import bullet2.BulletSoftBody.btSoftBodySolver;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletCollision.CollisionDispatch.btCollisionCreateFunc;
import bullet2.BulletCollision.BroadphaseCollision.btDbvt;
import bullet2.BulletCollision.BroadphaseCollision.btDispatcher;
import bullet2.BulletCollision.BroadphaseCollision.btBroadphaseInterface;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObjectWrapper;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.BulletSoftBody.btSparseSDF;
import bullet2.BulletSoftBody.btSoftBodyData;

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btMatrix3x3;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btTransform;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btIDebugDraw;
import bullet2.LinearMath.btQuaternion;
import bullet2.LinearMath.btSerializer;

import bullet2.BulletDynamics.Dynamics.btRigidBody;

//#ifdef BT_USE_DOUBLE_PRECISION
//#define btRigidBodyData	btRigidBodyDoubleData
//#define btRigidBodyDataName	"btRigidBodyDoubleData"
//#else
alias btSoftBodyData = btSoftBodyFloatData;
enum btSoftBodyDataName = "btSoftBodyFloatData";
//#endif //BT_USE_DOUBLE_PRECISION

/* btSoftBodyWorldInfo	*/
struct btSoftBodyWorldInfo
{
	btScalar air_density = btScalar(1.2);
	btScalar water_density = 0;
	btScalar water_offset = 0;
	btScalar m_maxDisplacement = 1000.0f; //avoid soft body from 'exploding' so use some upper threshold of maximum motion that a node can travel per frame
	btVector3 water_normal = btVector3(0, 0, 0);
	btBroadphaseInterface m_broadphase = null;
	btDispatcher m_dispatcher = null;
	btVector3 m_gravity = btVector3(0, -10, 0);
	btSparseSdf!(3) m_sparsesdf;
};

///The btSoftBody is an class to simulate cloth and volumetric soft bodies.
///There is two-way interaction between btSoftBody and btRigidBody/btCollisionObject.
class btSoftBody : btCollisionObject
{
public:
	btAlignedObjectArray!(btCollisionObject) m_collisionDisabledObjects;

	// The solver object that handles this soft body
	btSoftBodySolver m_softBodySolver;

	//
	// Enumerations
	//

	///eAeroModel
	struct eAeroModel
	{
		enum _
		{
			V_Point,             ///Vertex normals are oriented toward velocity
			V_TwoSided,          ///Vertex normals are flipped to match velocity
			V_TwoSidedLiftDrag,  ///Vertex normals are flipped to match velocity and lift and drag forces are applied
			V_OneSided,          ///Vertex normals are taken as it is
			F_TwoSided,          ///Face normals are flipped to match velocity
			F_TwoSidedLiftDrag,  ///Face normals are flipped to match velocity and lift and drag forces are applied
			F_OneSided,          ///Face normals are taken as it is
			END
		};
	};

	///eVSolver : velocities solvers
	struct eVSolver
	{
		enum _
		{
			Linear,  ///Linear solver
			END
		};
	};

	///ePSolver : positions solvers
	struct ePSolver
	{
		enum _
		{
			Linear,     ///Linear solver
			Anchors,    ///Anchor solver
			RContacts,  ///Rigid contacts solver
			SContacts,  ///Soft contacts solver
			END
		};
	};

	///eSolverPresets
	struct eSolverPresets
	{
		enum _
		{
			Positions,
			Velocities,
			Default = Positions,
			END
		};
	};

	///eFeature
	struct eFeature
	{
		enum _
		{
			None,
			Node,
			Link,
			Face,
			Tetra,
			END
		};
	};

	alias tVSolverArray = btAlignedObjectArray!(eVSolver._);
	alias tPSolverArray = btAlignedObjectArray!(ePSolver._);

	//
	// Flags
	//

	///fCollision
	struct fCollision
	{
		enum _
		{
			RVSmask = 0x000f,  ///Rigid versus soft mask
			SDF_RS = 0x0001,   ///SDF based rigid vs soft
			CL_RS = 0x0002,    ///Cluster vs convex rigid vs soft

			SVSmask = 0x0030,  ///Rigid versus soft mask
			VF_SS = 0x0010,    ///Vertex vs face soft vs soft handling
			CL_SS = 0x0020,    ///Cluster vs cluster soft vs soft handling
			CL_SELF = 0x0040,  ///Cluster soft body self collision
			/* presets	*/
			Default = SDF_RS,
			END
		};
	};

	///fMaterial
	struct fMaterial
	{
		enum _
		{
			DebugDraw = 0x0001,  /// Enable debug draw
			/* presets	*/
			Default = DebugDraw,
			END
		};
	};

	//
	// API Types
	//

	/* sRayCast		*/
	struct sRayCast
	{
		btSoftBody body_;     /// soft body
		eFeature._ feature;  /// feature type
		int index;            /// feature index
		btScalar fraction;    /// time of impact fraction (rayorg+(rayto-rayfrom)*fraction)
	};

	/* ImplicitFn	*/
	class ImplicitFn
	{
		/*virtual*/ ~this() {}
		abstract /*virtual*/ btScalar Eval(ref const(btVector3) x);
	};

	//
	// Internal types
	//

	alias tScalarArray = btAlignedObjectArray!btScalar;
	alias tVector3Array = btAlignedObjectArray!btVector3;

	/* sCti is Softbody contact info	*/
	struct sCti
	{
		const(btCollisionObject)* m_colObj; /* Rigid body			*/
		btVector3 m_normal;                /* Outward normal		*/
		btScalar m_offset;                 /* Offset from origin	*/
	};

	/* sMedium		*/
	struct sMedium
	{
		btVector3 m_velocity; /* Velocity				*/
		btScalar m_pressure;  /* Pressure				*/
		btScalar m_density;   /* Density				*/
	};

	/* Base type	*/
	extern (C++, struct)
	align(16) struct Element
	{
		void* m_tag = null;  // User data
		//this() : m_tag(0) {}
	};
	/* Material		*/
	extern (C++, struct)
	align(16) struct Material// : Element
	{
		Element element_;
		alias element_ this;

		btScalar m_kLST;  // Linear stiffness coefficient [0,1]
		btScalar m_kAST;  // Area/Angular stiffness coefficient [0,1]
		btScalar m_kVST;  // Volume stiffness coefficient [0,1]
		int m_flags;      // Flags
	};

	/* Feature		*/
	extern (C++, struct)
	align(16) struct Feature// : Element
	{
		Element element_;
		alias element_ this;

		Material* m_material;  // Material
	};
	/* Node			*/
	extern (C++, struct)
	align(16) struct Node// : Feature
	{
		Feature feature_;
		alias feature_ this;

		btVector3 m_x;       // Position
		btVector3 m_q;       // Previous step position
		btVector3 m_v;       // Velocity
		btVector3 m_f;       // Force accumulator
		btVector3 m_n;       // Normal
		btScalar m_im;       // 1/mass
		btScalar m_area;     // Area
		btDbvtNode* m_leaf;  // Leaf data
		byte m_battach;//int m_battach : 1;   // Attached
	};
	/* Link			*/
	//ATTRIBUTE_ALIGNED16(struct)
    extern (C++, struct)
	align(16) struct Link //: Feature
	{
		Feature feature_;
		alias feature_ this;

		btVector3 m_c3;      // gradient
		Node*[2] m_n;        // Node pointers
		btScalar m_rl;       // Rest length
		byte m_bbending;//int m_bbending : 1;  // Bending link
		btScalar m_c0;       // (ima+imb)*kLST
		btScalar m_c1;       // rl^2
		btScalar m_c2;       // |gradient|^2/c0

		//BT_DECLARE_ALIGNED_ALLOCATOR();
	};
	/* Face			*/
	extern (C++, struct)
	align(16) struct Face //: Feature
	{
		Feature feature_;
		alias feature_ this;

		Node*[3] m_n;        // Node pointers
		btVector3 m_normal;  // Normal
		btScalar m_ra;       // Rest area
		btDbvtNode* m_leaf;  // Leaf data
	};
	/* Tetra		*/
	extern (C++, struct)
	align(16) struct Tetra //: Feature
	{
		Feature feature_;
		alias feature_ this;

		Node*[4] m_n;        // Node pointers
		btScalar m_rv;       // Rest volume
		btDbvtNode* m_leaf;  // Leaf data
		btVector3[4] m_c0;   // gradients
		btScalar m_c1;       // (4*kVST)/(im0+im1+im2+im3)
		btScalar m_c2;       // m_c1/sum(|g0..3|^2)
	};
	/* RContact		*/
	struct RContact
	{
		sCti m_cti;        // Contact infos
		Node* m_node;      // Owner node
		btMatrix3x3 m_c0;  // Impulse matrix
		btVector3 m_c1;    // Relative anchor
		btScalar m_c2;     // ima*dt
		btScalar m_c3;     // Friction
		btScalar m_c4;     // Hardness
	};
	/* SContact		*/
	struct SContact
	{
		Node* m_node;         // Node
		Face* m_face;         // Face
		btVector3 m_weights;  // Weigths
		btVector3 m_normal;   // Normal
		btScalar m_margin;    // Margin
		btScalar m_friction;  // Friction
		btScalar[2] m_cfm;    // Constraint force mixing
	};
	/* Anchor		*/
	struct Anchor
	{
		Node* m_node;         // Node pointer
		btVector3 m_local;    // Anchor position in body space
		btRigidBody* m_body;  // Body
		btScalar m_influence;
		btMatrix3x3 m_c0;  // Impulse matrix
		btVector3 m_c1;    // Relative anchor
		btScalar m_c2;     // ima*dt
	};
	/* Note			*/
	extern (C++, struct)
	align(16) struct Note //: Element
	{
		Element element_;
		alias element_ this;

		char* m_text;    // Text
		btVector3 m_offset;    // Offset
		int m_rank;            // Rank
		Node*[4] m_nodes;      // Nodes
		btScalar[4] m_coords;  // Coordinates
	};
	/* Pose			*/
	struct Pose
	{
		bool m_bvolume;       // Is valid
		bool m_bframe;        // Is frame
		btScalar m_volume;    // Rest volume
		tVector3Array m_pos;  // Reference positions
		tScalarArray m_wgh;   // Weights
		btVector3 m_com;      // COM
		btMatrix3x3 m_rot;    // Rotation
		btMatrix3x3 m_scl;    // Scale
		btMatrix3x3 m_aqq;    // Base scaling
	};
	/* Cluster		*/
	struct Cluster
	{
		tScalarArray m_masses;
		btAlignedObjectArray!(Node*) m_nodes;
		tVector3Array m_framerefs;
		btTransform m_framexform;
		btScalar m_idmass;
		btScalar m_imass;
		btMatrix3x3 m_locii;
		btMatrix3x3 m_invwi;
		btVector3 m_com;
		btVector3[2] m_vimpulses;
		btVector3[2] m_dimpulses;
		int m_nvimpulses;
		int m_ndimpulses;
		btVector3 m_lv;
		btVector3 m_av;
		btDbvtNode* m_leaf = null;
		btScalar m_ndamping = 0; /* Node damping		*/
		btScalar m_ldamping = 0; /* Linear damping	*/
		btScalar m_adamping = 0; /* Angular damping	*/
		btScalar m_matching = 0;
		btScalar m_maxSelfCollisionImpulse = 100.0f;
		btScalar m_selfCollisionImpulseFactor = 0.01f;
		bool m_containsAnchor = false;
		bool m_collide;
		int m_clusterIndex;
		/*Cluster() : m_leaf(0), m_ndamping(0), m_ldamping(0), m_adamping(0), m_matching(0), m_maxSelfCollisionImpulse(100.f), m_selfCollisionImpulseFactor(0.01f), m_containsAnchor(false)
		{
		}*/
	};
	/* Impulse		*/
	struct Impulse
	{
        import std.bitmanip;
		btVector3 m_velocity = btVector3(0, 0, 0);
		btVector3 m_drift = btVector3(0, 0, 0);
        mixin(bitfields!(
            int, "m_asVelocity", 1,
            int, "m_asDrift", 1,
            int, "", 6));

        auto ref Impulse opUnary(string op)() const
		{
            static if (op == "-")
            {
                this.m_velocity = -m_velocity;
                this.m_drift = -m_drift;
                return this;
            }
            else
                static assert(0, "not implemented");
		}
		//Impulse operator*(btScalar x) const
        auto ref Impulse opBinary(string op)(in btScalar x) const
		{
            static if (op == "*")
            {
                this.m_velocity *= x;
                this.m_drift *= x;
                return this;
            }
            else
                static assert(0, "not implemented");
		}
	};
	/* Body			*/
	struct Body
	{
		Cluster* m_soft = null;
		btRigidBody m_rigid = null;
		const(btCollisionObject)* m_collisionObject = null;

		this(Cluster* p) /*: m_soft(p), m_rigid(0), m_collisionObject(0)*/ { m_soft = p; }
		this(const(btCollisionObject)* colObj) //: m_soft(0), m_collisionObject(colObj)
		{
            m_collisionObject = colObj;
			m_rigid = cast(btRigidBody)m_collisionObject;//(btRigidBody*)btRigidBody::upcast(m_collisionObject);
		}

		void activate() const
		{
			if (m_rigid)
				m_rigid.activate();
			if (m_collisionObject)
				m_collisionObject.activate();
		}
		ref const(btMatrix3x3) invWorldInertia() const
		{
			static const(btMatrix3x3) iwi = btMatrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
			if (m_rigid) return (m_rigid.getInvInertiaTensorWorld());
			if (m_soft) return (m_soft.m_invwi);
			return (iwi);
		}
		btScalar invMass() const
		{
			if (m_rigid) return (m_rigid.getInvMass());
			if (m_soft) return (m_soft.m_imass);
			return (0);
		}
		ref const(btTransform) xform() const
		{
			static const(btTransform) identity = btTransform.getIdentity();
			if (m_collisionObject) return (m_collisionObject.getWorldTransform());
			if (m_soft) return (m_soft.m_framexform);
			return (identity);
		}
		btVector3 linearVelocity() const
		{
			if (m_rigid) return (m_rigid.getLinearVelocity());
			if (m_soft) return (m_soft.m_lv);
			return (btVector3(0, 0, 0));
		}
		btVector3 angularVelocity(ref const(btVector3) rpos) const
		{
			if (m_rigid) return (btCross(m_rigid.getAngularVelocity(), rpos));
			if (m_soft) return (btCross(m_soft.m_av, rpos));
			return (btVector3(0, 0, 0));
		}
		btVector3 angularVelocity() const
		{
			if (m_rigid) return (m_rigid.getAngularVelocity());
			if (m_soft) return (m_soft.m_av);
			return (btVector3(0, 0, 0));
		}
		btVector3 velocity(ref const(btVector3) rpos) const
		{
			return (linearVelocity() + angularVelocity(rpos));
		}
		void applyVImpulse(ref const(btVector3) impulse, ref const(btVector3) rpos) //const
		{
			if (m_rigid) m_rigid.applyImpulse(impulse, rpos);
			if (m_soft) btSoftBody.clusterVImpulse(m_soft, rpos, impulse);
		}
		void applyDImpulse(ref const(btVector3) impulse, ref const(btVector3) rpos) //const
		{
			if (m_rigid) m_rigid.applyImpulse(impulse, rpos);
			if (m_soft) btSoftBody.clusterDImpulse(m_soft, rpos, impulse);
		}
		void applyImpulse(ref const(Impulse) impulse, ref const(btVector3) rpos) //const
		{
			if (impulse.m_asVelocity)
			{
				//				printf("impulse.m_velocity = %f,%f,%f\n",impulse.m_velocity.getX(),impulse.m_velocity.getY(),impulse.m_velocity.getZ());
				applyVImpulse(impulse.m_velocity, rpos);
			}
			if (impulse.m_asDrift)
			{
				//				printf("impulse.m_drift = %f,%f,%f\n",impulse.m_drift.getX(),impulse.m_drift.getY(),impulse.m_drift.getZ());
				applyDImpulse(impulse.m_drift, rpos);
			}
		}
		void applyVAImpulse(ref const(btVector3) impulse) //const
		{
			if (m_rigid) m_rigid.applyTorqueImpulse(impulse);
			if (m_soft) btSoftBody.clusterVAImpulse(m_soft, impulse);
		}
		void applyDAImpulse(ref const(btVector3) impulse) //const
		{
			if (m_rigid) m_rigid.applyTorqueImpulse(impulse);
			if (m_soft) btSoftBody.clusterDAImpulse(m_soft, impulse);
		}
		void applyAImpulse(ref const(Impulse) impulse) //const
		{
			if (impulse.m_asVelocity) applyVAImpulse(impulse.m_velocity);
			if (impulse.m_asDrift) applyDAImpulse(impulse.m_drift);
		}
		void applyDCImpulse(ref const(btVector3) impulse) //const
		{
			if (m_rigid) m_rigid.applyCentralImpulse(impulse);
			if (m_soft) btSoftBody.clusterDCImpulse(m_soft, impulse);
		}
	};
	/* Joint		*/
	extern (C++, struct)
	class Joint
	{
		struct eType
		{
			enum _
			{
				Linear = 0,
				Angular,
				Contact
			};
		};
		struct Specs
		{
			//Specs() : erp(1), cfm(1), split(1) {}
			btScalar erp = 1;
			btScalar cfm = 1;
			btScalar split = 1;
		};
		Body[2] m_bodies;
		btVector3[2] m_refs;
		btScalar m_cfm;
		btScalar m_erp;
		btScalar m_split;
		btVector3 m_drift;
		btVector3 m_sdrift;
		btMatrix3x3 m_massmatrix;
		bool m_delete = false;
		/*virtual*/ ~this() {}
		//this() : m_delete(false) {}
		/*virtual*/ void Prepare(btScalar dt, int iterations);
		abstract /*virtual*/ void Solve(btScalar dt, btScalar sor);
		abstract /*virtual*/ void Terminate(btScalar dt);
		abstract /*virtual*/ Joint.eType._ Type() const;
	};
	/* LJoint		*/
	extern (C++, struct)
	class LJoint : Joint
	{
		extern (C++, struct)
		struct Specs //: Joint.Specs
		{
			Joint.Specs specs_;
			alias specs_ this;

			btVector3 position;
		};
		btVector3[2] m_rpos;
		final override void Prepare(btScalar dt, int iterations);
		final override void Solve(btScalar dt, btScalar sor);
		final override void Terminate(btScalar dt);
		final override Joint.eType._ Type() const { return (Joint.eType._.Linear); }
	};
	/* AJoint		*/
	extern (C++, struct)
	class AJoint : Joint
	{
		struct IControl
		{
			/*virtual*/ ~this() {}
			/*virtual*/ void Prepare(AJoint*) {}
			/*virtual*/ btScalar Speed(AJoint*, btScalar current) { return (current); }
			static IControl* Default()
			{
				static __gshared IControl def;
				return (&def);
			}
		};
		extern (C++, struct)
		struct Specs //: Joint.Specs
		{
			Joint.Specs specs_;
			alias specs_ this;

			//Specs() : icontrol() {}
			btVector3 axis;
			IControl* icontrol = IControl.Default();
		};
		btVector3[2] m_axis;
		IControl* m_icontrol;
		final override void Prepare(btScalar dt, int iterations);
		final override void Solve(btScalar dt, btScalar sor);
		final override void Terminate(btScalar dt);
		final override Joint.eType._ Type() const { return (Joint.eType._.Angular); }
	};
	/* CJoint		*/
	extern (C++, struct)
	class CJoint : Joint
	{
		int m_life;
		int m_maxlife;
		btVector3[2] m_rpos;
		btVector3 m_normal;
		btScalar m_friction;
		final override void Prepare(btScalar dt, int iterations);
		final override void Solve(btScalar dt, btScalar sor);
		final override void Terminate(btScalar dt);
		final override Joint.eType._ Type() const { return (Joint.eType._.Contact); }
	};
	/* Config		*/
	struct Config
	{
		eAeroModel._ aeromodel;    // Aerodynamic model (default: V_Point)
		btScalar kVCF;              // Velocities correction factor (Baumgarte)
		btScalar kDP;               // Damping coefficient [0,1]
		btScalar kDG;               // Drag coefficient [0,+inf]
		btScalar kLF;               // Lift coefficient [0,+inf]
		btScalar kPR;               // Pressure coefficient [-inf,+inf]
		btScalar kVC;               // Volume conversation coefficient [0,+inf]
		btScalar kDF;               // Dynamic friction coefficient [0,1]
		btScalar kMT;               // Pose matching coefficient [0,1]
		btScalar kCHR;              // Rigid contacts hardness [0,1]
		btScalar kKHR;              // Kinetic contacts hardness [0,1]
		btScalar kSHR;              // Soft contacts hardness [0,1]
		btScalar kAHR;              // Anchors hardness [0,1]
		btScalar kSRHR_CL;          // Soft vs rigid hardness [0,1] (cluster only)
		btScalar kSKHR_CL;          // Soft vs kinetic hardness [0,1] (cluster only)
		btScalar kSSHR_CL;          // Soft vs soft hardness [0,1] (cluster only)
		btScalar kSR_SPLT_CL;       // Soft vs rigid impulse split [0,1] (cluster only)
		btScalar kSK_SPLT_CL;       // Soft vs rigid impulse split [0,1] (cluster only)
		btScalar kSS_SPLT_CL;       // Soft vs rigid impulse split [0,1] (cluster only)
		btScalar maxvolume;         // Maximum volume ratio for pose
		btScalar timescale;         // Time scale
		int viterations;            // Velocities solver iterations
		int piterations;            // Positions solver iterations
		int diterations;            // Drift solver iterations
		int citerations;            // Cluster solver iterations
		int collisions;             // Collisions flags
		tVSolverArray m_vsequence;  // Velocity solvers sequence
		tPSolverArray m_psequence;  // Position solvers sequence
		tPSolverArray m_dsequence;  // Drift solvers sequence
	};
	/* SolverState	*/
	struct SolverState
	{
		btScalar sdt;     // dt*timescale
		btScalar isdt;    // 1/sdt
		btScalar velmrg;  // velocity margin
		btScalar radmrg;  // radial margin
		btScalar updmrg;  // Update margin
	};
	/// RayFromToCaster takes a ray from, ray to (instead of direction!)
	/+extern (C++, struct)
	class RayFromToCaster : btDbvt.ICollide
	{
		btVector3 m_rayFrom;
		btVector3 m_rayTo;
		btVector3 m_rayNormalizedDirection;
		btScalar m_mint;
		Face* m_face;
		int m_tests;
		this(ref const(btVector3) rayFrom, ref const(btVector3) rayTo, btScalar mxt);
		final override void Process(const btDbvtNode* leaf);

		static /*inline*/ btScalar rayFromToTriangle(ref const(btVector3) rayFrom,
													 ref const(btVector3) rayTo,
													 ref const(btVector3) rayNormalizedDirection,
													 ref const(btVector3) a,
													 ref const(btVector3) b,
													 ref const(btVector3) c,
													 btScalar maxt = SIMD_INFINITY);
	};+/

	//
	// Typedefs
	//

	alias psolver_t = void function(btSoftBody, btScalar, btScalar);
	alias vsolver_t = void function(btSoftBody, btScalar);
	alias tClusterArray = btAlignedObjectArray!(Cluster*);
	alias tNoteArray = btAlignedObjectArray!(Note);
	alias tNodeArray = btAlignedObjectArray!(Node);
	alias tLeafArray = btAlignedObjectArray!(btDbvtNode*);
	alias tLinkArray = btAlignedObjectArray!(Link);
	alias tFaceArray = btAlignedObjectArray!(Face);
	alias tTetraArray = btAlignedObjectArray!(Tetra);
	alias tAnchorArray = btAlignedObjectArray!(Anchor);
	alias tRContactArray = btAlignedObjectArray!(RContact);
	alias tSContactArray = btAlignedObjectArray!(SContact);
	alias tMaterialArray = btAlignedObjectArray!(Material*);
	alias tJointArray = btAlignedObjectArray!(Joint*);
	alias tSoftBodyArray = btAlignedObjectArray!(btSoftBody);

	//
	// Fields
	//

	Config m_cfg;                      // Configuration
	SolverState m_sst;                 // Solver state
	Pose m_pose;                       // Pose
	void* m_tag;                       // User data
	btSoftBodyWorldInfo* m_worldInfo;  // World info
	tNoteArray m_notes;                // Notes
	tNodeArray m_nodes;                // Nodes
	tLinkArray m_links;                // Links
	tFaceArray m_faces;                // Faces
	tTetraArray m_tetras;              // Tetras
	tAnchorArray m_anchors;            // Anchors
	tRContactArray m_rcontacts;        // Rigid contacts
	tSContactArray m_scontacts;        // Soft contacts
	tJointArray m_joints;              // Joints
	tMaterialArray m_materials;        // Materials
	btScalar m_timeacc;                // Time accumulator
	btVector3[2] m_bounds;             // Spatial bounds
	bool m_bUpdateRtCst;               // Update runtime constants
	btDbvt m_ndbvt;                    // Nodes tree
	btDbvt m_fdbvt;                    // Faces tree
	btDbvt m_cdbvt;                    // Clusters tree
	tClusterArray m_clusters;          // Clusters

	btAlignedObjectArray!bool m_clusterConnectivity;  //cluster connectivity, for self-collision

	btTransform m_initialWorldTransform;

	btVector3 m_windVelocity;

	btScalar m_restLengthScale;

	//
	// Api
	//

	/* ctor																	*/
	this(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m);

	/* ctor																	*/
	this(btSoftBodyWorldInfo* worldInfo);

	final void initDefaults();

	/* dtor																	*/
	/*virtual*/ ~this();
	/* Check for existing link												*/

	btAlignedObjectArray!int m_userIndexMapping;

	btSoftBodyWorldInfo* getWorldInfo()
	{
		return m_worldInfo;
	}

	///@todo: avoid internal softbody shape hack and move collision code to collision library
	final override /*virtual*/ void setCollisionShape(btCollisionShape collisionShape)
	{
	}

	final bool checkLink(int node0,
				   int node1) const;
	final bool checkLink(const Node* node0,
				   const Node* node1) const;
	/* Check for existring face												*/
	final bool checkFace(int node0,
				   int node1,
				   int node2) const;
	/* Append material														*/
	final Material* appendMaterial();
	/* Append note															*/
	final void appendNote(const(char*) text,
					btVector3 o,
					btVector4 c = btVector4(1, 0, 0, 0),
					Node* n0 = null,
					Node* n1 = null,
					Node* n2 = null,
					Node* n3 = null)
	{
		appendNote(text, o, c, n0, n1, n2, n3);
	}
	final void appendNote(const(char*) text,
					ref const(btVector3) o,
					ref const(btVector4) c,
					Node* n0 = null,
					Node* n1 = null,
					Node* n2 = null,
					Node* n3 = null);
	final void appendNote(const(char*) text,
					ref const(btVector3) o,
					Node* feature);
	final void appendNote(const(char*) text,
					ref const(btVector3) o,
					Link* feature);
	final void appendNote(const(char*) text,
					ref const(btVector3) o,
					Face* feature);
	/* Append node															*/
	final void appendNode(ref const(btVector3) x, btScalar m);
	/* Append link															*/
	final void appendLink(int model = -1, Material* mat = null);
	final void appendLink(int node0,
					int node1,
					Material* mat = null,
					bool bcheckexist = false);
	final void appendLink(Node* node0,
					Node* node1,
					Material* mat = null,
					bool bcheckexist = false);
	/* Append face															*/
	final void appendFace(int model = -1, Material* mat = null);
	final void appendFace(int node0,
					int node1,
					int node2,
					Material* mat = null);
	final void appendTetra(int model, Material* mat);
	//
	final void appendTetra(int node0,
					 int node1,
					 int node2,
					 int node3,
					 Material* mat = null);

	/* Append anchor														*/
	final void appendAnchor(int node,
					  btRigidBody* body, bool disableCollisionBetweenLinkedBodies = false, btScalar influence = 1);
	final void appendAnchor(int node, btRigidBody* body, ref const(btVector3) localPivot, bool disableCollisionBetweenLinkedBodies = false, btScalar influence = 1);
	/* Append linear joint													*/
	final void appendLinearJoint(ref const(LJoint.Specs) specs, Cluster* body0, Body body1);
	final void appendLinearJoint(ref const(LJoint.Specs) specs, Body body_ = Body());
	final void appendLinearJoint(ref const(LJoint.Specs) specs, btSoftBody body_);
	/* Append linear joint													*/
	final void appendAngularJoint(ref const(AJoint.Specs) specs, Cluster* body0, Body body1);
	final void appendAngularJoint(ref const(AJoint.Specs) specs, Body body_ = Body());
	final void appendAngularJoint(ref const(AJoint.Specs) specs, btSoftBody body_);
	/* Add force (or gravity) to the entire body							*/
	final void addForce(ref const(btVector3) force);
	/* Add force (or gravity) to a node of the body							*/
	final void addForce(ref const(btVector3) force,
				  int node);
	/* Add aero force to a node of the body */
	final void addAeroForceToNode(ref const(btVector3) windVelocity, int nodeIndex);

	/* Add aero force to a face of the body */
	final void addAeroForceToFace(ref const(btVector3) windVelocity, int faceIndex);

	/* Add velocity to the entire body										*/
	final void addVelocity(ref const(btVector3) velocity);

	/* Set velocity for the entire body										*/
	final void setVelocity(ref const(btVector3) velocity);

	/* Add velocity to a node of the body									*/
	final void addVelocity(ref const(btVector3) velocity,
					 int node);
	/* Set mass																*/
	final void setMass(int node,
				 btScalar mass);
	/* Get mass																*/
	final btScalar getMass(int node) const;
	/* Get total mass														*/
	final btScalar getTotalMass() const;
	/* Set total mass (weighted by previous masses)							*/
	final void setTotalMass(btScalar mass,
					  bool fromfaces = false);
	/* Set total density													*/
	final void setTotalDensity(btScalar density);
	/* Set volume mass (using tetrahedrons)									*/
	final void setVolumeMass(btScalar mass);
	/* Set volume density (using tetrahedrons)								*/
	final void setVolumeDensity(btScalar density);
	/* Transform															*/
	final void transform(ref const(btTransform) trs);
	/* Translate															*/
	final void translate(ref const(btVector3) trs);
	/* Rotate															*/
	final void rotate(ref const(btQuaternion) rot);
	/* Scale																*/
	final void scale(ref const(btVector3) scl);
	/* Get link resting lengths scale										*/
	final btScalar getRestLengthScale();
	/* Scale resting length of all springs									*/
	final void setRestLengthScale(btScalar restLength);
	/* Set current state as pose											*/
	final void setPose(bool bvolume,
				 bool bframe);
	/* Set current link lengths as resting lengths							*/
	final void resetLinkRestLengths();
	/* Return the volume													*/
	final btScalar getVolume() const;
	/* Cluster count														*/
	final int clusterCount() const;
	/* Cluster center of mass												*/
	final static btVector3 clusterCom(const(Cluster)* cluster);
	final btVector3 clusterCom(int cluster) const;
	/* Cluster velocity at rpos												*/
	final static btVector3 clusterVelocity(const(Cluster)* cluster, ref const(btVector3) rpos);
	/* Cluster impulse														*/
	final static void clusterVImpulse(Cluster* cluster, ref const(btVector3) rpos, ref const(btVector3) impulse);
	final static void clusterDImpulse(Cluster* cluster, ref const(btVector3) rpos, ref const(btVector3) impulse);
	final static void clusterImpulse(Cluster* cluster, ref const(btVector3) rpos, ref const(Impulse) impulse);
	final static void clusterVAImpulse(Cluster* cluster, ref const(btVector3) impulse);
	final static void clusterDAImpulse(Cluster* cluster, ref const(btVector3) impulse);
	final static void clusterAImpulse(Cluster* cluster, ref const(Impulse) impulse);
	final static void clusterDCImpulse(Cluster* cluster, ref const(btVector3) impulse);
	/* Generate bending constraints based on distance in the adjency graph	*/
	final int generateBendingConstraints(int distance,
								   Material* mat = null);
	/* Randomize constraints to reduce solver bias							*/
	final void randomizeConstraints();
	/* Release clusters														*/
	final void releaseCluster(int index);
	final void releaseClusters();
	/* Generate clusters (K-mean)											*/
	///generateClusters with k=0 will create a convex cluster for each tetrahedron or triangle
	///otherwise an approximation will be used (better performance)
	final int generateClusters(int k, int maxiterations = 8192);
	/* Refine																*/
	final void refine(ImplicitFn ifn, btScalar accurary, bool cut);
	/* CutLink																*/
	final bool cutLink(int node0, int node1, btScalar position);
	final bool cutLink(const Node* node0, const Node* node1, btScalar position);

	///Ray casting using rayFrom and rayTo in worldspace, (not direction!)
	final bool rayTest(ref const(btVector3) rayFrom,
				 ref const(btVector3) rayTo,
				 ref sRayCast results);
	/* Solver presets														*/
	final void setSolver(eSolverPresets._ preset);
	/* predictMotion														*/
	final void predictMotion(btScalar dt);
	/* solveConstraints														*/
	final void solveConstraints();
	/* staticSolve															*/
	final void staticSolve(int iterations);
	/* solveCommonConstraints												*/
	final static void solveCommonConstraints(btSoftBody* bodies, int count, int iterations);
	/* solveClusters														*/
	final static void solveClusters(ref const(btAlignedObjectArray!btSoftBody) bodies);
	/* integrateMotion														*/
	final void integrateMotion();
	/* defaultCollisionHandlers												*/
	final void defaultCollisionHandler(const(btCollisionObjectWrapper)* pcoWrap);
	final void defaultCollisionHandler(btSoftBody psb);

	//
	// Functionality to deal with new accelerated solvers.
	//

	/**
	 * Set a wind velocity for interaction with the air.
	 */
	final void setWindVelocity(ref const(btVector3) velocity);

	/**
	 * Return the wind velocity for interaction with the air.
	 */
	final ref const(btVector3) getWindVelocity();

	//
	// Set the solver that handles this soft body
	// Should not be allowed to get out of sync with reality
	// Currently called internally on addition to the world
	void setSoftBodySolver(btSoftBodySolver softBodySolver)
	{
		m_softBodySolver = softBodySolver;
	}

	//
	// Return the solver that handles this soft body
	//
	btSoftBodySolver getSoftBodySolver()
	{
		return m_softBodySolver;
	}

	//
	// Return the solver that handles this soft body
	//
	/*btSoftBodySolver getSoftBodySolver() const
	{
		return m_softBodySolver;
	}*/

	//
	// Cast
	//

	static const(btSoftBody) upcast(const(btCollisionObject) colObj)
	{
		if (colObj.getInternalType() == CollisionObjectTypes.CO_SOFT_BODY)
			return cast(const btSoftBody)colObj;
		return null;
	}
	static btSoftBody upcast(btCollisionObject colObj)
	{
		if (colObj.getInternalType() == CollisionObjectTypes.CO_SOFT_BODY)
			return cast(btSoftBody)colObj;
		return null;
	}

	//
	// ::btCollisionObject
	//

	/*virtual*/ void getAabb(ref btVector3 aabbMin, ref btVector3 aabbMax) const
	{
		aabbMin = m_bounds[0];
		aabbMax = m_bounds[1];
	}
	//
	// Private
	//
	final void pointersToIndices();
	final void indicesToPointers(const(int)* map = null);

	final int rayTest(ref const(btVector3) rayFrom, ref const(btVector3) rayTo,
				ref btScalar mint, ref eFeature._ feature, ref int index, bool bcountonly) const;
	final void initializeFaceTree();
	final btVector3 evaluateCom() const;
	final bool checkContact(const btCollisionObjectWrapper* colObjWrap, ref const(btVector3) x, btScalar margin, ref btSoftBody.sCti cti) const;
	final void updateNormals();
	final void updateBounds();
	final void updatePose();
	final void updateConstants();
	final void updateLinkConstants();
	final void updateArea(bool averageArea = true);
	final void initializeClusters();
	final void updateClusters();
	final void cleanupClusters();
	final void prepareClusters(int iterations);
	final void solveClusters(btScalar sor);
	final void applyClusters(bool drift);
	final void dampClusters();
	final void applyForces();
	final static void PSolve_Anchors(btSoftBody psb, btScalar kst, btScalar ti);
	final static void PSolve_RContacts(btSoftBody psb, btScalar kst, btScalar ti);
	final static void PSolve_SContacts(btSoftBody psb, btScalar, btScalar ti);
	final static void PSolve_Links(btSoftBody psb, btScalar kst, btScalar ti);
	final static void VSolve_Links(btSoftBody psb, btScalar kst);
	final static psolver_t getSolver(ePSolver._ solver);
	final static vsolver_t getSolver(eVSolver._ solver);

	final override /*virtual*/ int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	pragma(mangle, "?serialize@btSoftBody@@UEBAPEBDPEAXPEAVbtSerializer@@@Z")
	final override /*virtual*/ const(char*) serialize(void* dataBuffer, btSerializer serializer) const;

	//final override /*virtual*/ void serializeSingleObject(class btSerializer serializer) const;
};
