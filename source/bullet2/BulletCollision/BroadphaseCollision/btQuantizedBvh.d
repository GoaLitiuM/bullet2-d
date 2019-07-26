module bullet2.BulletCollision.BroadphaseCollision.btQuantizedBvh;

extern (C++):

//#define DEBUG_CHECK_DEQUANTIZATION 1
/*#ifdef DEBUG_CHECK_DEQUANTIZATION
#ifdef __SPU__
#define printf spu_printf
#endif  //__SPU__

#include <stdio.h>
#include <stdlib.h>
#endif  //DEBUG_CHECK_DEQUANTIZATION
*/

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btAlignedAllocator;
import bullet2.LinearMath.btSerializer;
import bullet2.LinearMath.btAlignedObjectArray;


version (BT_USE_DOUBLE_PRECISION)
{
    alias btQuantizedBvhData = btQuantizedBvhDoubleData;
    alias btOptimizedBvhNodeData = btOptimizedBvhNodeDoubleData;
    enum btQuantizedBvhDataName = "btQuantizedBvhDoubleData";
}
else
{
    alias btQuantizedBvhData = btQuantizedBvhFloatData;
    alias btOptimizedBvhNodeData = btOptimizedBvhNodeFloatData;
    enum btQuantizedBvhDataName = "btQuantizedBvhFloatData";
}

//http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vclang/html/vclrf__m128.asp

//Note: currently we have 16 bytes per quantized node
enum MAX_SUBTREE_SIZE_IN_BYTES = 2048;

// 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
// actually) triangles each (since the sign bit is reserved
enum MAX_NUM_PARTS_IN_BITS = 10;

///btQuantizedBvhNode is a compressed aabb node, 16 bytes.
///Node can be used for leafnode or internal node. Leafnodes can point to 32-bit triangle index (non-negative range).
extern (C++, struct)
align(16) struct btQuantizedBvhNode
{
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	//12 bytes
	ushort[3] m_quantizedAabbMin;
	ushort[3] m_quantizedAabbMax;
	//4 bytes
	int m_escapeIndexOrTriangleIndex;

	bool isLeafNode() const
	{
		//skipindex is negative (internal node), triangleindex >=0 (leafnode)
		return (m_escapeIndexOrTriangleIndex >= 0);
	}
	int getEscapeIndex() const
	{
		btAssert(!isLeafNode());
		return -m_escapeIndexOrTriangleIndex;
	}
	int getTriangleIndex() const
	{
		btAssert(isLeafNode());
		uint x = 0;
		uint y = (~(x & 0)) << (31 - MAX_NUM_PARTS_IN_BITS);
		// Get only the lower bits where the triangle index is stored
		return (m_escapeIndexOrTriangleIndex & ~(y));
	}
	int getPartId() const
	{
		btAssert(isLeafNode());
		// Get only the highest bits where the part index is stored
		return (m_escapeIndexOrTriangleIndex >> (31 - MAX_NUM_PARTS_IN_BITS));
	}
};

/// btOptimizedBvhNode contains both internal and leaf node information.
/// Total node size is 44 bytes / node. You can use the compressed version of 16 bytes.
extern (C++, struct)
align(16) struct btOptimizedBvhNode
{
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	//32 bytes
	btVector3 m_aabbMinOrg;
	btVector3 m_aabbMaxOrg;

	//4
	int m_escapeIndex;

	//8
	//for child nodes
	int m_subPart;
	int m_triangleIndex;

	//pad the size to 64 bytes
	char[20] m_padding;
};

///btBvhSubtreeInfo provides info to gather a subtree of limited size
extern (C++, class)
align(16) struct btBvhSubtreeInfo
{
public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	//12 bytes
	ushort[3] m_quantizedAabbMin;
	ushort[3] m_quantizedAabbMax;
	//4 bytes, points to the root of the subtree
	int m_rootNodeIndex;
	//4 bytes
	int m_subtreeSize;
	int[3] m_padding;

	/*this()
	{
		//memset(&m_padding[0], 0, sizeof(m_padding));
	}*/

	void setAabbFromQuantizeNode(ref const(btQuantizedBvhNode) quantizedNode)
	{
		m_quantizedAabbMin[0] = quantizedNode.m_quantizedAabbMin[0];
		m_quantizedAabbMin[1] = quantizedNode.m_quantizedAabbMin[1];
		m_quantizedAabbMin[2] = quantizedNode.m_quantizedAabbMin[2];
		m_quantizedAabbMax[0] = quantizedNode.m_quantizedAabbMax[0];
		m_quantizedAabbMax[1] = quantizedNode.m_quantizedAabbMax[1];
		m_quantizedAabbMax[2] = quantizedNode.m_quantizedAabbMax[2];
	}
};

class btNodeOverlapCallback
{
public:
	/*virtual*/ ~this(){};

	abstract /*virtual*/ void processNode(int subPart, int triangleIndex);
};

///for code readability:
alias NodeArray = btAlignedObjectArray!btOptimizedBvhNode;
alias QuantizedNodeArray = btAlignedObjectArray!btQuantizedBvhNode;
alias BvhSubtreeInfoArray = btAlignedObjectArray!btBvhSubtreeInfo;

///The btQuantizedBvh class stores an AABB tree that can be quickly traversed on CPU and Cell SPU.
///It is used by the btBvhTriangleMeshShape as midphase.
///It is recommended to use quantization for better performance and lower memory requirements.
extern (C++, class)
align(16) class btQuantizedBvh
{
public:
	enum btTraversalMode
	{
		TRAVERSAL_STACKLESS = 0,
		TRAVERSAL_STACKLESS_CACHE_FRIENDLY,
		TRAVERSAL_RECURSIVE
	};

protected:
	btVector3 m_bvhAabbMin;
	btVector3 m_bvhAabbMax;
	btVector3 m_bvhQuantization;

	int m_bulletVersion;  //for serialization versioning. It could also be used to detect endianess.

	int m_curNodeIndex;
	//quantization data
	bool m_useQuantization;

	NodeArray m_leafNodes;
	NodeArray m_contiguousNodes;
	QuantizedNodeArray m_quantizedLeafNodes;
	QuantizedNodeArray m_quantizedContiguousNodes;

	btTraversalMode m_traversalMode;
	BvhSubtreeInfoArray m_SubtreeHeaders;

	//This is only used for serialization so we don't have to add serialization directly to btAlignedObjectArray
	/*mutable*/ int m_subtreeHeaderCount;

	///two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
	///this might be refactored into a /*virtual*/, it is usually not calculated at run-time
	void setInternalNodeAabbMin(int nodeIndex, ref const(btVector3) aabbMin)
	{
		if (m_useQuantization)
		{
			quantize(&m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0], aabbMin, 0);
		}
		else
		{
			m_contiguousNodes[nodeIndex].m_aabbMinOrg = aabbMin;
		}
	}
	void setInternalNodeAabbMax(int nodeIndex, ref const(btVector3) aabbMax)
	{
		if (m_useQuantization)
		{
			quantize(&m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0], aabbMax, 1);
		}
		else
		{
			m_contiguousNodes[nodeIndex].m_aabbMaxOrg = aabbMax;
		}
	}

	btVector3 getAabbMin(int nodeIndex) //const
	{
		if (m_useQuantization)
		{
			return unQuantize(&m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMin[0]);
		}
		//non-quantized
		return m_leafNodes[nodeIndex].m_aabbMinOrg;
	}
	btVector3 getAabbMax(int nodeIndex) //const
	{
		if (m_useQuantization)
		{
			return unQuantize(&m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMax[0]);
		}
		//non-quantized
		return m_leafNodes[nodeIndex].m_aabbMaxOrg;
	}

	void setInternalNodeEscapeIndex(int nodeIndex, int escapeIndex)
	{
		if (m_useQuantization)
		{
			m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = -escapeIndex;
		}
		else
		{
			m_contiguousNodes[nodeIndex].m_escapeIndex = escapeIndex;
		}
	}

	void mergeInternalNodeAabb(int nodeIndex, ref const(btVector3) newAabbMin, ref const(btVector3) newAabbMax)
	{
		if (m_useQuantization)
		{
			ushort[3] quantizedAabbMin;
			ushort[3] quantizedAabbMax;
			quantize(quantizedAabbMin.ptr, newAabbMin, 0);
			quantize(quantizedAabbMax.ptr, newAabbMax, 1);
			for (int i = 0; i < 3; i++)
			{
				if (m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] > quantizedAabbMin[i])
					m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] = quantizedAabbMin[i];

				if (m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] < quantizedAabbMax[i])
					m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] = quantizedAabbMax[i];
			}
		}
		else
		{
			//non-quantized
			m_contiguousNodes[nodeIndex].m_aabbMinOrg.setMin(newAabbMin);
			m_contiguousNodes[nodeIndex].m_aabbMaxOrg.setMax(newAabbMax);
		}
	}

	final void swapLeafNodes(int firstIndex, int secondIndex);

	final void assignInternalNodeFromLeafNode(int internalNode, int leafNodeIndex);

protected:
	final void buildTree(int startIndex, int endIndex);

	final int calcSplittingAxis(int startIndex, int endIndex);

	final int sortAndCalcSplittingIndex(int startIndex, int endIndex, int splitAxis);

	final void walkStacklessTree(btNodeOverlapCallback nodeCallback, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;

	final void walkStacklessQuantizedTreeAgainstRay(btNodeOverlapCallback nodeCallback, ref const(btVector3) raySource, ref const(btVector3) rayTarget, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax, int startNodeIndex, int endNodeIndex) const;
	final void walkStacklessQuantizedTree(btNodeOverlapCallback nodeCallback, ushort* quantizedQueryAabbMin, ushort* quantizedQueryAabbMax, int startNodeIndex, int endNodeIndex) const;
	final void walkStacklessTreeAgainstRay(btNodeOverlapCallback nodeCallback, ref const(btVector3) raySource, ref const(btVector3) rayTarget, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax, int startNodeIndex, int endNodeIndex) const;

	///tree traversal designed for small-memory processors like PS3 SPU
	final void walkStacklessQuantizedTreeCacheFriendly(btNodeOverlapCallback nodeCallback, ushort* quantizedQueryAabbMin, ushort* quantizedQueryAabbMax) const;

	///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
	final void walkRecursiveQuantizedTreeAgainstQueryAabb(const btQuantizedBvhNode* currentNode, btNodeOverlapCallback* nodeCallback, ushort* quantizedQueryAabbMin, ushort* quantizedQueryAabbMax) const;

	///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
	final void walkRecursiveQuantizedTreeAgainstQuantizedTree(const btQuantizedBvhNode* treeNodeA, const btQuantizedBvhNode* treeNodeB, btNodeOverlapCallback* nodeCallback) const;

	final void updateSubtreeHeaders(int leftChildNodexIndex, int rightChildNodexIndex);

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	final this();

	/*virtual*/ ~this();

	///***************************************** expert/internal use only *************************
	final void setQuantizationValues(ref const(btVector3) bvhAabbMin, ref const(btVector3) bvhAabbMax, btScalar quantizationMargin = btScalar(1.0));
	ref QuantizedNodeArray getLeafNodeArray() { return m_quantizedLeafNodes; }
	///buildInternal is expert use only: assumes that setQuantizationValues and LeafNodeArray are initialized
	final void buildInternal();
	///***************************************** expert/internal use only *************************

	final void reportAabbOverlappingNodex(btNodeOverlapCallback nodeCallback, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;
	final void reportRayOverlappingNodex(btNodeOverlapCallback nodeCallback, ref const(btVector3) raySource, ref const(btVector3) rayTarget) const;
	final void reportBoxCastOverlappingNodex(btNodeOverlapCallback nodeCallback, ref const(btVector3) raySource, ref const(btVector3) rayTarget, ref const(btVector3) aabbMin, ref const(btVector3) aabbMax) const;

	/*SIMD_FORCE_INLINE*/ void quantize(ushort* out_, ref const(btVector3) point, int isMax) const
	{
		btAssert(m_useQuantization);

		btAssert(point.getX() <= m_bvhAabbMax.getX());
		btAssert(point.getY() <= m_bvhAabbMax.getY());
		btAssert(point.getZ() <= m_bvhAabbMax.getZ());

		btAssert(point.getX() >= m_bvhAabbMin.getX());
		btAssert(point.getY() >= m_bvhAabbMin.getY());
		btAssert(point.getZ() >= m_bvhAabbMin.getZ());

		btVector3 v = (point - m_bvhAabbMin) * m_bvhQuantization;
		///Make sure rounding is done in a way that unQuantize(quantizeWithClamp(...)) is conservative
		///end-points always set the first bit, so that they are sorted properly (so that neighbouring AABBs overlap properly)
		///@todo: double-check this
		if (isMax)
		{
			out_[0] = cast(ushort)((cast(ushort)(v.getX() + btScalar(1.)) | 1));
			out_[1] = cast(ushort)((cast(ushort)(v.getY() + btScalar(1.)) | 1));
			out_[2] = cast(ushort)((cast(ushort)(v.getZ() + btScalar(1.)) | 1));
		}
		else
		{
			out_[0] = cast(ushort)((cast(ushort)(v.getX()) & 0xfffe));
			out_[1] = cast(ushort)((cast(ushort)(v.getY()) & 0xfffe));
			out_[2] = cast(ushort)((cast(ushort)(v.getZ()) & 0xfffe));
		}

/+#ifdef DEBUG_CHECK_DEQUANTIZATION
		btVector3 newPoint = unQuantize(out);
		if (isMax)
		{
			if (newPoint.getX() < point.getX())
			{
				printf("unconservative X, diffX = %f, oldX=%f,newX=%f\n", newPoint.getX() - point.getX(), newPoint.getX(), point.getX());
			}
			if (newPoint.getY() < point.getY())
			{
				printf("unconservative Y, diffY = %f, oldY=%f,newY=%f\n", newPoint.getY() - point.getY(), newPoint.getY(), point.getY());
			}
			if (newPoint.getZ() < point.getZ())
			{
				printf("unconservative Z, diffZ = %f, oldZ=%f,newZ=%f\n", newPoint.getZ() - point.getZ(), newPoint.getZ(), point.getZ());
			}
		}
		else
		{
			if (newPoint.getX() > point.getX())
			{
				printf("unconservative X, diffX = %f, oldX=%f,newX=%f\n", newPoint.getX() - point.getX(), newPoint.getX(), point.getX());
			}
			if (newPoint.getY() > point.getY())
			{
				printf("unconservative Y, diffY = %f, oldY=%f,newY=%f\n", newPoint.getY() - point.getY(), newPoint.getY(), point.getY());
			}
			if (newPoint.getZ() > point.getZ())
			{
				printf("unconservative Z, diffZ = %f, oldZ=%f,newZ=%f\n", newPoint.getZ() - point.getZ(), newPoint.getZ(), point.getZ());
			}
		}
#endif  //DEBUG_CHECK_DEQUANTIZATION+/
	}

	/*SIMD_FORCE_INLINE*/ void quantizeWithClamp(ushort* out_, ref const(btVector3) point2, int isMax) const
	{
		btAssert(m_useQuantization);

		btVector3 clampedPoint = btVector3(point2);
		clampedPoint.setMax(m_bvhAabbMin);
		clampedPoint.setMin(m_bvhAabbMax);

		quantize(out_, clampedPoint, isMax);
	}

	/*SIMD_FORCE_INLINE*/ btVector3 unQuantize(const ushort* vecIn) const
	{
		btVector3 vecOut;
		vecOut.setValue(
			cast(btScalar)(vecIn[0]) / (m_bvhQuantization.getX()),
			cast(btScalar)(vecIn[1]) / (m_bvhQuantization.getY()),
			cast(btScalar)(vecIn[2]) / (m_bvhQuantization.getZ()));
		vecOut += m_bvhAabbMin;
		return vecOut;
	}

	///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this is only implemented for quantized trees.
	void setTraversalMode(btTraversalMode traversalMode)
	{
		m_traversalMode = traversalMode;
	}

	/*SIMD_FORCE_INLINE*/ ref QuantizedNodeArray getQuantizedNodeArray()
	{
		return m_quantizedContiguousNodes;
	}

	/*SIMD_FORCE_INLINE*/ ref BvhSubtreeInfoArray getSubtreeInfoArray()
	{
		return m_SubtreeHeaders;
	}

	////////////////////////////////////////////////////////////////////

	/////Calculate space needed to store BVH for serialization
	final uint calculateSerializeBufferSize() const;

	/// Data buffer MUST be 16 byte aligned
	/*virtual*/ bool serialize(void* o_alignedDataBuffer, uint i_dataBufferSize, bool i_swapEndian) const;

	///deSerializeInPlace loads and initializes a BVH from a buffer in memory 'in place'
	final static btQuantizedBvh* deSerializeInPlace(void* i_alignedDataBuffer, uint i_dataBufferSize, bool i_swapEndian);

	final static uint getAlignmentSerializationPadding();
	//////////////////////////////////////////////////////////////////////

	/*virtual*/ int calculateSerializeBufferSizeNew() const
    {
        return btQuantizedBvhData.sizeof;
    }

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	final /*virtual*/ const(char*) serialize(void* dataBuffer, btSerializer serializer) const;

	final /*virtual*/ void deSerializeFloat(ref btQuantizedBvhFloatData quantizedBvhFloatData);

	final /*virtual*/ void deSerializeDouble(ref btQuantizedBvhDoubleData quantizedBvhDoubleData);

	////////////////////////////////////////////////////////////////////

	/*SIMD_FORCE_INLINE*/ bool isQuantized()
	{
		return m_useQuantization;
	}

private:
	// Special "copy" constructor that allows for in-place deserialization
	// Prevents btVector3's default constructor from being called, but doesn't inialize much else
	// ownsMemory should most likely be false if deserializing, and if you are not, don't call this (it also changes the function signature, which we need)
	this(ref btQuantizedBvh other, bool ownsMemory);
};

// clang-format off
// parser needs * with the name
struct btBvhSubtreeInfoData
{
	int m_rootNodeIndex;
	int m_subtreeSize;
	ushort[3] m_quantizedAabbMin;
	ushort[3] m_quantizedAabbMax;
};

struct btOptimizedBvhNodeFloatData
{
	btVector3FloatData m_aabbMinOrg;
	btVector3FloatData m_aabbMaxOrg;
	int m_escapeIndex;
	int m_subPart;
	int m_triangleIndex;
	char[4] m_pad;
};

struct btOptimizedBvhNodeDoubleData
{
	btVector3DoubleData m_aabbMinOrg;
	btVector3DoubleData m_aabbMaxOrg;
	int m_escapeIndex;
	int m_subPart;
	int m_triangleIndex;
	char[4] m_pad;
};


struct btQuantizedBvhNodeData
{
	ushort[3] m_quantizedAabbMin;
	ushort[3] m_quantizedAabbMax;
	int	m_escapeIndexOrTriangleIndex;
};

struct	btQuantizedBvhFloatData
{
	btVector3FloatData			m_bvhAabbMin;
	btVector3FloatData			m_bvhAabbMax;
	btVector3FloatData			m_bvhQuantization;
	int					m_curNodeIndex;
	int					m_useQuantization;
	int					m_numContiguousLeafNodes;
	int					m_numQuantizedContiguousNodes;
	btOptimizedBvhNodeFloatData	*m_contiguousNodesPtr;
	btQuantizedBvhNodeData		*m_quantizedContiguousNodesPtr;
	btBvhSubtreeInfoData	*m_subTreeInfoPtr;
	int					m_traversalMode;
	int					m_numSubtreeHeaders;

};

struct	btQuantizedBvhDoubleData
{
	btVector3DoubleData			m_bvhAabbMin;
	btVector3DoubleData			m_bvhAabbMax;
	btVector3DoubleData			m_bvhQuantization;
	int							m_curNodeIndex;
	int							m_useQuantization;
	int							m_numContiguousLeafNodes;
	int							m_numQuantizedContiguousNodes;
	btOptimizedBvhNodeDoubleData	*m_contiguousNodesPtr;
	btQuantizedBvhNodeData			*m_quantizedContiguousNodesPtr;

	int							m_traversalMode;
	int							m_numSubtreeHeaders;
	btBvhSubtreeInfoData		*m_subTreeInfoPtr;
};
