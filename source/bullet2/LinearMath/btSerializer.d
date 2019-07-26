module bullet2.LinearMath.btSerializer;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btHashMap;
import bullet2.LinearMath.btAlignedAllocator;
import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.BulletCollision.BroadphaseCollision.btQuantizedBvh;
import bullet2.BulletCollision.CollisionShapes.btCollisionShape;
import bullet2.BulletDynamics.Dynamics.btRigidBody;
import bullet2.BulletDynamics.Dynamics.btDynamicsWorld;
import bullet2.BulletDynamics.ConstraintSolver.btTypedConstraint;
import bullet2.BulletCollision.CollisionDispatch.btCollisionObject;
import bullet2.BulletSoftBody.btSoftBody;
import bullet2.BulletSoftBody.btSoftBodyData;

import core.stdc.string;

/*#if !defined(__CELLOS_LV2__) && !defined(__MWERKS__)
#include <memory.h>
#endif
#include <string.h>*/

pragma(mangle, "?sBulletDNAstr@@3PADA")
extern char* sBulletDNAstr;
extern int sBulletDNAlen;
pragma(mangle, "?sBulletDNAstr64@@3PADA")
extern char* sBulletDNAstr64;
extern int sBulletDNAlen64;

/*SIMD_FORCE_INLINE*/ int btStrLen(const(char*) str)
{
	if (!str)
		return (0);
	int len = 0;

	char* ptr = cast(char*)str;
	while (*ptr != 0)
	{
		ptr++;
		len++;
	}

	return len;
}

class btChunk
{
public:
	int m_chunkCode;
	int m_length;
	void* m_oldPtr;
	int m_dna_nr;
	int m_number;
};

enum btSerializationFlags
{
	BT_SERIALIZE_NO_BVH = 1,
	BT_SERIALIZE_NO_TRIANGLEINFOMAP = 2,
	BT_SERIALIZE_NO_DUPLICATE_ASSERT = 4,
	BT_SERIALIZE_CONTACT_MANIFOLDS = 8,
};

class btSerializer
{
public:
	/*virtual*/ ~this() {}

	/*virtual*/ abstract const(char*) getBufferPointer() const;

	/*virtual*/ abstract int getCurrentBufferSize() const;

	/*virtual*/ abstract btChunk allocate(size_t size, int numElements);

	/*virtual*/ abstract void finalizeChunk(btChunk chunk, const(char*) structType, int chunkCode, void* oldPtr);

	/*virtual*/ abstract void* findPointer(void* oldPtr);

	/*virtual*/ abstract void* getUniquePointer(void* oldPtr);

	/*virtual*/ abstract void startSerialization();

	/*virtual*/ abstract void finishSerialization();

	/*virtual*/ abstract const(char*) findNameForPointer(const(void*) ptr) const;

	/*virtual*/ abstract void registerNameForPointer(const(void*) ptr, const(char*) name);

	/*virtual*/ abstract void serializeName(const(char*) ptr);

	/*virtual*/ abstract int getSerializationFlags() const;

	/*virtual*/ abstract void setSerializationFlags(int flags);

	/*virtual*/ abstract int getNumChunks() const;

	/*virtual*/ abstract const(btChunk) getChunk(int chunkIndex) const;
};

enum BT_HEADER_LENGTH = 12;
version (BigEndian) //#if defined(__sgi) || defined(__sparc) || defined(__sparc__) || defined(__PPC__) || defined(__ppc__) || defined(__BIG_ENDIAN__)
{
	auto BT_MAKE_ID(char a, char b, char c, char d)
	{
		return (cast(int)(a) << 24 | cast(int)(b) << 16 | (c) << 8 | (d));
	}
}
else
{
	auto BT_MAKE_ID(char a, char b, char c, char d)
	{
		return (cast(int)(d) << 24 | cast(int)(c) << 16 | (b) << 8 | (a));
	}
}


enum BT_MULTIBODY_CODE = BT_MAKE_ID('M', 'B', 'D', 'Y');
enum BT_MB_LINKCOLLIDER_CODE = BT_MAKE_ID('M', 'B', 'L', 'C');
enum BT_SOFTBODY_CODE = BT_MAKE_ID('S', 'B', 'D', 'Y');
enum BT_COLLISIONOBJECT_CODE = BT_MAKE_ID('C', 'O', 'B', 'J');
enum BT_RIGIDBODY_CODE = BT_MAKE_ID('R', 'B', 'D', 'Y');
enum BT_CONSTRAINT_CODE = BT_MAKE_ID('C', 'O', 'N', 'S');
enum BT_BOXSHAPE_CODE = BT_MAKE_ID('B', 'O', 'X', 'S');
enum BT_QUANTIZED_BVH_CODE = BT_MAKE_ID('Q', 'B', 'V', 'H');
enum BT_TRIANLGE_INFO_MAP = BT_MAKE_ID('T', 'M', 'A', 'P');
enum BT_SHAPE_CODE = BT_MAKE_ID('S', 'H', 'A', 'P');
enum BT_ARRAY_CODE = BT_MAKE_ID('A', 'R', 'A', 'Y');
enum BT_SBMATERIAL_CODE = BT_MAKE_ID('S', 'B', 'M', 'T');
enum BT_SBNODE_CODE = BT_MAKE_ID('S', 'B', 'N', 'D');
enum BT_DYNAMICSWORLD_CODE = BT_MAKE_ID('D', 'W', 'L', 'D');
enum BT_CONTACTMANIFOLD_CODE = BT_MAKE_ID('C', 'O', 'N', 'T');
enum BT_DNA_CODE = BT_MAKE_ID('D', 'N', 'A', '1');

struct btPointerUid
{
	union {
		void* m_ptr;
		int[2] m_uniqueIds;
	};
};

/+struct btBulletSerializedArrays
{
	/*this()
	{
	}*/
	btAlignedObjectArray!(btQuantizedBvhDoubleData*) m_bvhsDouble;
	btAlignedObjectArray!(btQuantizedBvhFloatData*) m_bvhsFloat;
	btAlignedObjectArray!(btCollisionShapeData*) m_colShapeData;
	btAlignedObjectArray!(btDynamicsWorldDoubleData*) m_dynamicWorldInfoDataDouble;
	btAlignedObjectArray!(btDynamicsWorldFloatData*) m_dynamicWorldInfoDataFloat;
	btAlignedObjectArray!(btRigidBodyDoubleData*) m_rigidBodyDataDouble;
	btAlignedObjectArray!(btRigidBodyFloatData*) m_rigidBodyDataFloat;
	btAlignedObjectArray!(btCollisionObjectDoubleData*) m_collisionObjectDataDouble;
	btAlignedObjectArray!(btCollisionObjectFloatData*) m_collisionObjectDataFloat;
	btAlignedObjectArray!(btTypedConstraintFloatData*) m_constraintDataFloat;
	btAlignedObjectArray!(btTypedConstraintDoubleData*) m_constraintDataDouble;
	btAlignedObjectArray!(btTypedConstraintData*) m_constraintData;  //for backwards compatibility
	btAlignedObjectArray!(btSoftBodyFloatData*) m_softBodyFloatData;
	btAlignedObjectArray!(btSoftBodyDoubleData*) m_softBodyDoubleData;
};+/

///The btDefaultSerializer is the main Bullet serialization class.
///The constructor takes an optional argument for backwards compatibility, it is recommended to leave this empty/zero.
class btDefaultSerializer : btSerializer
{
protected:
	btAlignedObjectArray!(char*) mTypes;
	btAlignedObjectArray!(short*) mStructs;
	btAlignedObjectArray!(short) mTlens;
	btHashMap!(btHashInt, int) mStructReverse;
	btHashMap!(btHashString, int) mTypeLookup;

	btHashMap!(btHashPtr, void*) m_chunkP;

	btHashMap!(btHashPtr, char*) m_nameMap;

	btHashMap!(btHashPtr, btPointerUid) m_uniquePointers;
	int m_uniqueIdGenerator;

	int m_totalSize;
	char* m_buffer;
	bool m_ownsBuffer;
	int m_currentSize;
	void* m_dna;
	int m_dnaLength;

	int m_serializationFlags;

	btAlignedObjectArray!(btChunk) m_chunkPtrs;

protected:
	final override /*virtual*/ void* findPointer(void* oldPtr)
	{
		btHashPtr key = btHashPtr(cast(const(void*))oldPtr);
		void** ptr = m_chunkP.find(key);
		if (ptr && *ptr)
			return *ptr;
		return null;
	}

	/*virtual*/ void writeDNA()
	{
		btChunk dnaChunk = allocate(m_dnaLength, 1);
		memcpy(dnaChunk.m_oldPtr, m_dna, m_dnaLength);
		finalizeChunk(dnaChunk, "DNA1", BT_DNA_CODE, m_dna);
	}

	int getReverseType(const(char*) type) //const
	{
		btHashString key = btHashString(type);
		const int* valuePtr = mTypeLookup.find(key);
		if (valuePtr)
			return *valuePtr;

		return -1;
	}

	void initDNA(const(char*) bdnaOrg, int dnalen)
	{
		///was already initialized
		if (m_dna)
			return;

		int littleEndian = 1;
		littleEndian = (cast(char*)&littleEndian)[0];

		m_dna = btAlignedAlloc(dnalen, 16);
		memcpy(m_dna, bdnaOrg, dnalen);
		m_dnaLength = dnalen;

		int* intPtr = null;
		short* shtPtr = null;
		char* cp = null;
		int dataLen = 0;
		intPtr = cast(int*)m_dna;

		/*
				SDNA (4 bytes) (magic number)
				NAME (4 bytes)
				<nr> (4 bytes) amount of names (int)
				<string>
				<string>
			*/

		if (strncmp(cast(const(char*))m_dna, "SDNA", 4) == 0)
		{
			// skip ++ NAME
			intPtr++;
			intPtr++;
		}

		// Parse names
		if (!littleEndian)
			*intPtr = btSwapEndian(*intPtr);

		dataLen = *intPtr;

		intPtr++;

		cp = cast(char*)intPtr;
		int i;
		for (i = 0; i < dataLen; i++)
		{
			while (*cp) cp++;
			cp++;
		}
		cp = btAlignPointer(cp, 4);

		/*
				TYPE (4 bytes)
				<nr> amount of types (int)
				<string>
				<string>
			*/

		intPtr = cast(int*)cp;
		btAssert(strncmp(cp, "TYPE", 4) == 0);
		intPtr++;

		if (!littleEndian)
			*intPtr = btSwapEndian(*intPtr);

		dataLen = *intPtr;
		intPtr++;

		cp = cast(char*)intPtr;
		for (i = 0; i < dataLen; i++)
		{
			mTypes.push_back(cp);
			while (*cp) cp++;
			cp++;
		}

		cp = btAlignPointer(cp, 4);

		/*
				TLEN (4 bytes)
				<len> (short) the lengths of types
				<len>
			*/

		// Parse type lens
		intPtr = cast(int*)cp;
		btAssert(strncmp(cp, "TLEN", 4) == 0);
		intPtr++;

		dataLen = cast(int)mTypes.size();

		shtPtr = cast(short*)intPtr;
		for (i = 0; i < dataLen; i++, shtPtr++)
		{
			if (!littleEndian)
				shtPtr[0] = btSwapEndian(shtPtr[0]);
			mTlens.push_back(shtPtr[0]);
		}

		if (dataLen & 1) shtPtr++;

		/*
				STRC (4 bytes)
				<nr> amount of structs (int)
				<typenr>
				<nr_of_elems>
				<typenr>
				<namenr>
				<typenr>
				<namenr>
			*/

		intPtr = cast(int*)shtPtr;
		cp = cast(char*)intPtr;
		btAssert(strncmp(cp, "STRC", 4) == 0);
		intPtr++;

		if (!littleEndian)
			*intPtr = btSwapEndian(*intPtr);
		dataLen = *intPtr;
		intPtr++;

		shtPtr = cast(short*)intPtr;
		for (i = 0; i < dataLen; i++)
		{
			mStructs.push_back(shtPtr);

			if (!littleEndian)
			{
				shtPtr[0] = btSwapEndian(shtPtr[0]);
				shtPtr[1] = btSwapEndian(shtPtr[1]);

				int len = shtPtr[1];
				shtPtr += 2;

				for (int a = 0; a < len; a++, shtPtr += 2)
				{
					shtPtr[0] = btSwapEndian(shtPtr[0]);
					shtPtr[1] = btSwapEndian(shtPtr[1]);
				}
			}
			else
			{
				shtPtr += (2 * shtPtr[1]) + 2;
			}
		}

		// build reverse lookups
		for (i = 0; i < cast(int)mStructs.size(); i++)
		{
			short* strc = mStructs.at(i);
			mStructReverse.insert(btHashInt(strc[0]), i);
			mTypeLookup.insert(btHashString(mTypes[strc[0]]), i);
		}
	}

public:
	btHashMap!(btHashPtr, void*) m_skipPointers;

	this(int totalSize = 0, char* buffer = null)
	{
		m_uniqueIdGenerator = 0;
		m_totalSize = totalSize;
		m_currentSize = 0;
		m_dna = null;
		m_dnaLength = 0;
		m_serializationFlags = 0;

		if (buffer == null)
		{
			m_buffer = m_totalSize ? cast(char*)btAlignedAlloc(totalSize, 16) : null;
			m_ownsBuffer = true;
		}
		else
		{
			m_buffer = buffer;
			m_ownsBuffer = false;
		}

		const bool VOID_IS_8 = ((void*).sizeof == 8);

		version (BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)
		{
			if (VOID_IS_8)
			{
				version (Win64)
					initDNA(cast(const(char*))sBulletDNAstr64, sBulletDNAlen64);
				else
					btAssert(0);
			}
			else
			{
				version (Win64)
					btAssert(0);
				else
					initDNA(cast(const(char*))sBulletDNAstr, sBulletDNAlen);
			}
		}
		else
		{
			if (VOID_IS_8)
			{
				initDNA(cast(const(char*))sBulletDNAstr64, sBulletDNAlen64);
			}
			else
			{
				initDNA(cast(const(char*))sBulletDNAstr, sBulletDNAlen);
			}
		}
	}

	/*virtual*/ ~this()
	{
		if (m_buffer && m_ownsBuffer)
			btAlignedFree(m_buffer);
		if (m_dna)
			btAlignedFree(m_dna);
	}

	static int getMemoryDnaSizeInBytes()
	{
		const bool VOID_IS_8 = ((void*).sizeof == 8);

		if (VOID_IS_8)
			return sBulletDNAlen64;
		else
			return sBulletDNAlen;
	}
	static const(char*) getMemoryDna()
	{
		const bool VOID_IS_8 = ((void*).sizeof == 8);
		if (VOID_IS_8)
			return cast(const(char*))sBulletDNAstr64;
		else
			return cast(const(char*))sBulletDNAstr;
	}

	void insertHeader()
	{
		writeHeader(m_buffer);
		m_currentSize += BT_HEADER_LENGTH;
	}

	void writeHeader(char* buffer) const
	{
		version (BT_USE_DOUBLE_PRECISION)
			buffer[0..7] = "BULLETd";//memcpy(buffer, "BULLETd", 7);
		else
			buffer[0..7] = "BULLETf";//memcpy(buffer, "BULLETf", 7);

		int littleEndian = 1;
		littleEndian = (cast(char*)&littleEndian)[0];

		if ((void*).sizeof == 8)
		{
			buffer[7] = '-';
		}
		else
		{
			buffer[7] = '_';
		}

		if (littleEndian)
		{
			buffer[8] = 'v';
		}
		else
		{
			buffer[8] = 'V';
		}

		buffer[9] = '2';
		buffer[10] = '8';
		buffer[11] = '8';
	}

	final override /*virtual*/ void startSerialization()
	{
		m_uniqueIdGenerator = 1;
		if (m_totalSize)
		{
			char* buffer = internalAlloc(BT_HEADER_LENGTH);
			writeHeader(buffer);
		}
	}

	final override /*virtual*/ void finishSerialization()
	{
		writeDNA();

		//if we didn't pre-allocate a buffer, we need to create a contiguous buffer now
		int mysize = 0;
		if (!m_totalSize)
		{
			if (m_buffer)
				btAlignedFree(m_buffer);

			m_currentSize += BT_HEADER_LENGTH;
			m_buffer = cast(char*)btAlignedAlloc(m_currentSize, 16);

			char* currentPtr = m_buffer;
			writeHeader(m_buffer);
			currentPtr += BT_HEADER_LENGTH;
			mysize += BT_HEADER_LENGTH;
			for (int i = 0; i < m_chunkPtrs.size(); i++)
			{
				int curLength = cast(int)btChunk.sizeof + m_chunkPtrs[i].m_length;
				memcpy(cast(void*)currentPtr, cast(void*)m_chunkPtrs[i], curLength);
				btAlignedFree(cast(void*)m_chunkPtrs[i]);
				currentPtr += curLength;
				mysize += curLength;
			}
		}

		mTypes.clear();
		mStructs.clear();
		mTlens.clear();
		mStructReverse.clear();
		mTypeLookup.clear();
		m_skipPointers.clear();
		m_chunkP.clear();
		m_nameMap.clear();
		m_uniquePointers.clear();
		m_chunkPtrs.clear();
	}

	final override /*virtual*/ void* getUniquePointer(void* oldPtr)
	{
		btAssert(m_uniqueIdGenerator >= 0);
		if (!oldPtr)
			return null;

		btHashPtr key = btHashPtr(oldPtr);
		btPointerUid* uptr = cast(btPointerUid*)m_uniquePointers.find(key);
		if (uptr)
		{
			return uptr.m_ptr;
		}

		void** ptr2 = m_skipPointers[key];
		if (ptr2)
		{
			return null;
		}

		m_uniqueIdGenerator++;

		btPointerUid uid;
		uid.m_uniqueIds[0] = m_uniqueIdGenerator;
		uid.m_uniqueIds[1] = m_uniqueIdGenerator;
		m_uniquePointers.insert(key, uid);
		return uid.m_ptr;
	}

	final override /*virtual*/ const(char*) getBufferPointer() const
	{
		return m_buffer;
	}

	final override /*virtual*/ int getCurrentBufferSize() const
	{
		return m_currentSize;
	}

	final override /*virtual*/ void finalizeChunk(btChunk chunk, const(char*) structType, int chunkCode, void* oldPtr)
	{
		if (!(m_serializationFlags & btSerializationFlags.BT_SERIALIZE_NO_DUPLICATE_ASSERT))
		{
			btAssert(!findPointer(oldPtr));
		}

		chunk.m_dna_nr = getReverseType(structType);

		chunk.m_chunkCode = chunkCode;

		void* uniquePtr = getUniquePointer(oldPtr);

		m_chunkP.insert(btHashPtr(oldPtr), uniquePtr);  //chunk.m_oldPtr);
		chunk.m_oldPtr = uniquePtr;         //oldPtr;
	}

	/*virtual*/ char* internalAlloc(size_t size)
	{
		char* ptr = null;

		if (m_totalSize)
		{
			ptr = m_buffer + m_currentSize;
			m_currentSize += cast(int)size;
			btAssert(m_currentSize < m_totalSize);
		}
		else
		{
			ptr = cast(char*)btAlignedAlloc(size, 16);
			m_currentSize += cast(int)size;
		}
		return ptr;
	}

	final override /*virtual*/ btChunk allocate(size_t size, int numElements)
	{
		char* ptr = internalAlloc(cast(int)size * numElements + btChunk.sizeof);

		char* data = ptr + btChunk.sizeof;

		btChunk chunk = cast(btChunk)ptr;
		chunk.m_chunkCode = 0;
		chunk.m_oldPtr = data;
		chunk.m_length = cast(int)size * numElements;
		chunk.m_number = numElements;

		m_chunkPtrs.push_back(chunk);

		return chunk;
	}

	final override /*virtual*/ const(char*) findNameForPointer(const(void*) ptr) const
	{
		auto key = btHashPtr(ptr);
		/*const(const(char*)*)*/auto namePtr = m_nameMap.find(key);//const char* const* namePtr = m_nameMap.find(ptr);
		if (namePtr && *namePtr)
			return *namePtr;
		return null;
	}

	final override /*virtual*/ void registerNameForPointer(const(void*) ptr, const(char*) name)
	{
		m_nameMap.insert(btHashPtr(ptr), cast(char*)name);
	}

	final override /*virtual*/ void serializeName(const(char*) name)
	{
		if (name)
		{
			//don't serialize name twice
			if (findPointer(cast(void*)name))
				return;

			int len = btStrLen(name);
			if (len)
			{
				int newLen = len + 1;
				int padding = ((newLen + 3) & ~3) - newLen;
				newLen += padding;

				//serialize name string now
				btChunk chunk = allocate(char.sizeof, newLen);
				char* destinationName = cast(char*)chunk.m_oldPtr;
				for (int i = 0; i < len; i++)
				{
					destinationName[i] = name[i];
				}
				destinationName[len] = 0;
				finalizeChunk(chunk, "char", BT_ARRAY_CODE, cast(void*)name);
			}
		}
	}

	final override /*virtual*/ int getSerializationFlags() const
	{
		return m_serializationFlags;
	}

	final override /*virtual*/ void setSerializationFlags(int flags)
	{
		m_serializationFlags = flags;
	}
	final override int getNumChunks() const
	{
		return m_chunkPtrs.size();
	}

	final override const(btChunk) getChunk(int chunkIndex) const
	{
		return m_chunkPtrs[chunkIndex];
	}
};

///In general it is best to use btDefaultSerializer,
///in particular when writing the data to disk or sending it over the network.
///The btInMemorySerializer is experimental and only suitable in a few cases.
///The btInMemorySerializer takes a shortcut and can be useful to create a deep-copy
///of objects. There will be a demo on how to use the btInMemorySerializer.
/+version(ENABLE_INMEMORY_SERIALIZER)
{
	struct btInMemorySerializer : public btDefaultSerializer
	{
		btHashMap<btHashPtr, btChunk*> m_uid2ChunkPtr;
		btHashMap<btHashPtr, void*> m_orgPtr2UniqueDataPtr;
		btHashMap<btHashString, const(void*)> m_names2Ptr;

		btBulletSerializedArrays m_arrays;

		btInMemorySerializer(int totalSize = 0, unsigned char* buffer = 0)
			: btDefaultSerializer(totalSize, buffer)
		{
		}

		/*virtual*/ void startSerialization()
		{
			m_uid2ChunkPtr.clear();
			//todo: m_arrays.clear();
			btDefaultSerializer::startSerialization();
		}

		btChunk* findChunkFromUniquePointer(void* uniquePointer)
		{
			btChunk** chkPtr = m_uid2ChunkPtr[uniquePointer];
			if (chkPtr)
			{
				return *chkPtr;
			}
			return 0;
		}

		/*virtual*/ void registerNameForPointer(const(void*) ptr, const(char*) name)
		{
			btDefaultSerializer::registerNameForPointer(ptr, name);
			m_names2Ptr.insert(name, ptr);
		}

		/*virtual*/ void finishSerialization()
		{
		}

		/*virtual*/ void* getUniquePointer(void* oldPtr)
		{
			if (oldPtr == 0)
				return 0;

			// void* uniquePtr = getUniquePointer(oldPtr);
			btChunk* chunk = findChunkFromUniquePointer(oldPtr);
			if (chunk)
			{
				return chunk.m_oldPtr;
			}
			else
			{
				const(char*) n = (const(char*))oldPtr;
				const(void*)* ptr = m_names2Ptr[n];
				if (ptr)
				{
					return oldPtr;
				}
				else
				{
					void** ptr2 = m_skipPointers[oldPtr];
					if (ptr2)
					{
						return 0;
					}
					else
					{
						//If this assert hit, serialization happened in the wrong order
						// 'getUniquePointer'
						btAssert(0);
					}
				}
				return 0;
			}
			return oldPtr;
		}

		/*virtual*/ void finalizeChunk(btChunk* chunk, const(char*) structType, int chunkCode, void* oldPtr)
		{
			if (!(m_serializationFlags & BT_SERIALIZE_NO_DUPLICATE_ASSERT))
			{
				btAssert(!findPointer(oldPtr));
			}

			chunk.m_dna_nr = getReverseType(structType);
			chunk.m_chunkCode = chunkCode;
			//void* uniquePtr = getUniquePointer(oldPtr);
			m_chunkP.insert(oldPtr, oldPtr);  //chunk.m_oldPtr);
			// chunk.m_oldPtr = uniquePtr;//oldPtr;

			void* uid = findPointer(oldPtr);
			m_uid2ChunkPtr.insert(uid, chunk);

			switch (chunk.m_chunkCode)
			{
				case BT_SOFTBODY_CODE:
				{
	version (BT_USE_DOUBLE_PRECISION)
					m_arrays.m_softBodyDoubleData.push_back((btSoftBodyDoubleData*)chunk.m_oldPtr);
	else
					m_arrays.m_softBodyFloatData.push_back((btSoftBodyFloatData*)chunk.m_oldPtr);

					break;
				}
				case BT_COLLISIONOBJECT_CODE:
				{
	version (BT_USE_DOUBLE_PRECISION)
					m_arrays.m_collisionObjectDataDouble.push_back((btCollisionObjectDoubleData*)chunk.m_oldPtr);
	else
					m_arrays.m_collisionObjectDataFloat.push_back((btCollisionObjectFloatData*)chunk.m_oldPtr);

					break;
				}
				case BT_RIGIDBODY_CODE:
				{
	version (BT_USE_DOUBLE_PRECISION)
					m_arrays.m_rigidBodyDataDouble.push_back((btRigidBodyDoubleData*)chunk.m_oldPtr);
	else
					m_arrays.m_rigidBodyDataFloat.push_back((btRigidBodyFloatData*)chunk.m_oldPtr);

					break;
				};
				case BT_CONSTRAINT_CODE:
				{
	version (BT_USE_DOUBLE_PRECISION)
					m_arrays.m_constraintDataDouble.push_back((btTypedConstraintDoubleData*)chunk.m_oldPtr);
	else
					m_arrays.m_constraintDataFloat.push_back((btTypedConstraintFloatData*)chunk.m_oldPtr);

					break;
				}
				case BT_QUANTIZED_BVH_CODE:
				{
	version (BT_USE_DOUBLE_PRECISION)
					m_arrays.m_bvhsDouble.push_back((btQuantizedBvhDoubleData*)chunk.m_oldPtr);
	else
					m_arrays.m_bvhsFloat.push_back((btQuantizedBvhFloatData*)chunk.m_oldPtr);

					break;
				}

				case BT_SHAPE_CODE:
				{
					btCollisionShapeData* shapeData = (btCollisionShapeData*)chunk.m_oldPtr;
					m_arrays.m_colShapeData.push_back(shapeData);
					break;
				}
				case BT_TRIANLGE_INFO_MAP:
				case BT_ARRAY_CODE:
				case BT_SBMATERIAL_CODE:
				case BT_SBNODE_CODE:
				case BT_DYNAMICSWORLD_CODE:
				case BT_DNA_CODE:
				{
					break;
				}
				default:
				{
				}
			};
		}

		int getNumChunks() const
		{
			return m_uid2ChunkPtr.size();
		}

		const btChunk* getChunk(int chunkIndex) const
		{
			return *m_uid2ChunkPtr.getAtIndex(chunkIndex);
		}
	};
}
+/
