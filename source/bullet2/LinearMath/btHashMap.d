module bullet2.LinearMath.btHashMap;

extern (C++):

//#include <string>

import bullet2.LinearMath.btAlignedObjectArray;
import bullet2.LinearMath.btScalar;

///very basic hashable string implementation, compatible with btHashMap
struct btHashString
{
	string m_string1 = "";
	uint m_hash = 0;

	/*SIMD_FORCE_INLINE*/ uint getHash() const
	{
		return m_hash;
	}

	/*this()
	{
		m_string1 = "";
		m_hash = 0;
	}*/
	this(const(char*) name)
	{
		import std.conv;
        string m_string1 = to!string(name);
		/* magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/ */
		static const uint InitialFNV = 2166136261u;
		static const uint FNVMultiple = 16777619u;

		/* Fowler / Noll / Vo (FNV) Hash */
		uint hash = InitialFNV;

        import std.string;
        auto cstring = m_string1.toStringz();

		for (int i = 0; cstring[i]; i++)
		{
			hash = hash ^ (m_string1[i]); /* xor  the low 8 bits */
			hash = hash * FNVMultiple;            /* multiply by the magic number */
		}
		m_hash = hash;
	}

	bool equals(ref const(btHashString) other) const
	{
		return (m_string1 == other.m_string1);
	}
};

const __gshared int BT_HASH_NULL = 0xffffffff;

extern (C++, class)
struct btHashInt
{
	int m_uid;

public:
	this(int uid)
	{
        m_uid = uid;
	}

	int getUid1() const
	{
		return m_uid;
	}

	void setUid1(int uid)
	{
		m_uid = uid;
	}

	bool equals(ref const(btHashInt) other) const
	{
		return getUid1() == other.getUid1();
	}
	//to our success
	/*SIMD_FORCE_INLINE*/ uint getHash() const
	{
		uint key = m_uid;
		// Thomas Wang's hash
		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);

		return key;
	}
};

extern(C++, class)
struct btHashPtr
{
	union {
		const(void*) m_pointer;
		uint[2] m_hashValues;
	};

public:
	this(const(void*) ptr)
	{
        m_pointer = ptr;
	}

	const(void*) getPointer() const
	{
		return m_pointer;
	}

	bool equals(ref const(btHashPtr) other) const
	{
		return getPointer() == other.getPointer();
	}

	//to our success
	/*SIMD_FORCE_INLINE*/ uint getHash() const
	{
		const bool VOID_IS_8 = ((void*).sizeof == 8);

		uint key = VOID_IS_8 ? m_hashValues[0] + m_hashValues[1] : m_hashValues[0];
		// Thomas Wang's hash
		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);
		return key;
	}
};

//template <class Value>
class btHashKeyPtr(Value)
{
	int m_uid;

public:
	this(int uid)
	{
        m_uid = uid;
	}

	int getUid1() const
	{
		return m_uid;
	}

	bool equals(ref const(btHashKeyPtr!Value) other) const
	{
		return getUid1() == other.getUid1();
	}

	//to our success
	/*SIMD_FORCE_INLINE*/ uint getHash() const
	{
		uint key = m_uid;
		// Thomas Wang's hash
		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);
		return key;
	}
};

//template <class Value>
class btHashKey(Value)
{
	int m_uid;

public:
	this(int uid)
	{
        m_uid = uid;
	}

	int getUid1() const
	{
		return m_uid;
	}

	bool equals(ref const(btHashKey!Value) other) const
	{
		return getUid1() == other.getUid1();
	}
	//to our success
	/*SIMD_FORCE_INLINE*/ uint getHash() const
	{
		uint key = m_uid;
		// Thomas Wang's hash
		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);
		return key;
	}
};

///The btHashMap template class implements a generic and lightweight hashmap.
///A basic sample of how to use btHashMap is located in Demos\BasicDemo\main.cpp
//template <class Key, class Value>
class btHashMap(Key, Value)
{
protected:
	btAlignedObjectArray!int m_hashTable;
	btAlignedObjectArray!int m_next;

	btAlignedObjectArray!Value m_valueArray;
	btAlignedObjectArray!Key m_keyArray;

	void growTables(ref const(Key) key)
	{
		int newCapacity = m_valueArray.capacity();

		if (m_hashTable.size() < newCapacity)
		{
			//grow hashtable and next table
			int curHashtableSize = m_hashTable.size();

			m_hashTable.resize(newCapacity);
			m_next.resize(newCapacity);

			int i;

			for (i = 0; i < newCapacity; ++i)
			{
				m_hashTable[i] = BT_HASH_NULL;
			}
			for (i = 0; i < newCapacity; ++i)
			{
				m_next[i] = BT_HASH_NULL;
			}

			for (i = 0; i < curHashtableSize; i++)
			{
				//ref const(Value) value = m_valueArray[i];
				//ref const(Key) key = m_keyArray[i];

				int hashValue = m_keyArray[i].getHash() & (m_valueArray.capacity() - 1);  // New hash value with new mask
				m_next[i] = m_hashTable[hashValue];
				m_hashTable[hashValue] = i;
			}
		}
	}

public:
	void insert(Key key, Value value)
	{
		int hash = key.getHash() & (m_valueArray.capacity() - 1);

		//replace value if the key is already there
		int index = findIndex(key);
		if (index != BT_HASH_NULL)
		{
			m_valueArray[index] = value;
			return;
		}

		int count = m_valueArray.size();
		int oldCapacity = m_valueArray.capacity();
		m_valueArray.push_back(value);
		m_keyArray.push_back(key);

		int newCapacity = m_valueArray.capacity();
		if (oldCapacity < newCapacity)
		{
			growTables(key);
			//hash with new capacity
			hash = key.getHash() & (m_valueArray.capacity() - 1);
		}
		m_next[count] = m_hashTable[hash];
		m_hashTable[hash] = count;
	}

	void remove(ref const(Key) key)
	{
		int hash = key.getHash() & (m_valueArray.capacity() - 1);

		int pairIndex = findIndex(key);

		if (pairIndex == BT_HASH_NULL)
		{
			return;
		}

		// Remove the pair from the hash table.
		int index = m_hashTable[hash];
		btAssert(index != BT_HASH_NULL);

		int previous = BT_HASH_NULL;
		while (index != pairIndex)
		{
			previous = index;
			index = m_next[index];
		}

		if (previous != BT_HASH_NULL)
		{
			btAssert(m_next[previous] == pairIndex);
			m_next[previous] = m_next[pairIndex];
		}
		else
		{
			m_hashTable[hash] = m_next[pairIndex];
		}

		// We now move the last pair into spot of the
		// pair being removed. We need to fix the hash
		// table indices to support the move.

		int lastPairIndex = m_valueArray.size() - 1;

		// If the removed pair is the last pair, we are done.
		if (lastPairIndex == pairIndex)
		{
			m_valueArray.pop_back();
			m_keyArray.pop_back();
			return;
		}

		// Remove the last pair from the hash table.
		int lastHash = m_keyArray[lastPairIndex].getHash() & (m_valueArray.capacity() - 1);

		index = m_hashTable[lastHash];
		btAssert(index != BT_HASH_NULL);

		previous = BT_HASH_NULL;
		while (index != lastPairIndex)
		{
			previous = index;
			index = m_next[index];
		}

		if (previous != BT_HASH_NULL)
		{
			btAssert(m_next[previous] == lastPairIndex);
			m_next[previous] = m_next[lastPairIndex];
		}
		else
		{
			m_hashTable[lastHash] = m_next[lastPairIndex];
		}

		// Copy the last pair into the remove pair's spot.
		m_valueArray[pairIndex] = m_valueArray[lastPairIndex];
		m_keyArray[pairIndex] = m_keyArray[lastPairIndex];

		// Insert the last pair into the hash table
		m_next[pairIndex] = m_hashTable[lastHash];
		m_hashTable[lastHash] = pairIndex;

		m_valueArray.pop_back();
		m_keyArray.pop_back();
	}

	int size() const
	{
		return m_valueArray.size();
	}

	const(Value)* getAtIndex(int index) //const
	{
		btAssert(index < m_valueArray.size());
		btAssert(index >= 0);
		if (index >= 0 && index < m_valueArray.size())
		{
			return &m_valueArray[index];
		}
		return null;
	}

	Value* getAtIndex(int index)
	{
		btAssert(index < m_valueArray.size());
		btAssert(index >= 0);
		if (index >= 0 && index < m_valueArray.size())
		{
			return &m_valueArray[index];
		}
		return null;
	}

	Key getKeyAtIndex(int index)
	{
		btAssert(index < m_keyArray.size());
		btAssert(index >= 0);
		return m_keyArray[index];
	}

	const(Key) getKeyAtIndex(int index) const
	{
		btAssert(index < m_keyArray.size());
		btAssert(index >= 0);
		return m_keyArray[index];
	}



	//const(Value)* operator[](ref const(Key) key) const

	extern (D)
	const(Value)* opIndex(const(Key) key) const
	{
		return find(key);
	}

	extern (D)
	Value* opIndex(const(Key) key)
	//Value* operator[](ref const(Key) key)
	{
		return find(key);
	}

	/*const(Value)* find(const(Key) key) //const
	{
		int index = findIndex(key);
		if (index == BT_HASH_NULL)
		{
			return null;
		}
		return &m_valueArray[index];
	}*/

	Value* find(const(Key) key) //const
	{
		int index = findIndex(key);
		if (index == BT_HASH_NULL)
		{
			return null;
		}
		return &m_valueArray[index];
	}

	const(Value)* find(const(Key) key) const
	{
		int index = findIndex(key);
		if (index == BT_HASH_NULL)
		{
			return null;
		}
		return &m_valueArray[index];
	}

	/*Value* find(const(Key) key)
	{
		int index = findIndex(key);
		if (index == BT_HASH_NULL)
		{
			return null;
		}
		return &m_valueArray[index];
	}*/

	int findIndex(const(Key) key) const
	{
		uint hash = key.getHash() & (m_valueArray.capacity() - 1);

		if (hash >= cast(uint)m_hashTable.size())
		{
			return BT_HASH_NULL;
		}

		int index = m_hashTable[hash];
		while ((index != BT_HASH_NULL) && key.equals(m_keyArray[index]) == false)
		{
			index = m_next[index];
		}
		return index;
	}

	void clear()
	{
		m_hashTable.clear();
		m_next.clear();
		m_valueArray.clear();
		m_keyArray.clear();
	}
};
