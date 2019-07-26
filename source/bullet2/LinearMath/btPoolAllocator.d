module bullet2.LinearMath.btPoolAllocator;

extern (C++):

import bullet2.LinearMath.btThreads;
import bullet2.LinearMath.btAlignedAllocator;
import bullet2.LinearMath.btScalar;

///The btPoolAllocator class allows to efficiently allocate a large pool of objects, instead of dynamically allocating them separately.
class btPoolAllocator
{
	int m_elemSize;
	int m_maxElements;
	int m_freeCount;
	void* m_firstFree;
	ubyte* m_pool;
	btSpinMutex m_mutex;  // only used if BT_THREADSAFE

public:
	this(int elemSize, int maxElements)
	{
		m_elemSize = elemSize;
		m_maxElements = maxElements;
		m_pool = cast(ubyte*)btAlignedAlloc(cast(uint)(m_elemSize * m_maxElements), 16);

		ubyte* p = m_pool;
		m_firstFree = p;
		m_freeCount = m_maxElements;
		int count = m_maxElements;
		while (--count)
		{
			*cast(void**)p = (p + m_elemSize);
			p += m_elemSize;
		}
		*cast(void**)p = null;
	}

	~this()
	{
		btAlignedFree(m_pool);
	}

	int getFreeCount() const
	{
		return m_freeCount;
	}

	int getUsedCount() const
	{
		return m_maxElements - m_freeCount;
	}

	int getMaxCount() const
	{
		return m_maxElements;
	}

	void* allocate(int size)
	{
		// release mode fix
		//(void)size;
		btMutexLock(&m_mutex);
		btAssert(!size || size <= m_elemSize);
		//btAssert(m_freeCount>0);  // should return null if all full
		void* result = m_firstFree;
		if (null != m_firstFree)
		{
			m_firstFree = *cast(void**)m_firstFree;
			--m_freeCount;
		}
		btMutexUnlock(&m_mutex);
		return result;
	}

	bool validPtr(void* ptr)
	{
		if (ptr)
		{
			if ((cast(ubyte*)ptr >= m_pool && cast(ubyte*)ptr < m_pool + m_maxElements * m_elemSize))
			{
				return true;
			}
		}
		return false;
	}

	void freeMemory(void* ptr)
	{
		if (ptr)
		{
			btAssert(cast(ubyte*)ptr >= m_pool && cast(ubyte*)ptr < m_pool + m_maxElements * m_elemSize);

			btMutexLock(&m_mutex);
			*cast(void**)ptr = m_firstFree;
			m_firstFree = ptr;
			++m_freeCount;
			btMutexUnlock(&m_mutex);
		}
	}

	int getElementSize() const
	{
		return m_elemSize;
	}

	ubyte* getPoolAddress()
	{
		return m_pool;
	}

	const(ubyte*) getPoolAddress() const
	{
		return m_pool;
	}
};
