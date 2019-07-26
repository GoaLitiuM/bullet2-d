module bullet2.LinearMath.btAlignedObjectArray;

extern (C++):

import bullet2.LinearMath.btScalar;  // has definitions like /*SIMD_FORCE_INLINE*/
import bullet2.LinearMath.btAlignedAllocator;

///If the platform doesn't support placement new, you can disable BT_USE_PLACEMENT_NEW
///then the btAlignedObjectArray doesn't support objects with virtual methods, and non-trivial constructors/destructors
///You can enable BT_USE_MEMCPY, then swapping elements in the array will use memcpy instead of operator=
///see discussion here: http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1231 and
///http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1240
/+
#define BT_USE_PLACEMENT_NEW 1
//#define BT_USE_MEMCPY 1 //disable, because it is cumbersome to find out for each platform where memcpy is defined. It can be in <memory.h> or <string.h> or otherwise...
#define BT_ALLOW_ARRAY_COPY_OPERATOR  // enabling this can accidently perform deep copies of data if you are not careful

#ifdef BT_USE_MEMCPY
#include <memory.h>
#include <string.h>
#endif  //BT_USE_MEMCPY

#ifdef BT_USE_PLACEMENT_NEW
#include <new>  //for placement new
#endif          //BT_USE_PLACEMENT_NEW
+/

///The btAlignedObjectArray template class uses a subset of the stl::vector interface for its methods
///It is developed to replace stl::vector to avoid portability issues, including STL alignment issues to add SIMD/SSE data
extern (C++, class)
struct btAlignedObjectArray(T)
{
	btAlignedAllocator!(T, 16) m_allocator;

	int m_size;
	int m_capacity;
	T* m_data;
	//PCK: added this line
	bool m_ownsMemory = true;

/+version (BT_ALLOW_ARRAY_COPY_OPERATOR)
{
    public:
        /*SIMD_FORCE_INLINE*/ btAlignedObjectArray(T)& operator=(ref const(btAlignedObjectArray)!(T) other)
        {
            copyFromArray(other);
            return *this;
        }
}
else
{
    private:
        /*SIMD_FORCE_INLINE*/ btAlignedObjectArray(T)& operator=(ref const(btAlignedObjectArray)!(T) other);
}+/

protected:
	/*SIMD_FORCE_INLINE*/ int allocSize(int size)
	{
		return (size ? size * 2 : 1);
	}
	/*SIMD_FORCE_INLINE*/ void copy(int start, int end, T* dest) //const
	{
		int i;
		for (i = start; i < end; ++i)
/*#ifdef BT_USE_PLACEMENT_NEW
			new (&dest[i]) T(m_data[i]);
#else*/
			dest[i] = cast(T)(m_data[i]);
//#endif  //BT_USE_PLACEMENT_NEW
	}

	/*SIMD_FORCE_INLINE*/ void init()
	{
		//PCK: added this line
		m_ownsMemory = true;
		m_data = null;
		m_size = 0;
		m_capacity = 0;
	}
	/*SIMD_FORCE_INLINE*/ void destroy(int first, int last)
	{
		/+static if (is(typeof(m_data[0].__xdtor())))
		{
			for (int i = first; i < last; i++)
			{
				static if (__traits(compiles, m_data[i] is null))
				{
					if (m_data[i] is null)
						continue;
				}

				object.destroy(m_data[i]);

				//m_data[i].~T();
			}
		}+/
	}

	/*SIMD_FORCE_INLINE*/ void* allocate(int size)
	{
		if (size)
		{
			import core.memory;
			void* ptr = cast(void*)m_allocator.allocate(size);
			GC.addRange(ptr, T.sizeof * size, null);
			return ptr;
		}
		return null;
	}

	/*SIMD_FORCE_INLINE*/ void deallocate()
	{
		if (m_data)
		{
			import core.memory;
			GC.removeRange(m_data);
			//PCK: enclosed the deallocation in this block
			if (m_ownsMemory)
			{
				m_allocator.deallocate(m_data);
			}
			m_data = null;
		}
	}

public:
	/*this()
	{
		init();
	}*/

	~this()
	{
		clear();
	}

	///Generally it is best to avoid using the copy constructor of an btAlignedObjectArray, and use a (const) reference to the array instead.
	/*this(btAlignedObjectArray otherArray)
	{
		this(otherArray);
	}*/

	this(btAlignedObjectArray otherArray)
	{
		init();

		int otherSize = otherArray.size();
		resize(otherSize);
		otherArray.copy(0, otherSize, m_data);
	}

	/// return the number of elements in the array
	/*SIMD_FORCE_INLINE*/ int size() const
	{
		return m_size;
	}


	/*SIMD_FORCE_INLINE*/ /*const ref T at(int n)
	{
		btAssert(n >= 0);
		btAssert(n < size());
		return m_data[n];
	}*/

	/*SIMD_FORCE_INLINE*/ ref T at(int n)
	{
		btAssert(n >= 0);
		btAssert(n < size());
		return m_data[n];
	}

	/*auto ref T opIndex(int n) const
	{
		btAssert(n >= 0);
		btAssert(n < size());
		return m_data[n];
	}*/

	/*ref T opIndex(int n) const
	{
		btAssert(n >= 0);
		btAssert(n < size());
		return m_data[n];
	}*/

	/*T* opIndex(int n) //const
	{
		btAssert(n >= 0);
		btAssert(n < size());
		return &m_data[n];
	}

	T opIndex(int n) const
	{
		btAssert(n >= 0);
		btAssert(n < size());
		return m_data[n];
	}*/

	ref const(T) opIndex(int n) const
	{
		btAssert(n >= 0);
		btAssert(n < size());

		const(T)[] d = m_data[0..m_size];
		return d[n];
	}

	ref T opIndex(int n)
	{
		btAssert(n >= 0);
		btAssert(n < size());

		T[] d = m_data[0..m_size];
		return d[n];
	}

	/*auto ref T opIndex(int n) const
	{
		btAssert(n >= 0);
		btAssert(n < size());
		return m_data[n];
	}*/

	///clear the array, deallocated memory. Generally it is better to use array.resize(0), to reduce performance overhead of run-time memory (de)allocations.
	/*SIMD_FORCE_INLINE*/ void clear()
	{
		destroy(0, size());

		deallocate();

		init();
	}

	/*SIMD_FORCE_INLINE*/ void pop_back()
	{
		btAssert(m_size > 0);
		m_size--;
		static if (is(typeof(m_data[m_size].__xdtor())))
			object.destroy(m_data[m_size]);
		//object.destroy(m_data[m_size]);//m_data[m_size].~T();
	}

	///resize changes the number of elements in the array. If the new size is larger, the new elements will be constructed using the optional second argument.
	///when the new number of elements is smaller, the destructor will be called, but memory will not be freed, to reduce performance overhead of run-time memory (de)allocations.
	/*SIMD_FORCE_INLINE*/ void resizeNoInitialize(int newsize)
	{
		if (newsize > size())
		{
			reserve(newsize);
		}
		m_size = newsize;
	}

	final void resize(int newsize, T fillData)
	{
		resize(newsize);
	}
	/*SIMD_FORCE_INLINE*/ void resize(int newsize/*, T fillData = T()*/)
	{
		const int curSize = size();

		if (newsize < curSize)
		{
			for (int i = newsize; i < curSize; i++)
			{
				static if (is(typeof(m_data[i].__xdtor())))
					object.destroy(m_data[i]);
				//object.destroy(m_data[i]);//m_data[i].~T();
			}
		}
		else
		{
			if (newsize > curSize)
			{
				reserve(newsize);
			}
/*#ifdef BT_USE_PLACEMENT_NEW
			for (int i = curSize; i < newsize; i++)
			{
				new (&m_data[i]) T(fillData);
			}
#endif  //BT_USE_PLACEMENT_NEW*/
		}

		m_size = newsize;
	}
	/*SIMD_FORCE_INLINE*/ ref T expandNonInitializing()
	{
		const int sz = size();
		if (sz == capacity())
		{
			reserve(allocSize(size()));
		}
		m_size++;

		return m_data[sz];
	}

	/*SIMD_FORCE_INLINE*/ ref T expand(const ref T fillValue)
	{
		const int sz = size();
		if (sz == capacity())
		{
			reserve(allocSize(size()));
		}
		m_size++;
/*#ifdef BT_USE_PLACEMENT_NEW
		new (&m_data[sz]) T(fillValue);  //use the in-place new (not really allocating heap memory)
#endif*/

		return m_data[sz];
	}

	/*SIMD_FORCE_INLINE*/ void push_back()(auto ref T _Val)
	{
		const int sz = size();
		if (sz == capacity())
		{
			reserve(allocSize(size()));
		}

/*#ifdef BT_USE_PLACEMENT_NEW
		new (&m_data[m_size]) T(_Val);
#else*/
		m_data[size()] = _Val;
//#endif  //BT_USE_PLACEMENT_NEW

		m_size++;
	}

	/// return the pre-allocated (reserved) elements, this is at least as large as the total number of elements,see size() and reserve()
	/*SIMD_FORCE_INLINE*/ int capacity() const
	{
		return m_capacity;
	}

	/*SIMD_FORCE_INLINE*/ void reserve(int _Count)
	{  // determine new minimum length of allocated storage
		if (capacity() < _Count)
		{  // not enough room, reallocate
			T* s = cast(T*)allocate(_Count);

			copy(0, size(), s);

			destroy(0, size());

			deallocate();

			//PCK: added this line
			m_ownsMemory = true;

			m_data = s;

			m_capacity = _Count;
		}
	}

	/*class less
	{
	public:
		bool operator()(const ref T a, const ref T b) const
		{
			return (a < b);
		}
	};*/

	//template <typename L>
	void quickSortInternal(L)(const ref L CompareFunc, int lo, int hi)
	{
		//  lo is the lower index, hi is the upper index
		//  of the region of array a that is to be sorted
		int i = lo, j = hi;
		T x = m_data[(lo + hi) / 2];

		//  partition
		do
		{
			while (CompareFunc(m_data[i], x))
				i++;
			while (CompareFunc(x, m_data[j]))
				j--;
			if (i <= j)
			{
				swap(i, j);
				i++;
				j--;
			}
		} while (i <= j);

		//  recursion
		if (lo < j)
			quickSortInternal(CompareFunc, lo, j);
		if (i < hi)
			quickSortInternal(CompareFunc, i, hi);
	}

	//template <typename L>
	void quickSort(L)(const ref L CompareFunc)
	{
		//don't sort 0 or 1 elements
		if (size() > 1)
		{
			quickSortInternal(CompareFunc, 0, size() - 1);
		}
	}

	///heap sort from http://www.csse.monash.edu.au/~lloyd/tildeAlgDS/Sort/Heap/
	//template <typename L>
	void downHeap(L)(T* pArr, int k, int n, const ref L CompareFunc)
	{
		/*  PRE: a[k+1..N] is a heap */
		/* POST:  a[k..N]  is a heap */

		T temp = pArr[k - 1];
		/* k has child(s) */
		while (k <= n / 2)
		{
			int child = 2 * k;

			if ((child < n) && CompareFunc(pArr[child - 1], pArr[child]))
			{
				child++;
			}
			/* pick larger child */
			if (CompareFunc(temp, pArr[child - 1]))
			{
				/* move child up */
				pArr[k - 1] = pArr[child - 1];
				k = child;
			}
			else
			{
				break;
			}
		}
		pArr[k - 1] = temp;
	} /*downHeap*/

	void swap(int index0, int index1)
	{
/*#ifdef BT_USE_MEMCPY
		char temp[sizeof(T)];
		memcpy(temp, &m_data[index0], sizeof(T));
		memcpy(&m_data[index0], &m_data[index1], sizeof(T));
		memcpy(&m_data[index1], temp, sizeof(T));
#else*/
		T temp = m_data[index0];
		m_data[index0] = m_data[index1];
		m_data[index1] = temp;
//#endif  //BT_USE_PLACEMENT_NEW
	}

	//template <typename L>
	void heapSort(L)(const ref L CompareFunc)
	{
		/* sort a[0..N-1],  N.B. 0 to N-1 */
		int k;
		int n = m_size;
		for (k = n / 2; k > 0; k--)
		{
			downHeap(m_data, k, n, CompareFunc);
		}

		/* a[1..N] is now a heap */
		while (n >= 1)
		{
			swap(0, n - 1); /* largest of a[0..n-1] */

			n = n - 1;
			/* restore a[1..i-1] heap */
			downHeap(m_data, 1, n, CompareFunc);
		}
	}

	///non-recursive binary search, assumes sorted array
	/+int findBinarySearch(const ref T key) const
	{
		int first = 0;
		int last = size() - 1;

		//assume sorted array
		while (first <= last)
		{
			int mid = (first + last) / 2;  // compute mid point.
			if (key > m_data[mid])
				first = mid + 1;  // repeat search in top half.
			else if (key < m_data[mid])
				last = mid - 1;  // repeat search in bottom half.
			else
				return mid;  // found it. return position /////
		}
		return size();  // failed to find key
	}+/

	int findLinearSearch(const ref T key) const
	{
		int index = size();
		int i;

		for (i = 0; i < size(); i++)
		{
			if (m_data[i] == key)
			{
				index = i;
				break;
			}
		}
		return index;
	}

	// If the key is not in the array, return -1 instead of 0,
	// since 0 also means the first element in the array.
	int findLinearSearch2(const ref T key) const
	{
		int index = -1;
		int i;

		for (i = 0; i < size(); i++)
		{
			if (m_data[i] == key)
			{
				index = i;
				break;
			}
		}
		return index;
	}

	void removeAtIndex(int index)
	{
		if (index < size())
		{
			swap(index, size() - 1);
			pop_back();
		}
	}
	void remove(const ref T key)
	{
		int findIndex = findLinearSearch(key);
		removeAtIndex(findIndex);
	}

	//PCK: whole function
	void initializeFromBuffer(void* buffer, int size, int capacity)
	{
		clear();
		m_ownsMemory = false;
		m_data = cast(T*)buffer;
		m_size = size;
		m_capacity = capacity;
	}

	void copyFromArray(ref btAlignedObjectArray otherArray)
	{
		int otherSize = otherArray.size();
		resize(otherSize);
		otherArray.copy(0, otherSize, m_data);
	}
};

