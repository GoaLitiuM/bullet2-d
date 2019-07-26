module bullet2.BulletCollision.CollisionDispatch.btUnionFind;

extern (C++):

import bullet2.LinearMath.btAlignedObjectArray;

version = USE_PATH_COMPRESSION;

///see for discussion of static island optimizations by Vroonsh here: http://code.google.com/p/bullet/issues/detail?id=406
enum STATIC_SIMULATION_ISLAND_OPTIMIZATION = 1;

struct btElement
{
	int m_id;
	int m_sz;
};

///UnionFind calculates connected subsets
// Implements weighted Quick Union with path compression
// optimization: could use short ints instead of ints (halving memory, would limit the number of rigid bodies to 64k, sounds reasonable)
class btUnionFind
{
private:
	btAlignedObjectArray!btElement m_elements;

public:
	this();
	//~this();
    final void __dtor();

	//this is a special operation, destroying the content of btUnionFind.
	//it sorts the elements, based on island id, in order to make it easy to iterate over islands
	final void sortIslands();

	final void reset(int N);

	/*SIMD_FORCE_INLINE*/ int getNumElements() const
	{
		return int(m_elements.size());
	}
	/*SIMD_FORCE_INLINE*/ bool isRoot(int x) //const
	{
		return (x == m_elements[x].m_id);
	}

	ref btElement getElement(int index)
	{
		return m_elements[index];
	}
	ref const(btElement) getElement(int index) //const
	{
		return m_elements[index];
	}

	final void allocate(int N);
	final void Free();

	int find(int p, int q)
	{
		return (find(p) == find(q));
	}

	void unite(int p, int q)
	{
		int i = find(p), j = find(q);
		if (i == j)
			return;

        version (USE_PATH_COMPRESSION)
        {
            //weighted quick union, this keeps the 'trees' balanced, and keeps performance of unite O( log(n) )
            if (m_elements[i].m_sz < m_elements[j].m_sz)
            {
                m_elements[i].m_id = j;
                m_elements[j].m_sz += m_elements[i].m_sz;
            }
            else
            {
                m_elements[j].m_id = i;
                m_elements[i].m_sz += m_elements[j].m_sz;
            }
        }
        else
        {
            m_elements[i].m_id = j;
            m_elements[j].m_sz += m_elements[i].m_sz;
        }
	}

	int find(int x)
	{
		//btAssert(x < m_N);
		//btAssert(x >= 0);

		while (x != m_elements[x].m_id)
		{
			//not really a reason not to use path compression, and it flattens the trees/improves find performance dramatically

            version (USE_PATH_COMPRESSION)
            {
                const btElement* elementPtr = &m_elements[m_elements[x].m_id];
                m_elements[x].m_id = elementPtr.m_id;
                x = elementPtr.m_id;
            }
            else
            {
                x = m_elements[x].m_id;
            }
			//btAssert(x < m_N);
			//btAssert(x >= 0);
		}
		return x;
	}
};
