module bullet2.LinearMath.btAlignedAllocator;

extern (C++):
///we probably replace this with our own aligned memory allocator
///so we replace _aligned_malloc and _aligned_free with our own
///that is better portable and more predictable

import bullet2.LinearMath.btScalar;

///BT_DEBUG_MEMORY_ALLOCATIONS preprocessor can be set in build system
///for regression tests to detect memory leaks
///#define BT_DEBUG_MEMORY_ALLOCATIONS 1
/+#ifdef BT_DEBUG_MEMORY_ALLOCATIONS

int btDumpMemoryLeaks();

#define btAlignedAlloc(a, b) \
	btAlignedAllocInternal(a, b, __LINE__, __FILE__)

#define btAlignedFree(ptr) \
	btAlignedFreeInternal(ptr, __LINE__, __FILE__)

void* btAlignedAllocInternal(size_t size, int alignment, int line, char* filename);

void btAlignedFreeInternal(void* ptr, int line, char* filename);

#else+/
void* btAlignedAllocInternal(size_t size, int alignment);
void btAlignedFreeInternal(void* ptr);

//#define btAlignedAlloc(size, alignment) btAlignedAllocInternal(size, alignment)
//#define btAlignedFree(ptr) btAlignedFreeInternal(ptr)
alias btAlignedAlloc = btAlignedAllocInternal;
alias btAlignedFree = btAlignedFreeInternal;

//#endif
alias size_type = int;


alias btAlignedAllocFunc = void* function(size_t size, int alignment);
alias btAlignedFreeFunc = void function(void* memblock);
alias btAllocFunc = void* function(size_t size);
alias btFreeFunc = void function(void* memblock);

///The developer can let all Bullet memory allocations go through a custom memory allocator, using btAlignedAllocSetCustom
void btAlignedAllocSetCustom(btAllocFunc* allocFunc, btFreeFunc* freeFunc);
///If the developer has already an custom aligned allocator, then btAlignedAllocSetCustomAligned can be used. The default aligned allocator pre-allocates extra memory using the non-aligned allocator, and instruments it.
void btAlignedAllocSetCustomAligned(btAlignedAllocFunc* allocFunc, btAlignedFreeFunc* freeFunc);

///The btAlignedAllocator is a portable class for aligned memory allocations.
///Default implementations for unaligned and aligned allocations can be overridden by a custom allocator using btAlignedAllocSetCustom and btAlignedAllocSetCustomAligned.
//template <typename T, unsigned Alignment>
struct btAlignedAllocator(T, int Alignment)
{
	alias self_type = btAlignedAllocator!(T, Alignment);

public:
	//just going down a list:
	//this() {}
	/*
	btAlignedAllocator( const self_type & ) {}
	*/

	//template <typename Other>
	this(Other)(const ref btAlignedAllocator!(Other, Alignment) unused_)
	{
	}

	alias const_pointer = const T*;
	alias const_reference = const ref T;
	alias pointer = T*;
	alias reference = ref T;
	alias value_type = T;

	//pointer address(reference ref_) const { return &ref_; }
	//const_pointer address(const_reference ref_) const { return &ref_; }
	pointer allocate(size_type n, const_pointer* hint = null)
	{
		//(void)hint;
		return cast(pointer)(btAlignedAlloc(value_type.sizeof * n, Alignment));
	}
	//void construct(pointer ptr, const ref value_type value) { new (ptr) value_type(value); }
	void deallocate(pointer ptr)
	{
		btAlignedFree(cast(void*)(ptr));
	}
	void destroy(pointer ptr) { object.destroy(ptr); }//ptr.destroy();/*ptr.__dtor();*/ }

	/*struct rebind(O)
	{
		typedef btAlignedAllocator<O, Alignment> other;
	};*/

	/*template <typename O>
	self_type& operator=(const btAlignedAllocator<O, Alignment>&)
	{
		return *this;
	}

	friend bool operator==(const self_type&, const self_type&) { return true; }*/
};
