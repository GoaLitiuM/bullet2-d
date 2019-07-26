module bullet2.LinearMath.btMinMax;

extern (C++):

import bullet2.LinearMath.btScalar;

/*SIMD_FORCE_INLINE*/ auto ref T btMin(T)(inout T a, inout T b)
{
	return a < b ? a : b;
}

/*SIMD_FORCE_INLINE*/ auto ref T btMax(T)(inout T a, inout T b)
{
	return a > b ? a : b;
}

/*SIMD_FORCE_INLINE*/ auto ref T btClamped(T)(inout T a, inout T lb, inout T ub)
{
	return a < lb ? lb : (ub < a ? ub : a);
}

/*SIMD_FORCE_INLINE*/ void btSetMin(T)(ref T a, inout T b)
{
	if (b < a)
	{
		a = b;
	}
}

/*SIMD_FORCE_INLINE*/ void btSetMax(T)(ref T a, inout T b)
{
	if (a < b)
	{
		a = b;
	}
}

/*SIMD_FORCE_INLINE*/ void btClamp(T)(ref T a, inout T lb, inout T ub)
{
	if (a < lb)
	{
		a = lb;
	}
	else if (ub < a)
	{
		a = ub;
	}
}
