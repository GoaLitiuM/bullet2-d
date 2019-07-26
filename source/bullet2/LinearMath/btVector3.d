module bullet2.LinearMath.btVector3;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btAlignedAllocator;
import bullet2.LinearMath.btMinMax;

version (BT_USE_DOUBLE_PRECISION)
{
    alias btVector3Data = btVector3DoubleData;
    enum btVector3DataName = "btVector3DoubleData";
}
else
{
    alias btVector3Data = btVector3FloatData;
    enum btVector3DataName = "btVector3FloatData";
}

/+#if defined BT_USE_SSE

//typedef  uint32_t __m128i __attribute__ ((vector_size(16)));

#ifdef _MSC_VER
#pragma warning(disable : 4556)  // value of intrinsic immediate argument '4294967239' is out of range '0 - 255'
#endif

#define BT_SHUFFLE(x, y, z, w) (((w) << 6 | (z) << 4 | (y) << 2 | (x)) & 0xff)
//#define bt_pshufd_ps( _a, _mask ) (__m128) _mm_shuffle_epi32((__m128i)(_a), (_mask) )
#define bt_pshufd_ps(_a, _mask) _mm_shuffle_ps((_a), (_a), (_mask))
#define bt_splat3_ps(_a, _i) bt_pshufd_ps((_a), BT_SHUFFLE(_i, _i, _i, 3))
#define bt_splat_ps(_a, _i) bt_pshufd_ps((_a), BT_SHUFFLE(_i, _i, _i, _i))

#define btv3AbsiMask (_mm_set_epi32(0x00000000, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF))
#define btvAbsMask (_mm_set_epi32(0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF))
#define btvFFF0Mask (_mm_set_epi32(0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF))
#define btv3AbsfMask btCastiTo128f(btv3AbsiMask)
#define btvFFF0fMask btCastiTo128f(btvFFF0Mask)
#define btvxyzMaskf btvFFF0fMask
#define btvAbsfMask btCastiTo128f(btvAbsMask)

//there is an issue with XCode 3.2 (LCx errors)
#define btvMzeroMask (_mm_set_ps(-0.0f, -0.0f, -0.0f, -0.0f))
#define v1110 (_mm_set_ps(0.0f, 1.0f, 1.0f, 1.0f))
#define vHalf (_mm_set_ps(0.5f, 0.5f, 0.5f, 0.5f))
#define v1_5 (_mm_set_ps(1.5f, 1.5f, 1.5f, 1.5f))

//const __m128 ATTRIBUTE_ALIGNED16(btvMzeroMask) = {-0.0f, -0.0f, -0.0f, -0.0f};
//const __m128 ATTRIBUTE_ALIGNED16(v1110) = {1.0f, 1.0f, 1.0f, 0.0f};
//const __m128 ATTRIBUTE_ALIGNED16(vHalf) = {0.5f, 0.5f, 0.5f, 0.5f};
//const __m128 ATTRIBUTE_ALIGNED16(v1_5)  = {1.5f, 1.5f, 1.5f, 1.5f};

#endif

#ifdef BT_USE_NEON

const float32x4_t ATTRIBUTE_ALIGNED16(btvMzeroMask) = (float32x4_t){-0.0f, -0.0f, -0.0f, -0.0f};
const int32x4_t ATTRIBUTE_ALIGNED16(btvFFF0Mask) = (int32x4_t){static_cast<int32_t>(0xFFFFFFFF),
															   static_cast<int32_t>(0xFFFFFFFF), static_cast<int32_t>(0xFFFFFFFF), 0x0};
const int32x4_t ATTRIBUTE_ALIGNED16(btvAbsMask) = (int32x4_t){0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF};
const int32x4_t ATTRIBUTE_ALIGNED16(btv3AbsMask) = (int32x4_t){0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x0};

#endif+/

/**@brief btVector3 can be used to represent 3D points and vectors.
 * It has an un-used w component to suit 16-byte alignment when btVector3 is stored in containers. This extra component can be used by derived classes (Quaternion?) or by user
 * Ideally, this class should be replaced by a platform optimized SIMD version that keeps the data in registers
 */
//ATTRIBUTE_ALIGNED16(class)
extern (C++, class)
align(16) struct btVector3
{
public:
	btScalar[4] m_floats;

public:
	/**@brief No initialization constructor */
	/*this()
	{
	}*/

    this(inout btScalar _x, inout btScalar _y, inout btScalar _z)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = btScalar(0.0f);
	}

	this(inout btVector3 v)
	{
		m_floats = v.m_floats;
	}

	/*this(inout btVector3 v, inout btScalar w)
	{
		m_floats[0..2] = v.m_floats[0..2];
		m_floats[3] = w;
	}*/

	extern (D) inout btScalar x() { return m_floats[0]; };
    extern (D) inout btScalar y() { return m_floats[1]; };
    extern (D) inout btScalar z() { return m_floats[2]; };
	extern (D) inout btScalar w() { return m_floats[3]; };

	extern (D)
	auto ref btScalar opIndex(size_t i)
	{
		return m_floats[i];
	}

	extern (D)
	btScalar opIndex(size_t i) const
	{
		return m_floats[i];
	}

	extern (D)
	auto ref btVector3 opBinary(string op)(in btVector3 v2) const
	{
		static if (op == "+")
		{
			/*#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
				return btVector3(_mm_add_ps(v1.mVec128, v2.mVec128));
			#elif defined(BT_USE_NEON)
				return btVector3(vaddq_f32(v1.mVec128, v2.mVec128));
			#else*/
			return btVector3(
			this.m_floats[0] + v2.m_floats[0],
			this.m_floats[1] + v2.m_floats[1],
			this.m_floats[2] + v2.m_floats[2]);
			//#endif
		}
		else static if (op == "*")
		{
			/*#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
				return btVector3(_mm_mul_ps(v1.mVec128, v2.mVec128));
			#elif defined(BT_USE_NEON)
				return btVector3(vmulq_f32(v1.mVec128, v2.mVec128));
			#else*/
				return btVector3(
					this.m_floats[0] * v2.m_floats[0],
					this.m_floats[1] * v2.m_floats[1],
					this.m_floats[2] * v2.m_floats[2]);
			//#endif
		}
		else static if (op == "-")
		{
			/*#if defined BT_USE_SIMD_VECTOR3 && (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))

				//	without _mm_and_ps this code causes slowdown in Concave moving
				__m128 r = _mm_sub_ps(v1.mVec128, v2.mVec128);
				return btVector3(_mm_and_ps(r, btvFFF0fMask));
			#elif defined(BT_USE_NEON)
				float32x4_t r = vsubq_f32(v1.mVec128, v2.mVec128);
				return btVector3((float32x4_t)vandq_s32((int32x4_t)r, btvFFF0Mask));
			#else*/
				return btVector3(
					this.m_floats[0] - v2.m_floats[0],
					this.m_floats[1] - v2.m_floats[1],
					this.m_floats[2] - v2.m_floats[2]);
			//#endif
		}
		else static if (op == "/")
		{
			/*#if defined BT_USE_SIMD_VECTOR3 && (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))
				__m128 vec = _mm_div_ps(v1.mVec128, v2.mVec128);
				vec = _mm_and_ps(vec, btvFFF0fMask);
				return btVector3(vec);
			#elif defined(BT_USE_NEON)
				float32x4_t x, y, v, m;

				x = v1.mVec128;
				y = v2.mVec128;

				v = vrecpeq_f32(y);     // v ~ 1/y
				m = vrecpsq_f32(y, v);  // m = (2-v*y)
				v = vmulq_f32(v, m);    // vv = v*m ~~ 1/y
				m = vrecpsq_f32(y, v);  // mm = (2-vv*y)
				v = vmulq_f32(v, x);    // x*vv
				v = vmulq_f32(v, m);    // (x*vv)*(2-vv*y) = x*(vv(2-vv*y)) ~~~ x/y

				return btVector3(v);
			#else*/
				return btVector3(
					this.m_floats[0] / v2.m_floats[0],
					this.m_floats[1] / v2.m_floats[1],
					this.m_floats[2] / v2.m_floats[2]);
			//#endif
		}
		else
			static assert(0, "not implemented");
	}

	extern (D)
	auto ref btVector3 opBinary(string op)(in btScalar s) const
	{
		static if (op == "*")
		{
				/*#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
				__m128 vs = _mm_load_ss(&s);  //	(S 0 0 0)
				vs = bt_pshufd_ps(vs, 0x80);  //	(S S S 0.0)
				return btVector3(_mm_mul_ps(v.mVec128, vs));
			#elif defined(BT_USE_NEON)
				float32x4_t r = vmulq_n_f32(v.mVec128, s);
				return btVector3((float32x4_t)vandq_s32((int32x4_t)r, btvFFF0Mask));
			#else*/
				return btVector3(this.m_floats[0] * s, this.m_floats[1] * s, this.m_floats[2] * s);
			//#endif
		}
		else static if (op == "/")
		{
			btAssert(s != btScalar(0.0));
			/*#if 0  //defined(BT_USE_SSE_IN_API)
			// this code is not faster !
				__m128 vs = _mm_load_ss(&s);
				vs = _mm_div_ss(v1110, vs);
				vs = bt_pshufd_ps(vs, 0x00);	//	(S S S S)

				return btVector3(_mm_mul_ps(v.mVec128, vs));
			#else*/
				return this * (btScalar(1.0) / s);
			//#endif
		}
		else
			static assert("not implemented: " ~op);
	}

	extern (D)
	auto ref btVector3 opBinaryRight(string op)(in btScalar s) const
	{
		static if (op == "*")
		{
			return opBinary!op(s);
		}
		else
			static assert("not implemented");
	}

	extern (D)
	auto ref btVector3 opUnary(string op)() const
	{
		static if (op == "-")
		{
			/*#if defined BT_USE_SIMD_VECTOR3 && (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))
				__m128 r = _mm_xor_ps(v.mVec128, btvMzeroMask);
				return btVector3(_mm_and_ps(r, btvFFF0fMask));
			#elif defined(BT_USE_NEON)
				return btVector3((btSimdFloat4)veorq_s32((int32x4_t)v.mVec128, (int32x4_t)btvMzeroMask));
			#else*/
				return btVector3(-this.m_floats[0], -this.m_floats[1], -this.m_floats[2]);
			//#endif
		}
		else
			static assert(0, "not implemented");
	}

	extern (D)
	auto ref btVector3 opOpAssign(string op)(in btVector3 v)
	{
		static if (op == "+")
		{
			m_floats[0] += v.m_floats[0];
			m_floats[1] += v.m_floats[1];
			m_floats[2] += v.m_floats[2];
			return this;
		}
		else static if (op == "-")
		{
			m_floats[0] -= v.m_floats[0];
			m_floats[1] -= v.m_floats[1];
			m_floats[2] -= v.m_floats[2];
			return this;
		}
		else
			static assert(0, "not implemented");
	}

	extern (D)
	auto ref btVector3 opOpAssign(string op)(in btScalar s)
	{
		static if (op == "*")
		{
			m_floats[0] *= s;
			m_floats[1] *= s;
			m_floats[2] *= s;
			return this;
		}
		else static if (op == "/")
		{
			btFullAssert(s != btScalar(0.0));
			//return opAssign!"*"(btScalar(1.0) / s);//*this *= btScalar(1.0) / s;
			this *= btScalar(1.0) / s;
			return this;
		}
		else
			static assert(0, "not implemented");
	}

	/**@brief Return the dot product
   * @param v The other vector in the dot product */
   	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar dot(inout btVector3 v) const
	{
		//return m_floats[0] * v.m_floats[0] + m_floats[1] * v.m_floats[1] + m_floats[2] * v.m_floats[2];
		/+#if defined BT_USE_SIMD_VECTOR3 && defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
				__m128 vd = _mm_mul_ps(mVec128, v.mVec128);
				__m128 z = _mm_movehl_ps(vd, vd);
				__m128 y = _mm_shuffle_ps(vd, vd, 0x55);
				vd = _mm_add_ss(vd, y);
				vd = _mm_add_ss(vd, z);
				return _mm_cvtss_f32(vd);
		#elif defined(BT_USE_NEON)
				float32x4_t vd = vmulq_f32(mVec128, v.mVec128);
				float32x2_t x = vpadd_f32(vget_low_f32(vd), vget_low_f32(vd));
				x = vadd_f32(x, vget_high_f32(vd));
				return vget_lane_f32(x, 0);
		#else+/
				return m_floats[0] * v.m_floats[0] +
					m_floats[1] * v.m_floats[1] +
					m_floats[2] * v.m_floats[2];
		//#endif
	}

	/**@brief Return the length of the vector squared */
	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar length2() const
	{
		return dot(this);
	}

	/**@brief Return the length of the vector */
	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar length() const
	{
		return btSqrt(length2());
	}

	/**@brief Return the norm (length) of the vector */
	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar norm() const
	{
		return length();
	}

	/**@brief Return the norm (length) of the vector */
	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar safeNorm() const
	{
		btScalar d = length2();
		//workaround for some clang/gcc issue of sqrtf(tiny number) = -INF
		if (d > SIMD_EPSILON)
			return btSqrt(d);
		return btScalar(0);
	}

	/**@brief Return the distance squared between the ends of this and another vector
   	* This is symantically treating the vector like a point */
   	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar distance2(ref btVector3 v) //const
	{
		return (v - this).length2();
	}

	/**@brief Return the distance between the ends of this and another vector
   	* This is symantically treating the vector like a point */
	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar distance(ref btVector3 v) //const
	{
		return (v - this).length();
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ ref btVector3 safeNormalize() return
	{
		btVector3 absVec = this.absolute();
		int maxIndex = absVec.maxAxis();
		if (absVec[maxIndex] > 0)
		{
			this = this / absVec[maxIndex];
			this = this / length();
			return this;
			//opAssign!"/"(absVec[maxIndex]);//this /= absVec[maxIndex];
			//return opAssign!"/"(length());//(*this_) /= length();
		}
		setValue(1, 0, 0);
		return this;
	}

	/**@brief Normalize this vector
   * x^2 + y^2 + z^2 = 1 */
	extern (D)
	/*SIMD_FORCE_INLINE*/ ref btVector3 normalize() return
	{
		this = this / length();
		return this;
		//return opAssign!"/"(length());//this /= length();
	}

	/**@brief Return a normalized version of this vector */
	extern (D)
	/*SIMD_FORCE_INLINE*/ btVector3 normalized() //const
	{
		return this / length();
	}

	/**@brief Return a rotated version of this vector
   * @param wAxis The axis to rotate about
   * @param angle The angle to rotate by */
   	extern (D)
	/*SIMD_FORCE_INLINE*/ btVector3 rotate(ref btVector3 wAxis, const btScalar angle) //const
	{
		// wAxis must be a unit lenght vector

		btVector3 o = wAxis * wAxis.dot(this);
		btVector3 x = this - o;
		btVector3 y;

		y = wAxis.cross(this);

		return (o + x * btCos(angle) + y * btSin(angle));
	}

	/**@brief Return the angle between this and another vector
   * @param v The other vector */
   	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar angle(ref const(btVector3) v) const
	{
		btScalar s = btSqrt(length2() * v.length2());
		btAssert(s != btScalar(0.0));
		return btAcos(dot(v) / s);
	}
	/**@brief Return a vector will the absolute values of each element */
	extern (D)
	/*SIMD_FORCE_INLINE*/ btVector3 absolute() const
	{
		return btVector3(
			btFabs(m_floats[0]),
			btFabs(m_floats[1]),
			btFabs(m_floats[2]));
	}
	/**@brief Return the cross product between this and another vector
   * @param v The other vector */
   	extern (D)
	/*SIMD_FORCE_INLINE*/ btVector3 cross(inout btVector3 v) const
	{
		return btVector3(
			m_floats[1] * v.m_floats[2] - m_floats[2] * v.m_floats[1],
			m_floats[2] * v.m_floats[0] - m_floats[0] * v.m_floats[2],
			m_floats[0] * v.m_floats[1] - m_floats[1] * v.m_floats[0]);
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ btScalar triple(inout btVector3 v1, inout btVector3 v2) const
	{
		return m_floats[0] * (v1.m_floats[1] * v2.m_floats[2] - v1.m_floats[2] * v2.m_floats[1]) + m_floats[1] * (v1.m_floats[2] * v2.m_floats[0] - v1.m_floats[0] * v2.m_floats[2]) + m_floats[2] * (v1.m_floats[0] * v2.m_floats[1] - v1.m_floats[1] * v2.m_floats[0]);
	}

	/**@brief Return the axis with the smallest value
   * Note return values are 0,1,2 for x, y, or z */
	extern (D)
	/*SIMD_FORCE_INLINE*/ int minAxis() const
	{
		return m_floats[0] < m_floats[1] ? (m_floats[0] < m_floats[2] ? 0 : 2) : (m_floats[1] < m_floats[2] ? 1 : 2);
	}

	/**@brief Return the axis with the largest value
   * Note return values are 0,1,2 for x, y, or z */
	extern (D)
	/*SIMD_FORCE_INLINE*/ int maxAxis() const
	{
		return m_floats[0] < m_floats[1] ? (m_floats[1] < m_floats[2] ? 2 : 1) : (m_floats[0] < m_floats[2] ? 2 : 0);
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ int furthestAxis() const
	{
		return absolute().minAxis();
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ int closestAxis() const
	{
		return absolute().maxAxis();
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ void setInterpolate3(ref const(btVector3) v0, ref const(btVector3) v1, btScalar rt)
	{
		btScalar s = btScalar(1.0) - rt;
		m_floats[0] = s * v0.m_floats[0] + rt * v1.m_floats[0];
		m_floats[1] = s * v0.m_floats[1] + rt * v1.m_floats[1];
		m_floats[2] = s * v0.m_floats[2] + rt * v1.m_floats[2];
		//don't do the unused w component
		//		m_co[3] = s * v0[3] + rt * v1[3];
	}

	/**@brief Return the linear interpolation between this and another vector
   * @param v The other vector
   * @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
	extern (D)
	/*SIMD_FORCE_INLINE*/ btVector3 lerp(ref const(btVector3) v, const ref btScalar t) const
	{
		return btVector3(m_floats[0] + (v.m_floats[0] - m_floats[0]) * t,
						 m_floats[1] + (v.m_floats[1] - m_floats[1]) * t,
						 m_floats[2] + (v.m_floats[2] - m_floats[2]) * t);
	}
/+
	/**@brief Elementwise multiply this vector by the other
   * @param v The other vector */
	/*SIMD_FORCE_INLINE*/ ref btVector3 operator*=(ref const(btVector3) v)
	{
		m_floats[0] *= v.m_floats[0];
		m_floats[1] *= v.m_floats[1];
		m_floats[2] *= v.m_floats[2];
		return *this;
	}
+/
	/**@brief Return the x value */
	extern (D)/*SIMD_FORCE_INLINE*/ inout btScalar getX() { return m_floats[0]; }
	/**@brief Return the y value */
	extern (D)/*SIMD_FORCE_INLINE*/ inout btScalar getY() { return m_floats[1]; }
	/**@brief Return the z value */
	extern (D)/*SIMD_FORCE_INLINE*/ inout btScalar getZ() { return m_floats[2]; }
	/**@brief Set the x value */
	extern (D)/*SIMD_FORCE_INLINE*/ void setX(btScalar x) { m_floats[0] = x; };
	/**@brief Set the y value */
	extern (D)/*SIMD_FORCE_INLINE*/ void setY(btScalar y) { m_floats[1] = y; };
	/**@brief Set the z value */
	extern (D)/*SIMD_FORCE_INLINE*/ void setZ(btScalar z) { m_floats[2] = z; };
	/**@brief Set the w value */
	extern (D)/*SIMD_FORCE_INLINE*/ void setW(btScalar w) { m_floats[3] = w; };

	///*SIMD_FORCE_INLINE*/ ref btScalar       operator[](int i)       { return (&m_floats[0])[i];	}
	///*SIMD_FORCE_INLINE*/ const ref btScalar operator[](int i) const { return (&m_floats[0])[i]; }
	///operator btScalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
/+	/*SIMD_FORCE_INLINE*/ operator btScalar*() { return &m_floats[0]; }
	/*SIMD_FORCE_INLINE*/ operator const btScalar*() { return &m_floats[0]; }+/

	/*T opCast(T : bool)()
	{
		//return true;
		return this != null;
	}*/

	extern (D) bool opEquals(const btVector3 other) const
	{
		return ((m_floats[3] == other.m_floats[3]) && (m_floats[2] == other.m_floats[2]) && (m_floats[1] == other.m_floats[1]) && (m_floats[0] == other.m_floats[0]));
	}

	/**@brief Set each element to the max of the current values and the values of another btVector3
   * @param other The other btVector3 to compare with
   */
	extern (D) /*SIMD_FORCE_INLINE*/ void setMax(ref const(btVector3) other)
	{
		btSetMax(m_floats[0], other.m_floats[0]);
		btSetMax(m_floats[1], other.m_floats[1]);
		btSetMax(m_floats[2], other.m_floats[2]);
		btSetMax(m_floats[3], other.w());
	}
	/**@brief Set each element to the min of the current values and the values of another btVector3
   * @param other The other btVector3 to compare with
   */
	extern (D) /*SIMD_FORCE_INLINE*/ void setMin(ref const(btVector3) other)
	{
		btSetMin(m_floats[0], other.m_floats[0]);
		btSetMin(m_floats[1], other.m_floats[1]);
		btSetMin(m_floats[2], other.m_floats[2]);
		btSetMin(m_floats[3], other.w());
	}

	extern (D) /*SIMD_FORCE_INLINE*/ void setValue(inout btScalar _x, inout btScalar _y, inout btScalar _z)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = btScalar(0.0f);
	}

	extern (D) void getSkewSymmetricMatrix(btVector3 * v0, btVector3 * v1, btVector3 * v2) const
	{
		v0.setValue(0., -z(), y());
		v1.setValue(z(), 0., -x());
		v2.setValue(-y(), x(), 0.);
	}

	extern (D) void setZero()
	{
		setValue(btScalar(0.), btScalar(0.), btScalar(0.));
	}

	extern (D) /*SIMD_FORCE_INLINE*/ bool isZero() const
	{
		return m_floats[0] == btScalar(0) && m_floats[1] == btScalar(0) && m_floats[2] == btScalar(0);
	}

	extern (D) /*SIMD_FORCE_INLINE*/ bool fuzzyZero() const
	{
		return length2() < SIMD_EPSILON;
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ void serialize(ref btVector3Data dataOut) const
	{
		///could also do a memcpy, check if it is worth it
		for (int i = 0; i < 4; i++)
			dataOut.m_floats[i] = m_floats[i];
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ void deSerialize(const ref btVector3Data dataIn)
	{
		for (int i = 0; i < 4; i++)
			m_floats[i] = cast(btScalar)dataIn.m_floats[i];
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ void serializeFloat(ref btVector3FloatData dataOut) const
	{
		///could also do a memcpy, check if it is worth it
		for (int i = 0; i < 4; i++)
			dataOut.m_floats[i] = cast(float)(m_floats[i]);
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ void deSerializeFloat(const ref btVector3FloatData dataIn)
	{
		for (int i = 0; i < 4; i++)
			m_floats[i] = cast(btScalar)(dataIn.m_floats[i]);
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ void serializeDouble(ref btVector3DoubleData dataOut) const
	{
		///could also do a memcpy, check if it is worth it
		for (int i = 0; i < 4; i++)
			dataOut.m_floats[i] = cast(double)(m_floats[i]);
	}

	extern (D)
	/*SIMD_FORCE_INLINE*/ void deSerializeDouble(const ref btVector3DoubleData dataIn)
	{
		for (int i = 0; i < 4; i++)
			m_floats[i] = cast(btScalar)(dataIn.m_floats[i]);
	}

	/* create a vector as  btVector3( this->dot( btVector3 v0 ), this->dot( btVector3 v1), this->dot( btVector3 v2 ))  */
	extern (D) /*SIMD_FORCE_INLINE*/ btVector3 dot3(const(btVector3) v0, const(btVector3) v1, const(btVector3) v2) const
	{
/+#if defined BT_USE_SIMD_VECTOR3 && defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)

		__m128 a0 = _mm_mul_ps(v0.mVec128, this->mVec128);
		__m128 a1 = _mm_mul_ps(v1.mVec128, this->mVec128);
		__m128 a2 = _mm_mul_ps(v2.mVec128, this->mVec128);
		__m128 b0 = _mm_unpacklo_ps(a0, a1);
		__m128 b1 = _mm_unpackhi_ps(a0, a1);
		__m128 b2 = _mm_unpacklo_ps(a2, _mm_setzero_ps());
		__m128 r = _mm_movelh_ps(b0, b2);
		r = _mm_add_ps(r, _mm_movehl_ps(b2, b0));
		a2 = _mm_and_ps(a2, btvxyzMaskf);
		r = _mm_add_ps(r, btCastdTo128f(_mm_move_sd(btCastfTo128d(a2), btCastfTo128d(b1))));
		return btVector3(r);

#elif defined(BT_USE_NEON)
		static const uint32x4_t xyzMask = (const uint32x4_t){static_cast<uint32_t>(-1), static_cast<uint32_t>(-1), static_cast<uint32_t>(-1), 0};
		float32x4_t a0 = vmulq_f32(v0.mVec128, this->mVec128);
		float32x4_t a1 = vmulq_f32(v1.mVec128, this->mVec128);
		float32x4_t a2 = vmulq_f32(v2.mVec128, this->mVec128);
		float32x2x2_t zLo = vtrn_f32(vget_high_f32(a0), vget_high_f32(a1));
		a2 = (float32x4_t)vandq_u32((uint32x4_t)a2, xyzMask);
		float32x2_t b0 = vadd_f32(vpadd_f32(vget_low_f32(a0), vget_low_f32(a1)), zLo.val[0]);
		float32x2_t b1 = vpadd_f32(vpadd_f32(vget_low_f32(a2), vget_high_f32(a2)), vdup_n_f32(0.0f));
		return btVector3(vcombine_f32(b0, b1));
#else+/
		return btVector3(dot(v0), dot(v1), dot(v2));
//#endif
	}
};

extern (C++, class)
align(16) struct btVector4
{
	btVector3 vec3;
	alias vec3 this;

	this(inout btScalar _x, inout btScalar _y, inout btScalar _z, inout btScalar _w)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = _w;
	}

	/*SIMD_FORCE_INLINE*/ void setValue(inout btScalar _x, inout btScalar _y, inout btScalar _z, inout btScalar _w)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = _w;
	}
}

/+
///btSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
/*SIMD_FORCE_INLINE*/ void btSwapScalarEndian(const ref btScalar sourceVal, ref btScalar destVal)
{
#ifdef BT_USE_DOUBLE_PRECISION
	unsigned char* dest = (unsigned char*)&destVal;
	const unsigned char* src = (const unsigned char*)&sourceVal;
	dest[0] = src[7];
	dest[1] = src[6];
	dest[2] = src[5];
	dest[3] = src[4];
	dest[4] = src[3];
	dest[5] = src[2];
	dest[6] = src[1];
	dest[7] = src[0];
#else
	unsigned char* dest = (unsigned char*)&destVal;
	const unsigned char* src = (const unsigned char*)&sourceVal;
	dest[0] = src[3];
	dest[1] = src[2];
	dest[2] = src[1];
	dest[3] = src[0];
#endif  //BT_USE_DOUBLE_PRECISION
}
///btSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
/*SIMD_FORCE_INLINE*/ void btSwapVector3Endian(ref const(btVector3) sourceVec, ref btVector3 destVec)
{
	for (int i = 0; i < 4; i++)
	{
		btSwapScalarEndian(sourceVec[i], destVec[i]);
	}
}

///btUnSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
/*SIMD_FORCE_INLINE*/ void btUnSwapVector3Endian(ref btVector3 vector)
{
	btVector3 swappedVec;
	for (int i = 0; i < 4; i++)
	{
		btSwapScalarEndian(vector[i], swappedVec[i]);
	}
	vector = swappedVec;
}
+/
/*SIMD_FORCE_INLINE*/ void btPlaneSpace1(T)(ref T n, ref T p, ref T q)
{
	if (btFabs(n[2]) > SIMDSQRT12)
	{
		// choose p in y-z plane
		btScalar a = n[1] * n[1] + n[2] * n[2];
		btScalar k = btRecipSqrt(a);
		p[0] = 0;
		p[1] = -n[2] * k;
		p[2] = n[1] * k;
		// set q = n x p
		q[0] = a * k;
		q[1] = -n[0] * p[2];
		q[2] = n[0] * p[1];
	}
	else
	{
		// choose p in x-y plane
		btScalar a = n[0] * n[0] + n[1] * n[1];
		btScalar k = btRecipSqrt(a);
		p[0] = -n[1] * k;
		p[1] = n[0] * k;
		p[2] = 0;
		// set q = n x p
		q[0] = -n[2] * p[1];
		q[1] = n[2] * p[0];
		q[2] = a * k;
	}
}

struct btVector3FloatData
{
	float[4] m_floats;
};

struct btVector3DoubleData
{
	double[4] m_floats;
};

/**@brief Return the dot product between two vectors */
/*SIMD_FORCE_INLINE*/ btScalar
btDot(const inout btVector3 v1, const inout btVector3 v2)
{
	return v1.dot(v2);
}

/**@brief Return the distance squared between two vectors */
/*SIMD_FORCE_INLINE*/ btScalar
btDistance2(ref btVector3 v1, ref btVector3 v2)
{
	return v1.distance2(v2);
}

/**@brief Return the distance between two vectors */
/*SIMD_FORCE_INLINE*/ btScalar
btDistance(ref btVector3 v1, ref btVector3 v2)
{
	return v1.distance(v2);
}

/**@brief Return the angle between two vectors */
/*SIMD_FORCE_INLINE*/ btScalar
btAngle(const inout btVector3 v1, const inout btVector3 v2)
{
	return v1.angle(v2);
}

/**@brief Return the cross product of two vectors */
/*SIMD_FORCE_INLINE*/ btVector3
btCross(const btVector3 v1, const btVector3 v2)
{
	return v1.cross(v2);
}

/*SIMD_FORCE_INLINE*/ btScalar
btTriple(const inout btVector3 v1, const inout btVector3 v2, const inout btVector3 v3)
{
	return v1.triple(v2, v3);
}
