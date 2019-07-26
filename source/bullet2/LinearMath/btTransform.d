module bullet2.LinearMath.btTransform;

extern (C++):

import bullet2.LinearMath.btScalar;
import bullet2.LinearMath.btMatrix3x3;
import bullet2.LinearMath.btVector3;
import bullet2.LinearMath.btQuaternion;

version (BT_USE_DOUBLE_PRECISION)
    alias btTransformData = btTransformDoubleData;
else
    alias btTransformData = btTransformFloatData;


/**@brief The btTransform class supports rigid transforms with only translation and rotation and no scaling/shear.
 *It can be used in combination with btVector3, btQuaternion and btMatrix3x3 linear algebra classes. */
extern (C++, class)
align(16) struct btTransform
{
	///Storage for the rotation
	btMatrix3x3 m_basis;
	///Storage for the translation
	btVector3 m_origin;

public:
	/**@brief No initialization constructor */
	//this() {}
	/**@brief Constructor from btQuaternion (optional btVector3 )
   * @param q Rotation from quaternion
   * @param c Translation from Vector (default 0,0,0) */
	/*explicit SIMD_FORCE_INLINE*/ this(in btQuaternion q,
										   /*const*/ btVector3 c = btVector3(0, 0, 0))
	{
        m_basis = q;
        m_origin = c;
	}

	/**@brief Constructor from btMatrix3x3 (optional btVector3)
   * @param b Rotation from Matrix
   * @param c Translation from Vector default (0,0,0)*/
	/*explicit SIMD_FORCE_INLINE*/ this(in btMatrix3x3 b,
										   /*const*/ btVector3 c = btVector3(0, 0, 0))
	{
        m_basis = b;
        m_origin = c;
	}
	/**@brief Copy constructor */
	/*SIMD_FORCE_INLINE*/ this(in btTransform other)
	{
        m_basis = other.m_basis;
        m_origin = other.m_origin;
	}
	/**@brief Assignment Operator */
	///*SIMD_FORCE_INLINE*/ ref btTransform operator=(const ref btTransform other)
	//extern (D)
    void opAssign(in btTransform other)
	{
		m_basis = other.m_basis;
		m_origin = other.m_origin;
	}

	/**@brief Set the current transform as the value of the product of two transforms
   * @param t1 Transform 1
   * @param t2 Transform 2
   * This = Transform1 * Transform2 */
	/*SIMD_FORCE_INLINE*/ void mult(ref btTransform t1, ref btTransform t2)
	{
		m_basis = t1.m_basis * t2.m_basis;
		m_origin = t1(t2.m_origin);
	}

	/*		void multInverseLeft(const ref btTransform t1, const ref btTransform t2) {
			btVector3 v = t2.m_origin - t1.m_origin;
			m_basis = btMultTransposeLeft(t1.m_basis, t2.m_basis);
			m_origin = v * t1.m_basis;
		}
		*/

	/**@brief Return the transform of the vector */
	///*SIMD_FORCE_INLINE*/ btVector3 operator()(const ref btVector3 x) const
	//extern (D)
    btVector3 opCall(ref const(btVector3) x) const
	{
		return x.dot3(m_basis[0], m_basis[1], m_basis[2]) + m_origin;
	}

	/**@brief Return the transform of the vector */
	/+/*SIMD_FORCE_INLINE*/ btVector3 operator*(const ref btVector3 x) const
	{
		return (*this)(x);
	}

	/**@brief Return the transform of the btQuaternion */
	/*SIMD_FORCE_INLINE*/ btQuaternion operator*(const ref btQuaternion q) const
	{
		return getRotation() * q;
	}+/
	//extern (D)
    auto ref btVector3 opBinary(string op)(in btVector3 x)
    {
        static if (op == "*")
            return opCall(x);
        else
            static assert("operator '" ~ "' not implemented");
    }

	//extern (D)
    auto ref btQuaternion opBinary(string op)(in btQuaternion q)
    {
        static if (op == "*")
            return getRotation() * q;
        else
            static assert("operator '" ~ "' not implemented");
    }

	/**@brief Return the basis matrix for the rotation */
	/*SIMD_FORCE_INLINE*/ auto ref btMatrix3x3 getBasis() const { return m_basis; }
	extern (D) ref btMatrix3x3 getBasis() return { return m_basis; }

	/**@brief Return the basis matrix for the rotation */
	///*SIMD_FORCE_INLINE*/ const ref btMatrix3x3 getBasis() { return m_basis; }

	/**@brief Return the origin vector translation */
	/*SIMD_FORCE_INLINE*/ auto ref btVector3 getOrigin() const { return m_origin; }
	extern (D) ref btVector3 getOrigin() return { return m_origin; }

	/**@brief Return the origin vector translation */
	///*SIMD_FORCE_INLINE*/ const ref btVector3 getOrigin() { return m_origin; }

	/**@brief Return a quaternion representing the rotation */
	btQuaternion getRotation() const
	{
		btQuaternion q;
		m_basis.getRotation(q);
		return q;
	}

	/**@brief Set from an array
   * @param m A pointer to a 16 element array (12 rotation(row major padded on the right by 1), and 3 translation */
	void setFromOpenGLMatrix(const btScalar* m)
	{
		m_basis.setFromOpenGLSubMatrix(m);
		m_origin.setValue(m[12], m[13], m[14]);
	}

	/**@brief Fill an array representation
   * @param m A pointer to a 16 element array (12 rotation(row major padded on the right by 1), and 3 translation */
	void getOpenGLMatrix(btScalar * m) const
	{
		m_basis.getOpenGLSubMatrix(m);
		m[12] = m_origin.x();
		m[13] = m_origin.y();
		m[14] = m_origin.z();
		m[15] = btScalar(1.0);
	}

	/**@brief Set the translational element
   * @param origin The vector to set the translation to */
	/*SIMD_FORCE_INLINE*/ void setOrigin(btVector3 origin)
	{
		m_origin = origin;
	}

	/*SIMD_FORCE_INLINE*/ btVector3 invXform(ref btVector3 inVec) const
    {
        btVector3 v = inVec - m_origin;
        return (m_basis.transpose() * v);
    }

	/**@brief Set the rotational element by btMatrix3x3 */
	/*SIMD_FORCE_INLINE*/ void setBasis(ref const(btMatrix3x3) basis)
	{
		m_basis = basis;
	}

	/**@brief Set the rotational element by btQuaternion */
	/*SIMD_FORCE_INLINE*/ void setRotation(ref const(btQuaternion) q)
	{
		m_basis.setRotation(q);
	}

	/**@brief Set this transformation to the identity */
	void setIdentity()
	{
		m_basis.setIdentity();
		m_origin.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	}

	/**@brief Multiply this Transform by another(this = this * another)
   * @param t The other transform */
	//ref btTransform operator*=(const ref btTransform t)
    ref btTransform opOpAssign(string op)(const ref btTransform t)
	{
		m_origin += m_basis * t.m_origin;
		m_basis *= t.m_basis;
		return *this;
	}

	/**@brief Return the inverse of this transform */
	btTransform inverse() const
	{
		btMatrix3x3 inv = m_basis.transpose();
		return btTransform(inv, inv * -m_origin);
	}

	/**@brief Return the inverse of this transform times the other transform
   * @param t The other transform
   * return this.inverse() * the other */
	btTransform inverseTimes(ref btTransform t) //const
    {
        btVector3 v = t.getOrigin() - m_origin;
		auto wat = v * m_basis;
        return btTransform(m_basis.transposeTimes(t.m_basis),
                        wat);
    }

	/**@brief Return the product of this transform and the other */
	//extern (D)
    auto ref btTransform opBinary(string op)(in btTransform t) const
    {
        static if (op == "*")
            return btTransform(m_basis * t.m_basis,
                    opCall(t.m_origin));
        else
            static assert("operator '" ~ "' not implemented");
    }

	/**@brief Return an identity transform */
	static btTransform getIdentity()
	{
		static const btTransform identityTransform = btMatrix3x3.getIdentity();
		return identityTransform;
	}

	void serialize(ref btTransformData dataOut) const
    {
        m_basis.serialize(dataOut.m_basis);
        m_origin.serialize(dataOut.m_origin);
    }

	void serializeFloat(ref btTransformFloatData dataOut) const
    {
        m_basis.serializeFloat(dataOut.m_basis);
        m_origin.serializeFloat(dataOut.m_origin);
    }

	void deSerialize(const ref btTransformData dataIn)
    {
        m_basis.deSerialize(dataIn.m_basis);
        m_origin.deSerialize(dataIn.m_origin);
    }

	void deSerializeDouble(const ref btTransformDoubleData dataIn)
    {
        m_basis.deSerializeDouble(dataIn.m_basis);
        m_origin.deSerializeDouble(dataIn.m_origin);
    }

	void deSerializeFloat(const ref btTransformFloatData dataIn)
    {
		m_basis.deSerializeFloat(dataIn.m_basis);
        m_origin.deSerializeFloat(dataIn.m_origin);
    }
};

/**@brief Test if two transforms have all elements equal */
///*SIMD_FORCE_INLINE*/ bool operator==(const ref btTransform t1, const ref btTransform t2)
bool opEquals(ref btTransform t1, ref btTransform t2)
{
	return (t1.getBasis() == t2.getBasis() &&
			t1.getOrigin() == t2.getOrigin());
}

///for serialization
struct btTransformFloatData
{
	btMatrix3x3FloatData m_basis;
	btVector3FloatData m_origin;
};

struct btTransformDoubleData
{
	btMatrix3x3DoubleData m_basis;
	btVector3DoubleData m_origin;
};
