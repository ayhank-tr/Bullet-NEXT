/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_TRANSFORM_H
#define BT_TRANSFORM_H


#include "btMatrix3x3.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define btlTransformData btlTransformDoubleData
#else
#define btlTransformData btlTransformFloatData
#endif




/**@brief The btTransform class supports rigid transforms with only translation and rotation and no scaling/shear.
 *It can be used in combination with btVector3, btQuaternion and btMatrix3x3 linear algebra classes. */
ATTRIBUTE_ALIGNED16(class) btlTransform {
	
  ///Storage for the rotation
	btlMatrix3x3 m_basis;
  ///Storage for the translation
	btlVector3   m_origin;

public:
	
  /**@brief No initialization constructor */
	btlTransform() {}
  /**@brief Constructor from btQuaternion (optional btVector3 )
   * @param q Rotation from quaternion 
   * @param c Translation from Vector (default 0,0,0) */
	explicit SIMD_FORCE_INLINE btlTransform(const btlQuaternion& q, 
		const btlVector3& c = btlVector3(btlScalar(0), btlScalar(0), btlScalar(0))) 
		: m_basis(q),
		m_origin(c)
	{}

  /**@brief Constructor from btMatrix3x3 (optional btVector3)
   * @param b Rotation from Matrix 
   * @param c Translation from Vector default (0,0,0)*/
	explicit SIMD_FORCE_INLINE btlTransform(const btlMatrix3x3& b, 
		const btlVector3& c = btlVector3(btlScalar(0), btlScalar(0), btlScalar(0)))
		: m_basis(b),
		m_origin(c)
	{}
  /**@brief Copy constructor */
	SIMD_FORCE_INLINE btlTransform (const btlTransform& other)
		: m_basis(other.m_basis),
		m_origin(other.m_origin)
	{
	}
  /**@brief Assignment Operator */
	SIMD_FORCE_INLINE btlTransform& operator=(const btlTransform& other)
	{
		m_basis = other.m_basis;
		m_origin = other.m_origin;
		return *this;
	}


  /**@brief Set the current transform as the value of the product of two transforms
   * @param t1 Transform 1
   * @param t2 Transform 2
   * This = Transform1 * Transform2 */
		SIMD_FORCE_INLINE void mult(const btlTransform& t1, const btlTransform& t2) {
			m_basis = t1.m_basis * t2.m_basis;
			m_origin = t1(t2.m_origin);
		}

/*		void multInverseLeft(const btTransform& t1, const btTransform& t2) {
			btVector3 v = t2.m_origin - t1.m_origin;
			m_basis = btMultTransposeLeft(t1.m_basis, t2.m_basis);
			m_origin = v * t1.m_basis;
		}
		*/

/**@brief Return the transform of the vector */
	SIMD_FORCE_INLINE btlVector3 operator()(const btlVector3& x) const
	{
        return x.dot3(m_basis[0], m_basis[1], m_basis[2]) + m_origin;
	}

  /**@brief Return the transform of the vector */
	SIMD_FORCE_INLINE btlVector3 operator*(const btlVector3& x) const
	{
		return (*this)(x);
	}

  /**@brief Return the transform of the btQuaternion */
	SIMD_FORCE_INLINE btlQuaternion operator*(const btlQuaternion& q) const
	{
		return getRotation() * q;
	}

  /**@brief Return the basis matrix for the rotation */
	SIMD_FORCE_INLINE btlMatrix3x3&       getBasis()          { return m_basis; }
  /**@brief Return the basis matrix for the rotation */
	SIMD_FORCE_INLINE const btlMatrix3x3& getBasis()    const { return m_basis; }

  /**@brief Return the origin vector translation */
	SIMD_FORCE_INLINE btlVector3&         getOrigin()         { return m_origin; }
  /**@brief Return the origin vector translation */
	SIMD_FORCE_INLINE const btlVector3&   getOrigin()   const { return m_origin; }

  /**@brief Return a quaternion representing the rotation */
	btlQuaternion getRotation() const { 
		btlQuaternion q;
		m_basis.getRotation(q);
		return q;
	}
	
	
  /**@brief Set from an array 
   * @param m A pointer to a 16 element array (12 rotation(row major padded on the right by 1), and 3 translation */
	void setFromOpenGLMatrix(const btlScalar *m)
	{
		m_basis.setFromOpenGLSubMatrix(m);
		m_origin.setValue(m[12],m[13],m[14]);
	}

  /**@brief Fill an array representation
   * @param m A pointer to a 16 element array (12 rotation(row major padded on the right by 1), and 3 translation */
	void getOpenGLMatrix(btlScalar *m) const 
	{
		m_basis.getOpenGLSubMatrix(m);
		m[12] = m_origin.x();
		m[13] = m_origin.y();
		m[14] = m_origin.z();
		m[15] = btlScalar(1.0);
	}

  /**@brief Set the translational element
   * @param origin The vector to set the translation to */
	SIMD_FORCE_INLINE void setOrigin(const btlVector3& origin) 
	{ 
		m_origin = origin;
	}

	SIMD_FORCE_INLINE btlVector3 invXform(const btlVector3& inVec) const;


  /**@brief Set the rotational element by btMatrix3x3 */
	SIMD_FORCE_INLINE void setBasis(const btlMatrix3x3& basis)
	{ 
		m_basis = basis;
	}

  /**@brief Set the rotational element by btQuaternion */
	SIMD_FORCE_INLINE void setRotation(const btlQuaternion& q)
	{
		m_basis.setRotation(q);
	}


  /**@brief Set this transformation to the identity */
	void setIdentity()
	{
		m_basis.setIdentity();
		m_origin.setValue(btlScalar(0.0), btlScalar(0.0), btlScalar(0.0));
	}

  /**@brief Multiply this Transform by another(this = this * another) 
   * @param t The other transform */
	btlTransform& operator*=(const btlTransform& t) 
	{
		m_origin += m_basis * t.m_origin;
		m_basis *= t.m_basis;
		return *this;
	}

  /**@brief Return the inverse of this transform */
	btlTransform inverse() const
	{ 
		btlMatrix3x3 inv = m_basis.transpose();
		return btlTransform(inv, inv * -m_origin);
	}

  /**@brief Return the inverse of this transform times the other transform
   * @param t The other transform 
   * return this.inverse() * the other */
	btlTransform inverseTimes(const btlTransform& t) const;  

  /**@brief Return the product of this transform and the other */
	btlTransform operator*(const btlTransform& t) const;

  /**@brief Return an identity transform */
	static const btlTransform&	getIdentity()
	{
		static const btlTransform identityTransform(btlMatrix3x3::getIdentity());
		return identityTransform;
	}

	void	serialize(struct	btlTransformData& dataOut) const;

	void	serializeFloat(struct	btlTransformFloatData& dataOut) const;

	void	deSerialize(const struct	btlTransformData& dataIn);

	void	deSerializeDouble(const struct	btlTransformDoubleData& dataIn);

	void	deSerializeFloat(const struct	btlTransformFloatData& dataIn);

};


SIMD_FORCE_INLINE btlVector3
btlTransform::invXform(const btlVector3& inVec) const
{
	btlVector3 v = inVec - m_origin;
	return (m_basis.transpose() * v);
}

SIMD_FORCE_INLINE btlTransform 
btlTransform::inverseTimes(const btlTransform& t) const  
{
	btlVector3 v = t.getOrigin() - m_origin;
		return btlTransform(m_basis.transposeTimes(t.m_basis),
			v * m_basis);
}

SIMD_FORCE_INLINE btlTransform 
btlTransform::operator*(const btlTransform& t) const
{
	return btlTransform(m_basis * t.m_basis, 
		(*this)(t.m_origin));
}

/**@brief Test if two transforms have all elements equal */
SIMD_FORCE_INLINE bool operator==(const btlTransform& t1, const btlTransform& t2)
{
   return ( t1.getBasis()  == t2.getBasis() &&
            t1.getOrigin() == t2.getOrigin() );
}


///for serialization
struct	btlTransformFloatData
{
	btlMatrix3x3FloatData	m_basis;
	btlVector3FloatData	m_origin;
};

struct	btlTransformDoubleData
{
	btlMatrix3x3DoubleData	m_basis;
	btlVector3DoubleData	m_origin;
};



SIMD_FORCE_INLINE	void	btlTransform::serialize(btlTransformData& dataOut) const
{
	m_basis.serialize(dataOut.m_basis);
	m_origin.serialize(dataOut.m_origin);
}

SIMD_FORCE_INLINE	void	btlTransform::serializeFloat(btlTransformFloatData& dataOut) const
{
	m_basis.serializeFloat(dataOut.m_basis);
	m_origin.serializeFloat(dataOut.m_origin);
}


SIMD_FORCE_INLINE	void	btlTransform::deSerialize(const btlTransformData& dataIn)
{
	m_basis.deSerialize(dataIn.m_basis);
	m_origin.deSerialize(dataIn.m_origin);
}

SIMD_FORCE_INLINE	void	btlTransform::deSerializeFloat(const btlTransformFloatData& dataIn)
{
	m_basis.deSerializeFloat(dataIn.m_basis);
	m_origin.deSerializeFloat(dataIn.m_origin);
}

SIMD_FORCE_INLINE	void	btlTransform::deSerializeDouble(const btlTransformDoubleData& dataIn)
{
	m_basis.deSerializeDouble(dataIn.m_basis);
	m_origin.deSerializeDouble(dataIn.m_origin);
}


#endif //BT_TRANSFORM_H






