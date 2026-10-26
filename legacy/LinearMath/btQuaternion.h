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



#ifndef BT_SIMD__QUATERNION_H_
#define BT_SIMD__QUATERNION_H_


#include "btVector3.h"
#include "btQuadWord.h"


#ifdef BT_USE_DOUBLE_PRECISION
#define btlQuaternionData btlQuaternionDoubleData
#define btlQuaternionDataName "btQuaternionDoubleData"
#else
#define btlQuaternionData btlQuaternionFloatData
#define btlQuaternionDataName "btQuaternionFloatData"
#endif //BT_USE_DOUBLE_PRECISION



#ifdef BT_USE_SSE

//const __m128 ATTRIBUTE_ALIGNED16(vOnes) = {1.0f, 1.0f, 1.0f, 1.0f};
#define vOnes (_mm_set_ps(1.0f, 1.0f, 1.0f, 1.0f))

#endif

#if defined(BT_USE_SSE) 

#define vQInv (_mm_set_ps(+0.0f, -0.0f, -0.0f, -0.0f))
#define vPPPM (_mm_set_ps(-0.0f, +0.0f, +0.0f, +0.0f))

#elif defined(BT_USE_NEON)

const btlSimdFloat4 ATTRIBUTE_ALIGNED16(vQInv) = {-0.0f, -0.0f, -0.0f, +0.0f};
const btlSimdFloat4 ATTRIBUTE_ALIGNED16(vPPPM) = {+0.0f, +0.0f, +0.0f, -0.0f};

#endif

/**@brief The btQuaternion implements quaternion to perform linear algebra rotations in combination with btMatrix3x3, btVector3 and btTransform. */
class btlQuaternion : public btlQuadWord {
public:
  /**@brief No initialization constructor */
	btlQuaternion() {}

#if (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))|| defined(BT_USE_NEON) 
	// Set Vector 
	SIMD_FORCE_INLINE btlQuaternion(const btlSimdFloat4 vec)
	{
		mVec128 = vec;
	}

	// Copy constructor
	SIMD_FORCE_INLINE btlQuaternion(const btlQuaternion& rhs)
	{
		mVec128 = rhs.mVec128;
	}

	// Assignment Operator
	SIMD_FORCE_INLINE btlQuaternion& 
	operator=(const btlQuaternion& v) 
	{
		mVec128 = v.mVec128;
		
		return *this;
	}
	
#endif

	//		template <typename btScalar>
	//		explicit Quaternion(const btScalar *v) : Tuple4<btScalar>(v) {}
  /**@brief Constructor from scalars */
	btlQuaternion(const btlScalar& _x, const btlScalar& _y, const btlScalar& _z, const btlScalar& _w) 
		: btlQuadWord(_x, _y, _z, _w) 
	{}
  /**@brief Axis angle Constructor
   * @param axis The axis which the rotation is around
   * @param angle The magnitude of the rotation around the angle (Radians) */
	btlQuaternion(const btlVector3& _axis, const btlScalar& _angle) 
	{ 
		setRotation(_axis, _angle); 
	}
  /**@brief Constructor from Euler angles
   * @param yaw Angle around Y unless BT_EULER_DEFAULT_ZYX defined then Z
   * @param pitch Angle around X unless BT_EULER_DEFAULT_ZYX defined then Y
   * @param roll Angle around Z unless BT_EULER_DEFAULT_ZYX defined then X */
	btlQuaternion(const btlScalar& yaw, const btlScalar& pitch, const btlScalar& roll)
	{ 
#ifndef BT_EULER_DEFAULT_ZYX
		setEuler(yaw, pitch, roll); 
#else
		setEulerZYX(yaw, pitch, roll); 
#endif 
	}
  /**@brief Set the rotation using axis angle notation 
   * @param axis The axis around which to rotate
   * @param angle The magnitude of the rotation in Radians */
	void setRotation(const btlVector3& axis, const btlScalar& _angle)
	{
		btlScalar d = axis.length();
		btlAssert(d != btlScalar(0.0));
		btlScalar s = btlSin(_angle * btlScalar(0.5)) / d;
		setValue(axis.x() * s, axis.y() * s, axis.z() * s, 
			btlCos(_angle * btlScalar(0.5)));
	}
  /**@brief Set the quaternion using Euler angles
   * @param yaw Angle around Y
   * @param pitch Angle around X
   * @param roll Angle around Z */
	void setEuler(const btlScalar& yaw, const btlScalar& pitch, const btlScalar& roll)
	{
		btlScalar halfYaw = btlScalar(yaw) * btlScalar(0.5);  
		btlScalar halfPitch = btlScalar(pitch) * btlScalar(0.5);  
		btlScalar halfRoll = btlScalar(roll) * btlScalar(0.5);  
		btlScalar cosYaw = btlCos(halfYaw);
		btlScalar sinYaw = btlSin(halfYaw);
		btlScalar cosPitch = btlCos(halfPitch);
		btlScalar sinPitch = btlSin(halfPitch);
		btlScalar cosRoll = btlCos(halfRoll);
		btlScalar sinRoll = btlSin(halfRoll);
		setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
			sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
	}
  /**@brief Set the quaternion using euler angles 
   * @param yaw Angle around Z
   * @param pitch Angle around Y
   * @param roll Angle around X */
	void setEulerZYX(const btlScalar& yaw, const btlScalar& pitch, const btlScalar& roll)
	{
		btlScalar halfYaw = btlScalar(yaw) * btlScalar(0.5);  
		btlScalar halfPitch = btlScalar(pitch) * btlScalar(0.5);  
		btlScalar halfRoll = btlScalar(roll) * btlScalar(0.5);  
		btlScalar cosYaw = btlCos(halfYaw);
		btlScalar sinYaw = btlSin(halfYaw);
		btlScalar cosPitch = btlCos(halfPitch);
		btlScalar sinPitch = btlSin(halfPitch);
		btlScalar cosRoll = btlCos(halfRoll);
		btlScalar sinRoll = btlSin(halfRoll);
		setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                         cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                         cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                         cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
	}
  /**@brief Add two quaternions
   * @param q The quaternion to add to this one */
	SIMD_FORCE_INLINE	btlQuaternion& operator+=(const btlQuaternion& q)
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		mVec128 = _mm_add_ps(mVec128, q.mVec128);
#elif defined(BT_USE_NEON)
		mVec128 = vaddq_f32(mVec128, q.mVec128);
#else	
		m_floats[0] += q.x(); 
        m_floats[1] += q.y(); 
        m_floats[2] += q.z(); 
        m_floats[3] += q.m_floats[3];
#endif
		return *this;
	}

  /**@brief Subtract out a quaternion
   * @param q The quaternion to subtract from this one */
	btlQuaternion& operator-=(const btlQuaternion& q) 
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		mVec128 = _mm_sub_ps(mVec128, q.mVec128);
#elif defined(BT_USE_NEON)
		mVec128 = vsubq_f32(mVec128, q.mVec128);
#else	
		m_floats[0] -= q.x(); 
        m_floats[1] -= q.y(); 
        m_floats[2] -= q.z(); 
        m_floats[3] -= q.m_floats[3];
#endif
        return *this;
	}

  /**@brief Scale this quaternion
   * @param s The scalar to scale by */
	btlQuaternion& operator*=(const btlScalar& s)
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		__m128	vs = _mm_load_ss(&s);	//	(S 0 0 0)
		vs = btl_pshufd_ps(vs, 0);	//	(S S S S)
		mVec128 = _mm_mul_ps(mVec128, vs);
#elif defined(BT_USE_NEON)
		mVec128 = vmulq_n_f32(mVec128, s);
#else
		m_floats[0] *= s; 
        m_floats[1] *= s; 
        m_floats[2] *= s; 
        m_floats[3] *= s;
#endif
		return *this;
	}

  /**@brief Multiply this quaternion by q on the right
   * @param q The other quaternion 
   * Equivilant to this = this * q */
	btlQuaternion& operator*=(const btlQuaternion& q)
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		__m128 vQ2 = q.get128();
		
		__m128 A1 = btl_pshufd_ps(mVec128, BT_SHUFFLE(0,1,2,0));
		__m128 B1 = btl_pshufd_ps(vQ2, BT_SHUFFLE(3,3,3,0));
		
		A1 = A1 * B1;
		
		__m128 A2 = btl_pshufd_ps(mVec128, BT_SHUFFLE(1,2,0,1));
		__m128 B2 = btl_pshufd_ps(vQ2, BT_SHUFFLE(2,0,1,1));
		
		A2 = A2 * B2;
		
		B1 = btl_pshufd_ps(mVec128, BT_SHUFFLE(2,0,1,2));
		B2 = btl_pshufd_ps(vQ2, BT_SHUFFLE(1,2,0,2));
		
		B1 = B1 * B2;	//	A3 *= B3
		
		mVec128 = btl_splat_ps(mVec128, 3);	//	A0
		mVec128 = mVec128 * vQ2;	//	A0 * B0
		
		A1 = A1 + A2;	//	AB12
		mVec128 = mVec128 - B1;	//	AB03 = AB0 - AB3 
		A1 = _mm_xor_ps(A1, vPPPM);	//	change sign of the last element
		mVec128 = mVec128+ A1;	//	AB03 + AB12

#elif defined(BT_USE_NEON)     

        float32x4_t vQ1 = mVec128;
        float32x4_t vQ2 = q.get128();
        float32x4_t A0, A1, B1, A2, B2, A3, B3;
        float32x2_t vQ1zx, vQ2wx, vQ1yz, vQ2zx, vQ2yz, vQ2xz;
        
        {
        float32x2x2_t tmp;
        tmp = vtrn_f32( vget_high_f32(vQ1), vget_low_f32(vQ1) );       // {z x}, {w y}
        vQ1zx = tmp.val[0];

        tmp = vtrn_f32( vget_high_f32(vQ2), vget_low_f32(vQ2) );       // {z x}, {w y}
        vQ2zx = tmp.val[0];
        }
        vQ2wx = vext_f32(vget_high_f32(vQ2), vget_low_f32(vQ2), 1); 

        vQ1yz = vext_f32(vget_low_f32(vQ1), vget_high_f32(vQ1), 1);

        vQ2yz = vext_f32(vget_low_f32(vQ2), vget_high_f32(vQ2), 1);
        vQ2xz = vext_f32(vQ2zx, vQ2zx, 1);

        A1 = vcombine_f32(vget_low_f32(vQ1), vQ1zx);                    // X Y  z x 
        B1 = vcombine_f32(vdup_lane_f32(vget_high_f32(vQ2), 1), vQ2wx); // W W  W X 

        A2 = vcombine_f32(vQ1yz, vget_low_f32(vQ1));
        B2 = vcombine_f32(vQ2zx, vdup_lane_f32(vget_low_f32(vQ2), 1));

        A3 = vcombine_f32(vQ1zx, vQ1yz);        // Z X Y Z
        B3 = vcombine_f32(vQ2yz, vQ2xz);        // Y Z x z

        A1 = vmulq_f32(A1, B1);
        A2 = vmulq_f32(A2, B2);
        A3 = vmulq_f32(A3, B3);	//	A3 *= B3
        A0 = vmulq_lane_f32(vQ2, vget_high_f32(vQ1), 1); //	A0 * B0

        A1 = vaddq_f32(A1, A2);	//	AB12 = AB1 + AB2
        A0 = vsubq_f32(A0, A3);	//	AB03 = AB0 - AB3 
        
        //	change the sign of the last element
        A1 = (btlSimdFloat4)veorq_s32((int32x4_t)A1, (int32x4_t)vPPPM);	
        A0 = vaddq_f32(A0, A1);	//	AB03 + AB12
        
        mVec128 = A0;
#else
		setValue(
            m_floats[3] * q.x() + m_floats[0] * q.m_floats[3] + m_floats[1] * q.z() - m_floats[2] * q.y(),
			m_floats[3] * q.y() + m_floats[1] * q.m_floats[3] + m_floats[2] * q.x() - m_floats[0] * q.z(),
			m_floats[3] * q.z() + m_floats[2] * q.m_floats[3] + m_floats[0] * q.y() - m_floats[1] * q.x(),
			m_floats[3] * q.m_floats[3] - m_floats[0] * q.x() - m_floats[1] * q.y() - m_floats[2] * q.z());
#endif
		return *this;
	}
  /**@brief Return the dot product between this quaternion and another
   * @param q The other quaternion */
	btlScalar dot(const btlQuaternion& q) const
	{
#if defined BT_USE_SIMD_VECTOR3 && defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		__m128	vd;
		
		vd = _mm_mul_ps(mVec128, q.mVec128);
		
        __m128 t = _mm_movehl_ps(vd, vd);
		vd = _mm_add_ps(vd, t);
		t = _mm_shuffle_ps(vd, vd, 0x55);
		vd = _mm_add_ss(vd, t);
		
        return _mm_cvtss_f32(vd);
#elif defined(BT_USE_NEON)
		float32x4_t vd = vmulq_f32(mVec128, q.mVec128);
		float32x2_t x = vpadd_f32(vget_low_f32(vd), vget_high_f32(vd));  
		x = vpadd_f32(x, x);
		return vget_lane_f32(x, 0);
#else    
		return  m_floats[0] * q.x() + 
                m_floats[1] * q.y() + 
                m_floats[2] * q.z() + 
                m_floats[3] * q.m_floats[3];
#endif
	}

  /**@brief Return the length squared of the quaternion */
	btlScalar length2() const
	{
		return dot(*this);
	}

  /**@brief Return the length of the quaternion */
	btlScalar length() const
	{
		return btlSqrt(length2());
	}

  /**@brief Normalize the quaternion 
   * Such that x^2 + y^2 + z^2 +w^2 = 1 */
	btlQuaternion& normalize() 
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		__m128	vd;
		
		vd = _mm_mul_ps(mVec128, mVec128);
		
        __m128 t = _mm_movehl_ps(vd, vd);
		vd = _mm_add_ps(vd, t);
		t = _mm_shuffle_ps(vd, vd, 0x55);
		vd = _mm_add_ss(vd, t);

		vd = _mm_sqrt_ss(vd);
		vd = _mm_div_ss(vOnes, vd);
        vd = btl_pshufd_ps(vd, 0); // splat
		mVec128 = _mm_mul_ps(mVec128, vd);
    
		return *this;
#else    
		return *this /= length();
#endif
	}

  /**@brief Return a scaled version of this quaternion
   * @param s The scale factor */
	SIMD_FORCE_INLINE btlQuaternion
	operator*(const btlScalar& s) const
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		__m128	vs = _mm_load_ss(&s);	//	(S 0 0 0)
		vs = btl_pshufd_ps(vs, 0x00);	//	(S S S S)
		
		return btlQuaternion(_mm_mul_ps(mVec128, vs));
#elif defined(BT_USE_NEON)
		return btlQuaternion(vmulq_n_f32(mVec128, s));
#else
		return btlQuaternion(x() * s, y() * s, z() * s, m_floats[3] * s);
#endif
	}

  /**@brief Return an inversely scaled versionof this quaternion
   * @param s The inverse scale factor */
	btlQuaternion operator/(const btlScalar& s) const
	{
		btlAssert(s != btlScalar(0.0));
		return *this * (btlScalar(1.0) / s);
	}

  /**@brief Inversely scale this quaternion
   * @param s The scale factor */
	btlQuaternion& operator/=(const btlScalar& s) 
	{
		btlAssert(s != btlScalar(0.0));
		return *this *= btlScalar(1.0) / s;
	}

  /**@brief Return a normalized version of this quaternion */
	btlQuaternion normalized() const 
	{
		return *this / length();
	} 
	/**@brief Return the ***half*** angle between this quaternion and the other
   * @param q The other quaternion */
	btlScalar angle(const btlQuaternion& q) const 
	{
		btlScalar s = btlSqrt(length2() * q.length2());
		btlAssert(s != btlScalar(0.0));
		return btlAcos(dot(q) / s);
	}
	
	/**@brief Return the angle between this quaternion and the other along the shortest path
	* @param q The other quaternion */
	btlScalar angleShortestPath(const btlQuaternion& q) const 
	{
		btlScalar s = btlSqrt(length2() * q.length2());
		btlAssert(s != btlScalar(0.0));
		if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
			return btlAcos(dot(-q) / s) * btlScalar(2.0);
		else 
			return btlAcos(dot(q) / s) * btlScalar(2.0);
	}

  /**@brief Return the angle of rotation represented by this quaternion */
	btlScalar getAngle() const 
	{
		btlScalar s = btlScalar(2.) * btlAcos(m_floats[3]);
		return s;
	}

	/**@brief Return the angle of rotation represented by this quaternion along the shortest path*/
	btlScalar getAngleShortestPath() const 
	{
		btlScalar s;
		if (dot(*this) < 0)
			s = btlScalar(2.) * btlAcos(m_floats[3]);
		else
			s = btlScalar(2.) * btlAcos(-m_floats[3]);

		return s;
	}


	/**@brief Return the axis of the rotation represented by this quaternion */
	btlVector3 getAxis() const
	{
		btlScalar s_squared = 1.f-m_floats[3]*m_floats[3];
		
		if (s_squared < btlScalar(10.) * SIMD_EPSILON) //Check for divide by zero
			return btlVector3(1.0, 0.0, 0.0);  // Arbitrary
		btlScalar s = 1.f/btlSqrt(s_squared);
		return btlVector3(m_floats[0] * s, m_floats[1] * s, m_floats[2] * s);
	}

	/**@brief Return the inverse of this quaternion */
	btlQuaternion inverse() const
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		return btlQuaternion(_mm_xor_ps(mVec128, vQInv));
#elif defined(BT_USE_NEON)
        return btlQuaternion((btlSimdFloat4)veorq_s32((int32x4_t)mVec128, (int32x4_t)vQInv));
#else	
		return btlQuaternion(-m_floats[0], -m_floats[1], -m_floats[2], m_floats[3]);
#endif
	}

  /**@brief Return the sum of this quaternion and the other 
   * @param q2 The other quaternion */
	SIMD_FORCE_INLINE btlQuaternion
	operator+(const btlQuaternion& q2) const
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		return btlQuaternion(_mm_add_ps(mVec128, q2.mVec128));
#elif defined(BT_USE_NEON)
        return btlQuaternion(vaddq_f32(mVec128, q2.mVec128));
#else	
		const btlQuaternion& q1 = *this;
		return btlQuaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(), q1.m_floats[3] + q2.m_floats[3]);
#endif
	}

  /**@brief Return the difference between this quaternion and the other 
   * @param q2 The other quaternion */
	SIMD_FORCE_INLINE btlQuaternion
	operator-(const btlQuaternion& q2) const
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		return btlQuaternion(_mm_sub_ps(mVec128, q2.mVec128));
#elif defined(BT_USE_NEON)
        return btlQuaternion(vsubq_f32(mVec128, q2.mVec128));
#else	
		const btlQuaternion& q1 = *this;
		return btlQuaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(), q1.m_floats[3] - q2.m_floats[3]);
#endif
	}

  /**@brief Return the negative of this quaternion 
   * This simply negates each element */
	SIMD_FORCE_INLINE btlQuaternion operator-() const
	{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
		return btlQuaternion(_mm_xor_ps(mVec128, btvMzeroMask));
#elif defined(BT_USE_NEON)
		return btlQuaternion((btlSimdFloat4)veorq_s32((int32x4_t)mVec128, (int32x4_t)btvMzeroMask) );
#else	
		const btlQuaternion& q2 = *this;
		return btlQuaternion( - q2.x(), - q2.y(),  - q2.z(),  - q2.m_floats[3]);
#endif
	}
  /**@todo document this and it's use */
	SIMD_FORCE_INLINE btlQuaternion farthest( const btlQuaternion& qd) const 
	{
		btlQuaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) > sum.dot(sum) )
			return qd;
		return (-qd);
	}

	/**@todo document this and it's use */
	SIMD_FORCE_INLINE btlQuaternion nearest( const btlQuaternion& qd) const 
	{
		btlQuaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) < sum.dot(sum) )
			return qd;
		return (-qd);
	}


  /**@brief Return the quaternion which is the result of Spherical Linear Interpolation between this and the other quaternion
   * @param q The other quaternion to interpolate with 
   * @param t The ratio between this and q to interpolate.  If t = 0 the result is this, if t=1 the result is q.
   * Slerp interpolates assuming constant velocity.  */
	btlQuaternion slerp(const btlQuaternion& q, const btlScalar& t) const
	{
	  btlScalar magnitude = btlSqrt(length2() * q.length2()); 
	  btlAssert(magnitude > btlScalar(0));

    btlScalar product = dot(q) / magnitude;
    if (btlFabs(product) < btlScalar(1))
		{
      // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
      const btlScalar sign = (product < 0) ? btlScalar(-1) : btlScalar(1);

      const btlScalar theta = btlAcos(sign * product);
      const btlScalar s1 = btlSin(sign * t * theta);   
      const btlScalar d = btlScalar(1.0) / btlSin(theta);
      const btlScalar s0 = btlSin((btlScalar(1.0) - t) * theta);

      return btlQuaternion(
          (m_floats[0] * s0 + q.x() * s1) * d,
          (m_floats[1] * s0 + q.y() * s1) * d,
          (m_floats[2] * s0 + q.z() * s1) * d,
          (m_floats[3] * s0 + q.m_floats[3] * s1) * d);
		}
		else
		{
			return *this;
		}
	}

	static const btlQuaternion&	getIdentity()
	{
		static const btlQuaternion identityQuat(btlScalar(0.),btlScalar(0.),btlScalar(0.),btlScalar(1.));
		return identityQuat;
	}

	SIMD_FORCE_INLINE const btlScalar& getW() const { return m_floats[3]; }

	SIMD_FORCE_INLINE	void	serialize(struct	btlQuaternionData& dataOut) const;

	SIMD_FORCE_INLINE	void	deSerialize(const struct	btlQuaternionData& dataIn);

	SIMD_FORCE_INLINE	void	serializeFloat(struct	btlQuaternionFloatData& dataOut) const;

	SIMD_FORCE_INLINE	void	deSerializeFloat(const struct	btlQuaternionFloatData& dataIn);

	SIMD_FORCE_INLINE	void	serializeDouble(struct	btlQuaternionDoubleData& dataOut) const;

	SIMD_FORCE_INLINE	void	deSerializeDouble(const struct	btlQuaternionDoubleData& dataIn);

};





/**@brief Return the product of two quaternions */
SIMD_FORCE_INLINE btlQuaternion
operator*(const btlQuaternion& q1, const btlQuaternion& q2) 
{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	__m128 vQ1 = q1.get128();
	__m128 vQ2 = q2.get128();
	__m128 A0, A1, B1, A2, B2;
    
	A1 = btl_pshufd_ps(vQ1, BT_SHUFFLE(0,1,2,0)); // X Y  z x     //      vtrn
	B1 = btl_pshufd_ps(vQ2, BT_SHUFFLE(3,3,3,0)); // W W  W X     // vdup vext

	A1 = A1 * B1;
	
	A2 = btl_pshufd_ps(vQ1, BT_SHUFFLE(1,2,0,1)); // Y Z  X Y     // vext 
	B2 = btl_pshufd_ps(vQ2, BT_SHUFFLE(2,0,1,1)); // z x  Y Y     // vtrn vdup

	A2 = A2 * B2;

	B1 = btl_pshufd_ps(vQ1, BT_SHUFFLE(2,0,1,2)); // z x Y Z      // vtrn vext
	B2 = btl_pshufd_ps(vQ2, BT_SHUFFLE(1,2,0,2)); // Y Z x z      // vext vtrn
	
	B1 = B1 * B2;	//	A3 *= B3

	A0 = btl_splat_ps(vQ1, 3);	//	A0
	A0 = A0 * vQ2;	//	A0 * B0

	A1 = A1 + A2;	//	AB12
	A0 =  A0 - B1;	//	AB03 = AB0 - AB3 
	
    A1 = _mm_xor_ps(A1, vPPPM);	//	change sign of the last element
	A0 = A0 + A1;	//	AB03 + AB12
	
	return btlQuaternion(A0);

#elif defined(BT_USE_NEON)     

	float32x4_t vQ1 = q1.get128();
	float32x4_t vQ2 = q2.get128();
	float32x4_t A0, A1, B1, A2, B2, A3, B3;
    float32x2_t vQ1zx, vQ2wx, vQ1yz, vQ2zx, vQ2yz, vQ2xz;
    
    {
    float32x2x2_t tmp;
    tmp = vtrn_f32( vget_high_f32(vQ1), vget_low_f32(vQ1) );       // {z x}, {w y}
    vQ1zx = tmp.val[0];

    tmp = vtrn_f32( vget_high_f32(vQ2), vget_low_f32(vQ2) );       // {z x}, {w y}
    vQ2zx = tmp.val[0];
    }
    vQ2wx = vext_f32(vget_high_f32(vQ2), vget_low_f32(vQ2), 1); 

    vQ1yz = vext_f32(vget_low_f32(vQ1), vget_high_f32(vQ1), 1);

    vQ2yz = vext_f32(vget_low_f32(vQ2), vget_high_f32(vQ2), 1);
    vQ2xz = vext_f32(vQ2zx, vQ2zx, 1);

    A1 = vcombine_f32(vget_low_f32(vQ1), vQ1zx);                    // X Y  z x 
    B1 = vcombine_f32(vdup_lane_f32(vget_high_f32(vQ2), 1), vQ2wx); // W W  W X 

	A2 = vcombine_f32(vQ1yz, vget_low_f32(vQ1));
    B2 = vcombine_f32(vQ2zx, vdup_lane_f32(vget_low_f32(vQ2), 1));

    A3 = vcombine_f32(vQ1zx, vQ1yz);        // Z X Y Z
    B3 = vcombine_f32(vQ2yz, vQ2xz);        // Y Z x z

	A1 = vmulq_f32(A1, B1);
	A2 = vmulq_f32(A2, B2);
	A3 = vmulq_f32(A3, B3);	//	A3 *= B3
	A0 = vmulq_lane_f32(vQ2, vget_high_f32(vQ1), 1); //	A0 * B0

	A1 = vaddq_f32(A1, A2);	//	AB12 = AB1 + AB2
	A0 = vsubq_f32(A0, A3);	//	AB03 = AB0 - AB3 
	
    //	change the sign of the last element
    A1 = (btlSimdFloat4)veorq_s32((int32x4_t)A1, (int32x4_t)vPPPM);	
	A0 = vaddq_f32(A0, A1);	//	AB03 + AB12
	
	return btlQuaternion(A0);

#else
	return btlQuaternion(
        q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
		q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
		q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
		q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z()); 
#endif
}

SIMD_FORCE_INLINE btlQuaternion
operator*(const btlQuaternion& q, const btlVector3& w)
{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	__m128 vQ1 = q.get128();
	__m128 vQ2 = w.get128();
	__m128 A1, B1, A2, B2, A3, B3;
	
	A1 = btl_pshufd_ps(vQ1, BT_SHUFFLE(3,3,3,0));
	B1 = btl_pshufd_ps(vQ2, BT_SHUFFLE(0,1,2,0));

	A1 = A1 * B1;
	
	A2 = btl_pshufd_ps(vQ1, BT_SHUFFLE(1,2,0,1));
	B2 = btl_pshufd_ps(vQ2, BT_SHUFFLE(2,0,1,1));

	A2 = A2 * B2;

	A3 = btl_pshufd_ps(vQ1, BT_SHUFFLE(2,0,1,2));
	B3 = btl_pshufd_ps(vQ2, BT_SHUFFLE(1,2,0,2));
	
	A3 = A3 * B3;	//	A3 *= B3

	A1 = A1 + A2;	//	AB12
	A1 = _mm_xor_ps(A1, vPPPM);	//	change sign of the last element
    A1 = A1 - A3;	//	AB123 = AB12 - AB3 
	
	return btlQuaternion(A1);
    
#elif defined(BT_USE_NEON)     

	float32x4_t vQ1 = q.get128();
	float32x4_t vQ2 = w.get128();
	float32x4_t A1, B1, A2, B2, A3, B3;
    float32x2_t vQ1wx, vQ2zx, vQ1yz, vQ2yz, vQ1zx, vQ2xz;
    
    vQ1wx = vext_f32(vget_high_f32(vQ1), vget_low_f32(vQ1), 1); 
    {
    float32x2x2_t tmp;

    tmp = vtrn_f32( vget_high_f32(vQ2), vget_low_f32(vQ2) );       // {z x}, {w y}
    vQ2zx = tmp.val[0];

    tmp = vtrn_f32( vget_high_f32(vQ1), vget_low_f32(vQ1) );       // {z x}, {w y}
    vQ1zx = tmp.val[0];
    }

    vQ1yz = vext_f32(vget_low_f32(vQ1), vget_high_f32(vQ1), 1);

    vQ2yz = vext_f32(vget_low_f32(vQ2), vget_high_f32(vQ2), 1);
    vQ2xz = vext_f32(vQ2zx, vQ2zx, 1);

    A1 = vcombine_f32(vdup_lane_f32(vget_high_f32(vQ1), 1), vQ1wx); // W W  W X 
    B1 = vcombine_f32(vget_low_f32(vQ2), vQ2zx);                    // X Y  z x 

	A2 = vcombine_f32(vQ1yz, vget_low_f32(vQ1));
    B2 = vcombine_f32(vQ2zx, vdup_lane_f32(vget_low_f32(vQ2), 1));

    A3 = vcombine_f32(vQ1zx, vQ1yz);        // Z X Y Z
    B3 = vcombine_f32(vQ2yz, vQ2xz);        // Y Z x z

	A1 = vmulq_f32(A1, B1);
	A2 = vmulq_f32(A2, B2);
	A3 = vmulq_f32(A3, B3);	//	A3 *= B3

	A1 = vaddq_f32(A1, A2);	//	AB12 = AB1 + AB2
	
    //	change the sign of the last element
    A1 = (btlSimdFloat4)veorq_s32((int32x4_t)A1, (int32x4_t)vPPPM);	
	
    A1 = vsubq_f32(A1, A3);	//	AB123 = AB12 - AB3
	
	return btlQuaternion(A1);
    
#else
	return btlQuaternion( 
         q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
		 q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
		 q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
		-q.x() * w.x() - q.y() * w.y() - q.z() * w.z()); 
#endif
}

SIMD_FORCE_INLINE btlQuaternion
operator*(const btlVector3& w, const btlQuaternion& q)
{
#if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	__m128 vQ1 = w.get128();
	__m128 vQ2 = q.get128();
	__m128 A1, B1, A2, B2, A3, B3;
	
	A1 = btl_pshufd_ps(vQ1, BT_SHUFFLE(0,1,2,0));  // X Y  z x
	B1 = btl_pshufd_ps(vQ2, BT_SHUFFLE(3,3,3,0));  // W W  W X 

	A1 = A1 * B1;
	
	A2 = btl_pshufd_ps(vQ1, BT_SHUFFLE(1,2,0,1));
	B2 = btl_pshufd_ps(vQ2, BT_SHUFFLE(2,0,1,1));

	A2 = A2 *B2;

	A3 = btl_pshufd_ps(vQ1, BT_SHUFFLE(2,0,1,2));
	B3 = btl_pshufd_ps(vQ2, BT_SHUFFLE(1,2,0,2));
	
	A3 = A3 * B3;	//	A3 *= B3

	A1 = A1 + A2;	//	AB12
	A1 = _mm_xor_ps(A1, vPPPM);	//	change sign of the last element
	A1 = A1 - A3;	//	AB123 = AB12 - AB3 
	
	return btlQuaternion(A1);

#elif defined(BT_USE_NEON)     

	float32x4_t vQ1 = w.get128();
	float32x4_t vQ2 = q.get128();
	float32x4_t  A1, B1, A2, B2, A3, B3;
    float32x2_t vQ1zx, vQ2wx, vQ1yz, vQ2zx, vQ2yz, vQ2xz;
    
    {
    float32x2x2_t tmp;
   
    tmp = vtrn_f32( vget_high_f32(vQ1), vget_low_f32(vQ1) );       // {z x}, {w y}
    vQ1zx = tmp.val[0];

    tmp = vtrn_f32( vget_high_f32(vQ2), vget_low_f32(vQ2) );       // {z x}, {w y}
    vQ2zx = tmp.val[0];
    }
    vQ2wx = vext_f32(vget_high_f32(vQ2), vget_low_f32(vQ2), 1); 

    vQ1yz = vext_f32(vget_low_f32(vQ1), vget_high_f32(vQ1), 1);

    vQ2yz = vext_f32(vget_low_f32(vQ2), vget_high_f32(vQ2), 1);
    vQ2xz = vext_f32(vQ2zx, vQ2zx, 1);

    A1 = vcombine_f32(vget_low_f32(vQ1), vQ1zx);                    // X Y  z x 
    B1 = vcombine_f32(vdup_lane_f32(vget_high_f32(vQ2), 1), vQ2wx); // W W  W X 

	A2 = vcombine_f32(vQ1yz, vget_low_f32(vQ1));
    B2 = vcombine_f32(vQ2zx, vdup_lane_f32(vget_low_f32(vQ2), 1));

    A3 = vcombine_f32(vQ1zx, vQ1yz);        // Z X Y Z
    B3 = vcombine_f32(vQ2yz, vQ2xz);        // Y Z x z

	A1 = vmulq_f32(A1, B1);
	A2 = vmulq_f32(A2, B2);
	A3 = vmulq_f32(A3, B3);	//	A3 *= B3

	A1 = vaddq_f32(A1, A2);	//	AB12 = AB1 + AB2
	
    //	change the sign of the last element
    A1 = (btlSimdFloat4)veorq_s32((int32x4_t)A1, (int32x4_t)vPPPM);	
	
    A1 = vsubq_f32(A1, A3);	//	AB123 = AB12 - AB3
	
	return btlQuaternion(A1);
    
#else
	return btlQuaternion( 
        +w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
		+w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
		+w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
		-w.x() * q.x() - w.y() * q.y() - w.z() * q.z()); 
#endif
}

/**@brief Calculate the dot product between two quaternions */
SIMD_FORCE_INLINE btlScalar 
dot(const btlQuaternion& q1, const btlQuaternion& q2) 
{ 
	return q1.dot(q2); 
}


/**@brief Return the length of a quaternion */
SIMD_FORCE_INLINE btlScalar
length(const btlQuaternion& q) 
{ 
	return q.length(); 
}

/**@brief Return the angle between two quaternions*/
SIMD_FORCE_INLINE btlScalar
btlAngle(const btlQuaternion& q1, const btlQuaternion& q2) 
{ 
	return q1.angle(q2); 
}

/**@brief Return the inverse of a quaternion*/
SIMD_FORCE_INLINE btlQuaternion
inverse(const btlQuaternion& q) 
{
	return q.inverse();
}

/**@brief Return the result of spherical linear interpolation betwen two quaternions 
 * @param q1 The first quaternion
 * @param q2 The second quaternion 
 * @param t The ration between q1 and q2.  t = 0 return q1, t=1 returns q2 
 * Slerp assumes constant velocity between positions. */
SIMD_FORCE_INLINE btlQuaternion
slerp(const btlQuaternion& q1, const btlQuaternion& q2, const btlScalar& t) 
{
	return q1.slerp(q2, t);
}

SIMD_FORCE_INLINE btlVector3 
btl_quatRotate(const btlQuaternion& rotation, const btlVector3& v) 
{
	btlQuaternion q = rotation * v;
	q *= rotation.inverse();
#if defined BT_USE_SIMD_VECTOR3 && defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
	return btlVector3(_mm_and_ps(q.get128(), btvFFF0fMask));
#elif defined(BT_USE_NEON)
    return btlVector3((float32x4_t)vandq_s32((int32x4_t)q.get128(), btvFFF0Mask));
#else	
	return btlVector3(q.getX(),q.getY(),q.getZ());
#endif
}

SIMD_FORCE_INLINE btlQuaternion 
shortestArcQuat(const btlVector3& v0, const btlVector3& v1) // Game Programming Gems 2.10. make sure v0,v1 are normalized
{
	btlVector3 c = v0.cross(v1);
	btlScalar  d = v0.dot(v1);

	if (d < -1.0 + SIMD_EPSILON)
	{
		btlVector3 n,unused;
		btlPlaneSpace1(v0,n,unused);
		return btlQuaternion(n.x(),n.y(),n.z(),0.0f); // just pick any vector that is orthogonal to v0
	}

	btlScalar  s = btlSqrt((1.0f + d) * 2.0f);
	btlScalar rs = 1.0f / s;

	return btlQuaternion(c.getX()*rs,c.getY()*rs,c.getZ()*rs,s * 0.5f);
}

SIMD_FORCE_INLINE btlQuaternion 
shortestArcQuatNormalize2(btlVector3& v0,btlVector3& v1)
{
	v0.normalize();
	v1.normalize();
	return shortestArcQuat(v0,v1);
}




struct	btlQuaternionFloatData
{
	float	m_floats[4];
};

struct	btlQuaternionDoubleData
{
	double	m_floats[4];

};

SIMD_FORCE_INLINE	void	btlQuaternion::serializeFloat(struct	btlQuaternionFloatData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = float(m_floats[i]);
}

SIMD_FORCE_INLINE void	btlQuaternion::deSerializeFloat(const struct	btlQuaternionFloatData& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = btlScalar(dataIn.m_floats[i]);
}


SIMD_FORCE_INLINE	void	btlQuaternion::serializeDouble(struct	btlQuaternionDoubleData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = double(m_floats[i]);
}

SIMD_FORCE_INLINE void	btlQuaternion::deSerializeDouble(const struct	btlQuaternionDoubleData& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = btlScalar(dataIn.m_floats[i]);
}


SIMD_FORCE_INLINE	void	btlQuaternion::serialize(struct	btlQuaternionData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = m_floats[i];
}

SIMD_FORCE_INLINE void	btlQuaternion::deSerialize(const struct	btlQuaternionData& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = dataIn.m_floats[i];
}


#endif //BT_SIMD__QUATERNION_H_



