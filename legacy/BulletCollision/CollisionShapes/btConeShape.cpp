/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btConeShape.h"



btlConeShape::btlConeShape (btlScalar radius,btlScalar height): btlConvexInternalShape (),
m_radius (radius),
m_height(height)
{
	m_shapeType = CONE_SHAPE_PROXYTYPE;
	setConeUpIndex(1);
	btlVector3 halfExtents;
	m_sinAngle = (m_radius / btlSqrt(m_radius * m_radius + m_height * m_height));
}

btlConeShapeZ::btlConeShapeZ (btlScalar radius,btlScalar height):
btlConeShape(radius,height)
{
	setConeUpIndex(2);
}

btlConeShapeX::btlConeShapeX (btlScalar radius,btlScalar height):
btlConeShape(radius,height)
{
	setConeUpIndex(0);
}

///choose upAxis index
void	btlConeShape::setConeUpIndex(int upIndex)
{
	switch (upIndex)
	{
	case 0:
			m_coneIndices[0] = 1;
			m_coneIndices[1] = 0;
			m_coneIndices[2] = 2;
		break;
	case 1:
			m_coneIndices[0] = 0;
			m_coneIndices[1] = 1;
			m_coneIndices[2] = 2;
		break;
	case 2:
			m_coneIndices[0] = 0;
			m_coneIndices[1] = 2;
			m_coneIndices[2] = 1;
		break;
	default:
		btlAssert(0);
	};
	
	m_implicitShapeDimensions[m_coneIndices[0]] = m_radius;
	m_implicitShapeDimensions[m_coneIndices[1]] = m_height;
	m_implicitShapeDimensions[m_coneIndices[2]] = m_radius;
}

btlVector3 btlConeShape::coneLocalSupport(const btlVector3& v) const
{
	
	btlScalar halfHeight = m_height * btlScalar(0.5);

 if (v[m_coneIndices[1]] > v.length() * m_sinAngle)
 {
	btlVector3 tmp;

	tmp[m_coneIndices[0]] = btlScalar(0.);
	tmp[m_coneIndices[1]] = halfHeight;
	tmp[m_coneIndices[2]] = btlScalar(0.);
	return tmp;
 }
  else {
    btlScalar s = btlSqrt(v[m_coneIndices[0]] * v[m_coneIndices[0]] + v[m_coneIndices[2]] * v[m_coneIndices[2]]);
    if (s > SIMD_EPSILON) {
      btlScalar d = m_radius / s;
	  btlVector3 tmp;
	  tmp[m_coneIndices[0]] = v[m_coneIndices[0]] * d;
	  tmp[m_coneIndices[1]] = -halfHeight;
	  tmp[m_coneIndices[2]] = v[m_coneIndices[2]] * d;
	  return tmp;
    }
    else  {
		btlVector3 tmp;
		tmp[m_coneIndices[0]] = btlScalar(0.);
		tmp[m_coneIndices[1]] = -halfHeight;
		tmp[m_coneIndices[2]] = btlScalar(0.);
		return tmp;
	}
  }

}

btlVector3	btlConeShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec) const
{
		return coneLocalSupport(vec);
}

void	btlConeShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		const btlVector3& vec = vectors[i];
		supportVerticesOut[i] = coneLocalSupport(vec);
	}
}


btlVector3	btlConeShape::localGetSupportingVertex(const btlVector3& vec)  const
{
	btlVector3 supVertex = coneLocalSupport(vec);
	if ( getMargin()!=btlScalar(0.) )
	{
		btlVector3 vecnorm = vec;
		if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
		{
			vecnorm.setValue(btlScalar(-1.),btlScalar(-1.),btlScalar(-1.));
		} 
		vecnorm.normalize();
		supVertex+= getMargin() * vecnorm;
	}
	return supVertex;
}


void	btlConeShape::setLocalScaling(const btlVector3& scaling)
{
	int axis = m_coneIndices[1];
	int r1 = m_coneIndices[0];
	int r2 = m_coneIndices[2];
	m_height *= scaling[axis] / m_localScaling[axis];
	m_radius *= (scaling[r1] / m_localScaling[r1] + scaling[r2] / m_localScaling[r2]) / 2;
	m_sinAngle = (m_radius / btlSqrt(m_radius * m_radius + m_height * m_height));
	btlConvexInternalShape::setLocalScaling(scaling);
}