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

#include "btCylinderShape.h"

btlCylinderShape::btlCylinderShape (const btlVector3& halfExtents)
:btlConvexInternalShape(),
m_upAxis(1)
{
	setSafeMargin(halfExtents);

	btlVector3 margin(getMargin(),getMargin(),getMargin());
	m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;
	m_shapeType = CYLINDER_SHAPE_PROXYTYPE;
}


btlCylinderShapeX::btlCylinderShapeX (const btlVector3& halfExtents)
:btlCylinderShape(halfExtents)
{
	m_upAxis = 0;

}


btlCylinderShapeZ::btlCylinderShapeZ (const btlVector3& halfExtents)
:btlCylinderShape(halfExtents)
{
	m_upAxis = 2;

}

void btlCylinderShape::getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const
{
	btlTransformAabb(getHalfExtentsWithoutMargin(),getMargin(),t,aabbMin,aabbMax);
}

void	btlCylinderShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{

//Until Bullet 2.77 a box approximation was used, so uncomment this if you need backwards compatibility
//#define USE_BOX_INERTIA_APPROXIMATION 1
#ifndef USE_BOX_INERTIA_APPROXIMATION

	/*
	cylinder is defined as following:
	*
	* - principle axis aligned along y by default, radius in x, z-value not used
	* - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
	* - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used
	*
	*/

	btlScalar radius2;	// square of cylinder radius
	btlScalar height2;	// square of cylinder height
	btlVector3 halfExtents = getHalfExtentsWithMargin();	// get cylinder dimension
	btlScalar div12 = mass / 12.f;
	btlScalar div4 = mass / 4.f;
	btlScalar div2 = mass / 2.f;
	int idxRadius, idxHeight;

	switch (m_upAxis)	// get indices of radius and height of cylinder
	{
		case 0:		// cylinder is aligned along x
			idxRadius = 1;
			idxHeight = 0;
			break;
		case 2:		// cylinder is aligned along z
			idxRadius = 0;
			idxHeight = 2;
			break;
		default:	// cylinder is aligned along y
			idxRadius = 0;
			idxHeight = 1;
	}

	// calculate squares
	radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
	height2 = btlScalar(4.) * halfExtents[idxHeight] * halfExtents[idxHeight];

	// calculate tensor terms
	btlScalar t1 = div12 * height2 + div4 * radius2;
	btlScalar t2 = div2 * radius2;

	switch (m_upAxis)	// set diagonal elements of inertia tensor
	{
		case 0:		// cylinder is aligned along x
			inertia.setValue(t2,t1,t1);
			break;
		case 2:		// cylinder is aligned along z
			inertia.setValue(t1,t1,t2);
			break;
		default:	// cylinder is aligned along y
			inertia.setValue(t1,t2,t1);
	}
#else //USE_BOX_INERTIA_APPROXIMATION
	//approximation of box shape
	btlVector3 halfExtents = getHalfExtentsWithMargin();

	btlScalar lx=btlScalar(2.)*(halfExtents.x());
	btlScalar ly=btlScalar(2.)*(halfExtents.y());
	btlScalar lz=btlScalar(2.)*(halfExtents.z());

	inertia.setValue(mass/(btlScalar(12.0)) * (ly*ly + lz*lz),
					mass/(btlScalar(12.0)) * (lx*lx + lz*lz),
					mass/(btlScalar(12.0)) * (lx*lx + ly*ly));
#endif //USE_BOX_INERTIA_APPROXIMATION
}


SIMD_FORCE_INLINE btlVector3 CylinderLocalSupportX(const btlVector3& halfExtents,const btlVector3& v) 
{
const int cylinderUpAxis = 0;
const int XX = 1;
const int YY = 0;
const int ZZ = 2;

	//mapping depends on how cylinder local orientation is
	// extents of the cylinder is: X,Y is for radius, and Z for height


	btlScalar radius = halfExtents[XX];
	btlScalar halfHeight = halfExtents[cylinderUpAxis];


    btlVector3 tmp;
	btlScalar d ;

    btlScalar s = btlSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != btlScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = btlScalar(0.0);
		return tmp;
    }


}






inline  btlVector3 CylinderLocalSupportY(const btlVector3& halfExtents,const btlVector3& v) 
{

const int cylinderUpAxis = 1;
const int XX = 0;
const int YY = 1;
const int ZZ = 2;


	btlScalar radius = halfExtents[XX];
	btlScalar halfHeight = halfExtents[cylinderUpAxis];


    btlVector3 tmp;
	btlScalar d ;

    btlScalar s = btlSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != btlScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = btlScalar(0.0);
		return tmp;
    }

}

inline btlVector3 CylinderLocalSupportZ(const btlVector3& halfExtents,const btlVector3& v) 
{
const int cylinderUpAxis = 2;
const int XX = 0;
const int YY = 2;
const int ZZ = 1;

	//mapping depends on how cylinder local orientation is
	// extents of the cylinder is: X,Y is for radius, and Z for height


	btlScalar radius = halfExtents[XX];
	btlScalar halfHeight = halfExtents[cylinderUpAxis];


    btlVector3 tmp;
	btlScalar d ;

    btlScalar s = btlSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != btlScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = btlScalar(0.0);
		return tmp;
    }


}

btlVector3	btlCylinderShapeX::localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
{
	return CylinderLocalSupportX(getHalfExtentsWithoutMargin(),vec);
}


btlVector3	btlCylinderShapeZ::localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
{
	return CylinderLocalSupportZ(getHalfExtentsWithoutMargin(),vec);
}
btlVector3	btlCylinderShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
{
	return CylinderLocalSupportY(getHalfExtentsWithoutMargin(),vec);
}

void	btlCylinderShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportY(getHalfExtentsWithoutMargin(),vectors[i]);
	}
}

void	btlCylinderShapeZ::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportZ(getHalfExtentsWithoutMargin(),vectors[i]);
	}
}




void	btlCylinderShapeX::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportX(getHalfExtentsWithoutMargin(),vectors[i]);
	}
}


