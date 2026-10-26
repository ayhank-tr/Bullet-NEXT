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

#include "btHeightfieldTerrainShape.h"

#include "LinearMath/btTransformUtil.h"



btlHeightfieldTerrainShape::btlHeightfieldTerrainShape
(
int heightStickWidth, int heightStickLength, const void* heightfieldData,
btlScalar heightScale, btlScalar minHeight, btlScalar maxHeight,int upAxis,
PHY_ScalarType hdt, bool flipQuadEdges
)
{
	initialize(heightStickWidth, heightStickLength, heightfieldData,
	           heightScale, minHeight, maxHeight, upAxis, hdt,
	           flipQuadEdges);
}



btlHeightfieldTerrainShape::btlHeightfieldTerrainShape(int heightStickWidth, int heightStickLength,const void* heightfieldData,btlScalar maxHeight,int upAxis,bool useFloatData,bool flipQuadEdges)
{
	// legacy constructor: support only float or unsigned char,
	// 	and min height is zero
	PHY_ScalarType hdt = (useFloatData) ? PHY_FLOAT : PHY_UCHAR;
	btlScalar minHeight = 0.0f;

	// previously, height = uchar * maxHeight / 65535.
	// So to preserve legacy behavior, heightScale = maxHeight / 65535
	btlScalar heightScale = maxHeight / 65535;

	initialize(heightStickWidth, heightStickLength, heightfieldData,
	           heightScale, minHeight, maxHeight, upAxis, hdt,
	           flipQuadEdges);
}



void btlHeightfieldTerrainShape::initialize
(
int heightStickWidth, int heightStickLength, const void* heightfieldData,
btlScalar heightScale, btlScalar minHeight, btlScalar maxHeight, int upAxis,
PHY_ScalarType hdt, bool flipQuadEdges
)
{
	// validation
	btlAssert(heightStickWidth > 1);// && "bad width");
	btlAssert(heightStickLength > 1);// && "bad length");
	btlAssert(heightfieldData);// && "null heightfield data");
	// btAssert(heightScale) -- do we care?  Trust caller here
	btlAssert(minHeight <= maxHeight);// && "bad min/max height");
	btlAssert(upAxis >= 0 && upAxis < 3);// && "bad upAxis--should be in range [0,2]");
	btlAssert(hdt != PHY_UCHAR || hdt != PHY_FLOAT || hdt != PHY_SHORT);// && "Bad height data type enum");

	// initialize member variables
	m_shapeType = TERRAIN_SHAPE_PROXYTYPE;
	m_heightStickWidth = heightStickWidth;
	m_heightStickLength = heightStickLength;
	m_minHeight = minHeight;
	m_maxHeight = maxHeight;
	m_width = (btlScalar) (heightStickWidth - 1);
	m_length = (btlScalar) (heightStickLength - 1);
	m_heightScale = heightScale;
	m_heightfieldDataUnknown = heightfieldData;
	m_heightDataType = hdt;
	m_flipQuadEdges = flipQuadEdges;
	m_useDiamondSubdivision = false;
	m_useZigzagSubdivision = false;
	m_upAxis = upAxis;
	m_localScaling.setValue(btlScalar(1.), btlScalar(1.), btlScalar(1.));

	// determine min/max axis-aligned bounding box (aabb) values
	switch (m_upAxis)
	{
	case 0:
		{
			m_localAabbMin.setValue(m_minHeight, 0, 0);
			m_localAabbMax.setValue(m_maxHeight, m_width, m_length);
			break;
		}
	case 1:
		{
			m_localAabbMin.setValue(0, m_minHeight, 0);
			m_localAabbMax.setValue(m_width, m_maxHeight, m_length);
			break;
		};
	case 2:
		{
			m_localAabbMin.setValue(0, 0, m_minHeight);
			m_localAabbMax.setValue(m_width, m_length, m_maxHeight);
			break;
		}
	default:
		{
			//need to get valid m_upAxis
			btlAssert(0);// && "Bad m_upAxis");
		}
	}

	// remember origin (defined as exact middle of aabb)
	m_localOrigin = btlScalar(0.5) * (m_localAabbMin + m_localAabbMax);
}



btlHeightfieldTerrainShape::~btlHeightfieldTerrainShape()
{
}



void btlHeightfieldTerrainShape::getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const
{
	btlVector3 halfExtents = (m_localAabbMax-m_localAabbMin)* m_localScaling * btlScalar(0.5);

	btlVector3 localOrigin(0, 0, 0);
	localOrigin[m_upAxis] = (m_minHeight + m_maxHeight) * btlScalar(0.5);
	localOrigin *= m_localScaling;

	btlMatrix3x3 abs_b = t.getBasis().absolute();  
	btlVector3 center = t.getOrigin();
    btlVector3 extent = halfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
	extent += btlVector3(getMargin(),getMargin(),getMargin());

	aabbMin = center - extent;
	aabbMax = center + extent;
}


/// This returns the "raw" (user's initial) height, not the actual height.
/// The actual height needs to be adjusted to be relative to the center
///   of the heightfield's AABB.
btlScalar
btlHeightfieldTerrainShape::getRawHeightFieldValue(int x,int y) const
{
	btlScalar val = 0.f;
	switch (m_heightDataType)
	{
	case PHY_FLOAT:
		{
			val = m_heightfieldDataFloat[(y*m_heightStickWidth)+x];
			break;
		}

	case PHY_UCHAR:
		{
			unsigned char heightFieldValue = m_heightfieldDataUnsignedChar[(y*m_heightStickWidth)+x];
			val = heightFieldValue * m_heightScale;
			break;
		}

	case PHY_SHORT:
		{
			short hfValue = m_heightfieldDataShort[(y * m_heightStickWidth) + x];
			val = hfValue * m_heightScale;
			break;
		}

	default:
		{
			btlAssert(!"Bad m_heightDataType");
		}
	}

	return val;
}




/// this returns the vertex in bullet-local coordinates
void	btlHeightfieldTerrainShape::getVertex(int x,int y,btlVector3& vertex) const
{
	btlAssert(x>=0);
	btlAssert(y>=0);
	btlAssert(x<m_heightStickWidth);
	btlAssert(y<m_heightStickLength);

	btlScalar	height = getRawHeightFieldValue(x,y);

	switch (m_upAxis)
	{
	case 0:
		{
		vertex.setValue(
			height - m_localOrigin.getX(),
			(-m_width/btlScalar(2.0)) + x,
			(-m_length/btlScalar(2.0) ) + y
			);
			break;
		}
	case 1:
		{
			vertex.setValue(
			(-m_width/btlScalar(2.0)) + x,
			height - m_localOrigin.getY(),
			(-m_length/btlScalar(2.0)) + y
			);
			break;
		};
	case 2:
		{
			vertex.setValue(
			(-m_width/btlScalar(2.0)) + x,
			(-m_length/btlScalar(2.0)) + y,
			height - m_localOrigin.getZ()
			);
			break;
		}
	default:
		{
			//need to get valid m_upAxis
			btlAssert(0);
		}
	}

	vertex*=m_localScaling;
}



static inline int
getQuantized
(
btlScalar x
)
{
	if (x < 0.0) {
		return (int) (x - 0.5);
	}
	return (int) (x + 0.5);
}



/// given input vector, return quantized version
/**
  This routine is basically determining the gridpoint indices for a given
  input vector, answering the question: "which gridpoint is closest to the
  provided point?".

  "with clamp" means that we restrict the point to be in the heightfield's
  axis-aligned bounding box.
 */
void btlHeightfieldTerrainShape::quantizeWithClamp(int* out, const btlVector3& point,int /*isMax*/) const
{
	btlVector3 clampedPoint(point);
	clampedPoint.setMax(m_localAabbMin);
	clampedPoint.setMin(m_localAabbMax);

	out[0] = getQuantized(clampedPoint.getX());
	out[1] = getQuantized(clampedPoint.getY());
	out[2] = getQuantized(clampedPoint.getZ());
		
}



/// process all triangles within the provided axis-aligned bounding box
/**
  basic algorithm:
    - convert input aabb to local coordinates (scale down and shift for local origin)
    - convert input aabb to a range of heightfield grid points (quantize)
    - iterate over all triangles in that subset of the grid
 */
void	btlHeightfieldTerrainShape::processAllTriangles(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax) const
{
	// scale down the input aabb's so they are in local (non-scaled) coordinates
	btlVector3	localAabbMin = aabbMin*btlVector3(1.f/m_localScaling[0],1.f/m_localScaling[1],1.f/m_localScaling[2]);
	btlVector3	localAabbMax = aabbMax*btlVector3(1.f/m_localScaling[0],1.f/m_localScaling[1],1.f/m_localScaling[2]);

	// account for local origin
	localAabbMin += m_localOrigin;
	localAabbMax += m_localOrigin;

	//quantize the aabbMin and aabbMax, and adjust the start/end ranges
	int	quantizedAabbMin[3];
	int	quantizedAabbMax[3];
	quantizeWithClamp(quantizedAabbMin, localAabbMin,0);
	quantizeWithClamp(quantizedAabbMax, localAabbMax,1);
	
	// expand the min/max quantized values
	// this is to catch the case where the input aabb falls between grid points!
	for (int i = 0; i < 3; ++i) {
		quantizedAabbMin[i]--;
		quantizedAabbMax[i]++;
	}	

	int startX=0;
	int endX=m_heightStickWidth-1;
	int startJ=0;
	int endJ=m_heightStickLength-1;

	switch (m_upAxis)
	{
	case 0:
		{
			if (quantizedAabbMin[1]>startX)
				startX = quantizedAabbMin[1];
			if (quantizedAabbMax[1]<endX)
				endX = quantizedAabbMax[1];
			if (quantizedAabbMin[2]>startJ)
				startJ = quantizedAabbMin[2];
			if (quantizedAabbMax[2]<endJ)
				endJ = quantizedAabbMax[2];
			break;
		}
	case 1:
		{
			if (quantizedAabbMin[0]>startX)
				startX = quantizedAabbMin[0];
			if (quantizedAabbMax[0]<endX)
				endX = quantizedAabbMax[0];
			if (quantizedAabbMin[2]>startJ)
				startJ = quantizedAabbMin[2];
			if (quantizedAabbMax[2]<endJ)
				endJ = quantizedAabbMax[2];
			break;
		};
	case 2:
		{
			if (quantizedAabbMin[0]>startX)
				startX = quantizedAabbMin[0];
			if (quantizedAabbMax[0]<endX)
				endX = quantizedAabbMax[0];
			if (quantizedAabbMin[1]>startJ)
				startJ = quantizedAabbMin[1];
			if (quantizedAabbMax[1]<endJ)
				endJ = quantizedAabbMax[1];
			break;
		}
	default:
		{
			//need to get valid m_upAxis
			btlAssert(0);
		}
	}

	
  

	for(int j=startJ; j<endJ; j++)
	{
		for(int x=startX; x<endX; x++)
		{
			btlVector3 vertices[3];
			if (m_flipQuadEdges || (m_useDiamondSubdivision && !((j+x) & 1))|| (m_useZigzagSubdivision  && !(j & 1)))
			{
        //first triangle
        getVertex(x,j,vertices[0]);
		getVertex(x, j + 1, vertices[1]);
		getVertex(x + 1, j + 1, vertices[2]);
        callback->processTriangle(vertices,x,j);
        //second triangle
      //  getVertex(x,j,vertices[0]);//already got this vertex before, thanks to Danny Chapman
        getVertex(x+1,j+1,vertices[1]);
		getVertex(x + 1, j, vertices[2]);
		callback->processTriangle(vertices, x, j);

			} else
			{
        //first triangle
        getVertex(x,j,vertices[0]);
        getVertex(x,j+1,vertices[1]);
        getVertex(x+1,j,vertices[2]);
        callback->processTriangle(vertices,x,j);
        //second triangle
        getVertex(x+1,j,vertices[0]);
        //getVertex(x,j+1,vertices[1]);
        getVertex(x+1,j+1,vertices[2]);
        callback->processTriangle(vertices,x,j);
			}
		}
	}

	

}

void	btlHeightfieldTerrainShape::calculateLocalInertia(btlScalar ,btlVector3& inertia) const
{
	//moving concave objects not supported
	
	inertia.setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
}

void	btlHeightfieldTerrainShape::setLocalScaling(const btlVector3& scaling)
{
	m_localScaling = scaling;
}
const btlVector3& btlHeightfieldTerrainShape::getLocalScaling() const
{
	return m_localScaling;
}
