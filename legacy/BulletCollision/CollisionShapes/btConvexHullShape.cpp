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

//#if defined (_WIN32) || defined (__i386__)
//#define BT_USE_SSE_IN_API
//#endif

#include "btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"
#include "LinearMath/btSerializer.h"

btlConvexHullShape ::btlConvexHullShape (const btlScalar* points,int numPoints,int stride) : btlPolyhedralConvexAabbCachingShape ()
{
	m_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
	m_unscaledPoints.resize(numPoints);

	unsigned char* pointsAddress = (unsigned char*)points;

	for (int i=0;i<numPoints;i++)
	{
		btlScalar* point = (btlScalar*)pointsAddress;
		m_unscaledPoints[i] = btlVector3(point[0], point[1], point[2]);
		pointsAddress += stride;
	}

	recalcLocalAabb();

}



void btlConvexHullShape::setLocalScaling(const btlVector3& scaling)
{
	m_localScaling = scaling;
	recalcLocalAabb();
}

void btlConvexHullShape::addPoint(const btlVector3& point, bool recalculateLocalAabb)
{
	m_unscaledPoints.push_back(point);
	if (recalculateLocalAabb)
		recalcLocalAabb();

}

btlVector3	btlConvexHullShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
{
	btlVector3 supVec(btlScalar(0.),btlScalar(0.),btlScalar(0.));
	btlScalar maxDot = btlScalar(-BT_LARGE_FLOAT);

    // Here we take advantage of dot(a, b*c) = dot(a*b, c).  Note: This is true mathematically, but not numerically. 
    if( 0 < m_unscaledPoints.size() )
    {
        btlVector3 scaled = vec * m_localScaling;
        int index = (int) scaled.maxDot( &m_unscaledPoints[0], m_unscaledPoints.size(), maxDot); // FIXME: may violate encapsulation of m_unscaledPoints
        return m_unscaledPoints[index] * m_localScaling;
    }

    return supVec;
}

void	btlConvexHullShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
	btlScalar newDot;
	//use 'w' component of supportVerticesOut?
	{
		for (int i=0;i<numVectors;i++)
		{
			supportVerticesOut[i][3] = btlScalar(-BT_LARGE_FLOAT);
		}
	}

    for (int j=0;j<numVectors;j++)
    {
        btlVector3 vec = vectors[j] * m_localScaling;        // dot(a*b,c) = dot(a,b*c)
        if( 0 <  m_unscaledPoints.size() )
        {
            int i = (int) vec.maxDot( &m_unscaledPoints[0], m_unscaledPoints.size(), newDot);
            supportVerticesOut[j] = getScaledPoint(i);
            supportVerticesOut[j][3] = newDot;        
        }
        else
            supportVerticesOut[j][3] = -BT_LARGE_FLOAT;
    }



}
	


btlVector3	btlConvexHullShape::localGetSupportingVertex(const btlVector3& vec)const
{
	btlVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

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









//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw btConvexHullShape with the Raytracer Demo
int	btlConvexHullShape::getNumVertices() const
{
	return m_unscaledPoints.size();
}

int btlConvexHullShape::getNumEdges() const
{
	return m_unscaledPoints.size();
}

void btlConvexHullShape::getEdge(int i,btlVector3& pa,btlVector3& pb) const
{

	int index0 = i%m_unscaledPoints.size();
	int index1 = (i+1)%m_unscaledPoints.size();
	pa = getScaledPoint(index0);
	pb = getScaledPoint(index1);
}

void btlConvexHullShape::getVertex(int i,btlVector3& vtx) const
{
	vtx = getScaledPoint(i);
}

int	btlConvexHullShape::getNumPlanes() const
{
	return 0;
}

void btlConvexHullShape::getPlane(btlVector3& ,btlVector3& ,int ) const
{

	btlAssert(0);
}

//not yet
bool btlConvexHullShape::isInside(const btlVector3& ,btlScalar ) const
{
	btlAssert(0);
	return false;
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlConvexHullShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	//int szc = sizeof(btConvexHullShapeData);
	btlConvexHullShapeData* shapeData = (btlConvexHullShapeData*) dataBuffer;
	btlConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	int numElem = m_unscaledPoints.size();
	shapeData->m_numUnscaledPoints = numElem;
#ifdef BT_USE_DOUBLE_PRECISION
	shapeData->m_unscaledPointsFloatPtr = 0;
	shapeData->m_unscaledPointsDoublePtr = numElem ? (btlVector3Data*)serializer->getUniquePointer((void*)&m_unscaledPoints[0]):  0;
#else
	shapeData->m_unscaledPointsFloatPtr = numElem ? (btlVector3Data*)serializer->getUniquePointer((void*)&m_unscaledPoints[0]):  0;
	shapeData->m_unscaledPointsDoublePtr = 0;
#endif
	
	if (numElem)
	{
		int sz = sizeof(btlVector3Data);
	//	int sz2 = sizeof(btVector3DoubleData);
	//	int sz3 = sizeof(btVector3FloatData);
		btlChunk* chunk = serializer->allocate(sz,numElem);
		btlVector3Data* memPtr = (btlVector3Data*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			m_unscaledPoints[i].serialize(*memPtr);
		}
		serializer->finalizeChunk(chunk,btlVector3DataName,BT_ARRAY_CODE,(void*)&m_unscaledPoints[0]);
	}
	
	return "btConvexHullShapeData";
}

void btlConvexHullShape::project(const btlTransform& trans, const btlVector3& dir, btlScalar& minProj, btlScalar& maxProj, btlVector3& witnesPtMin,btlVector3& witnesPtMax) const
{
#if 1
	minProj = FLT_MAX;
	maxProj = -FLT_MAX;

	int numVerts = m_unscaledPoints.size();
	for(int i=0;i<numVerts;i++)
	{
		btlVector3 vtx = m_unscaledPoints[i] * m_localScaling;
		btlVector3 pt = trans * vtx;
		btlScalar dp = pt.dot(dir);
		if(dp < minProj)	
		{
			minProj = dp;
			witnesPtMin = pt;
		}
		if(dp > maxProj)	
		{
			maxProj = dp;
			witnesPtMax=pt;
		}
	}
#else
	btlVector3 localAxis = dir*trans.getBasis();
	witnesPtMin  = trans(localGetSupportingVertex(localAxis));
	witnesPtMax = trans(localGetSupportingVertex(-localAxis));

	minProj = witnesPtMin.dot(dir);
	maxProj = witnesPtMax.dot(dir);
#endif

	if(minProj>maxProj)
	{
		btlSwap(minProj,maxProj);
		btlSwap(witnesPtMin,witnesPtMax);
	}


}


