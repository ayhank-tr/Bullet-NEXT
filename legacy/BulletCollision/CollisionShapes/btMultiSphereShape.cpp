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

#include "btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btSerializer.h"

btlMultiSphereShape::btlMultiSphereShape (const btlVector3* positions,const btlScalar* radi,int numSpheres)
:btlConvexInternalAabbCachingShape ()
{
	m_shapeType = MULTI_SPHERE_SHAPE_PROXYTYPE;
	//btScalar startMargin = btScalar(BT_LARGE_FLOAT);

	m_localPositionArray.resize(numSpheres);
	m_radiArray.resize(numSpheres);
	for (int i=0;i<numSpheres;i++)
	{
		m_localPositionArray[i] = positions[i];
		m_radiArray[i] = radi[i];
		
	}

	recalcLocalAabb();

}

#ifndef MIN
	#define MIN( _a, _b)    ((_a) < (_b) ? (_a) : (_b))
#endif
 btlVector3	btlMultiSphereShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec0)const
{
	btlVector3 supVec(0,0,0);

	btlScalar maxDot(btlScalar(-BT_LARGE_FLOAT));


	btlVector3 vec = vec0;
	btlScalar lenSqr = vec.length2();
	if (lenSqr < (SIMD_EPSILON*SIMD_EPSILON))
	{
		vec.setValue(1,0,0);
	} else
	{
		btlScalar rlen = btlScalar(1.) / btlSqrt(lenSqr );
		vec *= rlen;
	}

	btlVector3 vtx;
	btlScalar newDot;

	const btlVector3* pos = &m_localPositionArray[0];
	const btlScalar* rad = &m_radiArray[0];
	int numSpheres = m_localPositionArray.size();

	for( int k = 0; k < numSpheres; k+= 128 )
	{
		btlVector3 temp[128];
		int inner_count = MIN( numSpheres - k, 128 );
        for( long i = 0; i < inner_count; i++ )
        {
            temp[i] = (*pos) +vec*m_localScaling*(*rad) - vec * getMargin();
            pos++;
            rad++;
        }
        long i = vec.maxDot( temp, inner_count, newDot);
        if( newDot > maxDot )
		{
			maxDot = newDot;
			supVec = temp[i];
		}
    }

	return supVec;

}

 void	btlMultiSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{

	for (int j=0;j<numVectors;j++)
	{
		btlScalar maxDot(btlScalar(-BT_LARGE_FLOAT));

		const btlVector3& vec = vectors[j];

		btlVector3 vtx;
		btlScalar newDot;

		const btlVector3* pos = &m_localPositionArray[0];
		const btlScalar* rad = &m_radiArray[0];
		int numSpheres = m_localPositionArray.size();

        for( int k = 0; k < numSpheres; k+= 128 )
        {
            btlVector3 temp[128];
            int inner_count = MIN( numSpheres - k, 128 );
            for( long i = 0; i < inner_count; i++ )
            {
                temp[i] = (*pos) +vec*m_localScaling*(*rad) - vec * getMargin();
                pos++;
                rad++;
            }
            long i = vec.maxDot( temp, inner_count, newDot);
            if( newDot > maxDot )
            {
                maxDot = newDot;
                supportVerticesOut[j] = temp[i];
            }
        }
        
	}
}








void	btlMultiSphereShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
	//as an approximation, take the inertia of the box that bounds the spheres

	btlVector3 localAabbMin,localAabbMax;
	getCachedLocalAabb(localAabbMin,localAabbMax);
	btlVector3 halfExtents = (localAabbMax-localAabbMin)*btlScalar(0.5);

	btlScalar lx=btlScalar(2.)*(halfExtents.x());
	btlScalar ly=btlScalar(2.)*(halfExtents.y());
	btlScalar lz=btlScalar(2.)*(halfExtents.z());

	inertia.setValue(mass/(btlScalar(12.0)) * (ly*ly + lz*lz),
					mass/(btlScalar(12.0)) * (lx*lx + lz*lz),
					mass/(btlScalar(12.0)) * (lx*lx + ly*ly));

}


///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlMultiSphereShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlMultiSphereShapeData* shapeData = (btlMultiSphereShapeData*) dataBuffer;
	btlConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	int numElem = m_localPositionArray.size();
	shapeData->m_localPositionArrayPtr = numElem ? (btlPositionAndRadius*)serializer->getUniquePointer((void*)&m_localPositionArray[0]):  0;
	
	shapeData->m_localPositionArraySize = numElem;
	if (numElem)
	{
		btlChunk* chunk = serializer->allocate(sizeof(btlPositionAndRadius),numElem);
		btlPositionAndRadius* memPtr = (btlPositionAndRadius*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			m_localPositionArray[i].serializeFloat(memPtr->m_pos);
			memPtr->m_radius = float(m_radiArray[i]);
		}
		serializer->finalizeChunk(chunk,"btPositionAndRadius",BT_ARRAY_CODE,(void*)&m_localPositionArray[0]);
	}
	
	return "btMultiSphereShapeData";
}


