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
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btSerializer.h"

/*
  Make sure this dummy function never changes so that it
  can be used by probes that are checking whether the
  library is actually installed.
*/
extern "C" 
{
void btlBulletCollisionProbe ();

void btlBulletCollisionProbe () {}
}



void	btlCollisionShape::getBoundingSphere(btlVector3& center,btlScalar& radius) const
{
	btlTransform tr;
	tr.setIdentity();
	btlVector3 aabbMin,aabbMax;

	getAabb(tr,aabbMin,aabbMax);

	radius = (aabbMax-aabbMin).length()*btlScalar(0.5);
	center = (aabbMin+aabbMax)*btlScalar(0.5);
}


btlScalar	btlCollisionShape::getContactBreakingThreshold(btlScalar defaultContactThreshold) const
{
	return getAngularMotionDisc() * defaultContactThreshold;
}

btlScalar	btlCollisionShape::getAngularMotionDisc() const
{
	///@todo cache this value, to improve performance
	btlVector3	center;
	btlScalar disc;
	getBoundingSphere(center,disc);
	disc += (center).length();
	return disc;
}

void btlCollisionShape::calculateTemporalAabb(const btlTransform& curTrans,const btlVector3& linvel,const btlVector3& angvel,btlScalar timeStep, btlVector3& temporalAabbMin,btlVector3& temporalAabbMax) const
{
	//start with static aabb
	getAabb(curTrans,temporalAabbMin,temporalAabbMax);

	btlScalar temporalAabbMaxx = temporalAabbMax.getX();
	btlScalar temporalAabbMaxy = temporalAabbMax.getY();
	btlScalar temporalAabbMaxz = temporalAabbMax.getZ();
	btlScalar temporalAabbMinx = temporalAabbMin.getX();
	btlScalar temporalAabbMiny = temporalAabbMin.getY();
	btlScalar temporalAabbMinz = temporalAabbMin.getZ();

	// add linear motion
	btlVector3 linMotion = linvel*timeStep;
	///@todo: simd would have a vector max/min operation, instead of per-element access
	if (linMotion.x() > btlScalar(0.))
		temporalAabbMaxx += linMotion.x(); 
	else
		temporalAabbMinx += linMotion.x();
	if (linMotion.y() > btlScalar(0.))
		temporalAabbMaxy += linMotion.y(); 
	else
		temporalAabbMiny += linMotion.y();
	if (linMotion.z() > btlScalar(0.))
		temporalAabbMaxz += linMotion.z(); 
	else
		temporalAabbMinz += linMotion.z();

	//add conservative angular motion
	btlScalar angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
	btlVector3 angularMotion3d(angularMotion,angularMotion,angularMotion);
	temporalAabbMin = btlVector3(temporalAabbMinx,temporalAabbMiny,temporalAabbMinz);
	temporalAabbMax = btlVector3(temporalAabbMaxx,temporalAabbMaxy,temporalAabbMaxz);

	temporalAabbMin -= angularMotion3d;
	temporalAabbMax += angularMotion3d;
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlCollisionShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlCollisionShapeData* shapeData = (btlCollisionShapeData*) dataBuffer;
	char* name = (char*) serializer->findNameForPointer(this);
	shapeData->m_name = (char*)serializer->getUniquePointer(name);
	if (shapeData->m_name)
	{
		serializer->serializeName(name);
	}
	shapeData->m_shapeType = m_shapeType;
	//shapeData->m_padding//??
	return "btCollisionShapeData";
}

void	btlCollisionShape::serializeSingleShape(btlSerializer* serializer) const
{
	int len = calculateSerializeBufferSize();
	btlChunk* chunk = serializer->allocate(len,1);
	const char* structType = serialize(chunk->m_oldPtr, serializer);
	serializer->finalizeChunk(chunk,structType,BT_SHAPE_CODE,(void*)this);
}