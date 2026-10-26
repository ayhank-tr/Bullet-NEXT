/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btGImpactShape.h"
#include "btGImpactMassUtil.h"


#define CALC_EXACT_INERTIA 1


void btlGImpactCompoundShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
	lockChildShapes();
#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	int i = this->getNumChildShapes();
	btlScalar shapemass = mass/btlScalar(i);

	while(i--)
	{
		btlVector3 temp_inertia;
		m_childShapes[i]->calculateLocalInertia(shapemass,temp_inertia);
		if(childrenHasTransform())
		{
			inertia = btl_gim_inertia_add_transformed( inertia,temp_inertia,m_childTransforms[i]);
		}
		else
		{
			inertia = btl_gim_inertia_add_transformed( inertia,temp_inertia,btlTransform::getIdentity());
		}

	}

#else

	// Calc box inertia

	btlScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	btlScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	btlScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const btlScalar x2 = lx*lx;
	const btlScalar y2 = ly*ly;
	const btlScalar z2 = lz*lz;
	const btlScalar scaledmass = mass * btlScalar(0.08333333);

	inertia = scaledmass * (btlVector3(y2+z2,x2+z2,x2+y2));

#endif
	unlockChildShapes();
}



void btlGImpactMeshShapePart::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
	lockChildShapes();


#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	int i = this->getVertexCount();
	btlScalar pointmass = mass/btlScalar(i);

	while(i--)
	{
		btlVector3 pointintertia;
		this->getVertex(i,pointintertia);
		pointintertia = btl_gim_get_point_inertia(pointintertia,pointmass);
		inertia+=pointintertia;
	}

#else

	// Calc box inertia

	btlScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	btlScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	btlScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const btlScalar x2 = lx*lx;
	const btlScalar y2 = ly*ly;
	const btlScalar z2 = lz*lz;
	const btlScalar scaledmass = mass * btlScalar(0.08333333);

	inertia = scaledmass * (btlVector3(y2+z2,x2+z2,x2+y2));

#endif

	unlockChildShapes();
}

void btlGImpactMeshShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{

#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	int i = this->getMeshPartCount();
	btlScalar partmass = mass/btlScalar(i);

	while(i--)
	{
		btlVector3 partinertia;
		getMeshPart(i)->calculateLocalInertia(partmass,partinertia);
		inertia+=partinertia;
	}

#else

	// Calc box inertia

	btlScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	btlScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	btlScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const btlScalar x2 = lx*lx;
	const btlScalar y2 = ly*ly;
	const btlScalar z2 = lz*lz;
	const btlScalar scaledmass = mass * btlScalar(0.08333333);

	inertia = scaledmass * (btlVector3(y2+z2,x2+z2,x2+y2));

#endif
}

void btlGImpactMeshShape::rayTest(const btlVector3& rayFrom, const btlVector3& rayTo, btlCollisionWorld::RayResultCallback& resultCallback) const
{
}

void btlGImpactMeshShapePart::processAllTrianglesRay(btlTriangleCallback* callback,const btlVector3& rayFrom, const btlVector3& rayTo) const
{
	lockChildShapes();

	btlAlignedObjectArray<int> collided;
	btlVector3 rayDir(rayTo - rayFrom);
	rayDir.normalize();
	m_box_set.rayQuery(rayDir, rayFrom, collided);

	if(collided.size()==0)
	{
		unlockChildShapes();
		return;
	}

	int part = (int)getPart();
	btlPrimitiveTriangle triangle;
	int i = collided.size();
	while(i--)
	{
		getPrimitiveTriangle(collided[i],triangle);
		callback->processTriangle(triangle.m_vertices,part,collided[i]);
	}
	unlockChildShapes();
}

void btlGImpactMeshShapePart::processAllTriangles(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax) const
{
	lockChildShapes();
	btlAABB box;
	box.m_min = aabbMin;
	box.m_max = aabbMax;

	btlAlignedObjectArray<int> collided;
	m_box_set.boxQuery(box,collided);

	if(collided.size()==0)
	{
		unlockChildShapes();
		return;
	}

	int part = (int)getPart();
	btlPrimitiveTriangle triangle;
	int i = collided.size();
	while(i--)
	{
		this->getPrimitiveTriangle(collided[i],triangle);
		callback->processTriangle(triangle.m_vertices,part,collided[i]);
	}
	unlockChildShapes();

}

void btlGImpactMeshShape::processAllTriangles(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax) const
{
	int i = m_mesh_parts.size();
	while(i--)
	{
		m_mesh_parts[i]->processAllTriangles(callback,aabbMin,aabbMax);
	}
}

void btlGImpactMeshShape::processAllTrianglesRay(btlTriangleCallback* callback,const btlVector3& rayFrom, const btlVector3& rayTo) const
{
	int i = m_mesh_parts.size();
	while(i--)
	{
		m_mesh_parts[i]->processAllTrianglesRay(callback, rayFrom, rayTo);
	}
}


///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlGImpactMeshShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlGImpactMeshShapeData* trimeshData = (btlGImpactMeshShapeData*) dataBuffer;

	btlCollisionShape::serialize(&trimeshData->m_collisionShapeData,serializer);

	m_meshInterface->serialize(&trimeshData->m_meshInterface, serializer);

	trimeshData->m_collisionMargin = float(m_collisionMargin);

	localScaling.serializeFloat(trimeshData->m_localScaling);

	trimeshData->m_gimpactSubType = int(getGImpactShapeType());

	return "btGImpactMeshShapeData";
}

