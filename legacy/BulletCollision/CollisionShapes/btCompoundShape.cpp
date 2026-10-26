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

#include "btCompoundShape.h"
#include "btCollisionShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "LinearMath/btSerializer.h"

btlCompoundShape::btlCompoundShape(bool enableDynamicAabbTree, const int initialChildCapacity)
: m_localAabbMin(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT)),
m_localAabbMax(btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT)),
m_dynamicAabbTree(0),
m_updateRevision(1),
m_collisionMargin(btlScalar(0.)),
m_localScaling(btlScalar(1.),btlScalar(1.),btlScalar(1.))
{
	m_shapeType = COMPOUND_SHAPE_PROXYTYPE;

	if (enableDynamicAabbTree)
	{
		void* mem = btlAlignedAlloc(sizeof(btlDbvt),16);
		m_dynamicAabbTree = new(mem) btlDbvt();
		btlAssert(mem==m_dynamicAabbTree);
	}

	m_children.reserve(initialChildCapacity);
}


btlCompoundShape::~btlCompoundShape()
{
	if (m_dynamicAabbTree)
	{
		m_dynamicAabbTree->~btlDbvt();
		btlAlignedFree(m_dynamicAabbTree);
	}
}

void	btlCompoundShape::addChildShape(const btlTransform& localTransform,btlCollisionShape* shape)
{
	m_updateRevision++;
	//m_childTransforms.push_back(localTransform);
	//m_childShapes.push_back(shape);
	btlCompoundShapeChild child;
	child.m_node = 0;
	child.m_transform = localTransform;
	child.m_childShape = shape;
	child.m_childShapeType = shape->getShapeType();
	child.m_childMargin = shape->getMargin();

	
	//extend the local aabbMin/aabbMax
	btlVector3 localAabbMin,localAabbMax;
	shape->getAabb(localTransform,localAabbMin,localAabbMax);
	for (int i=0;i<3;i++)
	{
		if (m_localAabbMin[i] > localAabbMin[i])
		{
			m_localAabbMin[i] = localAabbMin[i];
		}
		if (m_localAabbMax[i] < localAabbMax[i])
		{
			m_localAabbMax[i] = localAabbMax[i];
		}

	}
	if (m_dynamicAabbTree)
	{
		const btlDbvtVolume	bounds=btlDbvtVolume::FromMM(localAabbMin,localAabbMax);
		size_t index = m_children.size();
		child.m_node = m_dynamicAabbTree->insert(bounds,reinterpret_cast<void*>(index) );
	}

	m_children.push_back(child);

}

void	btlCompoundShape::updateChildTransform(int childIndex, const btlTransform& newChildTransform,bool shouldRecalculateLocalAabb)
{
	m_children[childIndex].m_transform = newChildTransform;

	if (m_dynamicAabbTree)
	{
		///update the dynamic aabb tree
		btlVector3 localAabbMin,localAabbMax;
		m_children[childIndex].m_childShape->getAabb(newChildTransform,localAabbMin,localAabbMax);
		ATTRIBUTE_ALIGNED16(btlDbvtVolume)	bounds=btlDbvtVolume::FromMM(localAabbMin,localAabbMax);
		//int index = m_children.size()-1;
		m_dynamicAabbTree->update(m_children[childIndex].m_node,bounds);
	}

	if (shouldRecalculateLocalAabb)
	{
		recalculateLocalAabb();
	}
}

void btlCompoundShape::removeChildShapeByIndex(int childShapeIndex)
{
	m_updateRevision++;
	btlAssert(childShapeIndex >=0 && childShapeIndex < m_children.size());
	if (m_dynamicAabbTree)
	{
		m_dynamicAabbTree->remove(m_children[childShapeIndex].m_node);
	}
	m_children.swap(childShapeIndex,m_children.size()-1);
    if (m_dynamicAabbTree) 
		m_children[childShapeIndex].m_node->dataAsInt = childShapeIndex;
	m_children.pop_back();

}



void btlCompoundShape::removeChildShape(btlCollisionShape* shape)
{
	m_updateRevision++;
	// Find the children containing the shape specified, and remove those children.
	//note: there might be multiple children using the same shape!
	for(int i = m_children.size()-1; i >= 0 ; i--)
	{
		if(m_children[i].m_childShape == shape)
		{
			removeChildShapeByIndex(i);
		}
	}



	recalculateLocalAabb();
}

void btlCompoundShape::recalculateLocalAabb()
{
	// Recalculate the local aabb
	// Brute force, it iterates over all the shapes left.

	m_localAabbMin = btlVector3(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));
	m_localAabbMax = btlVector3(btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT));

	//extend the local aabbMin/aabbMax
	for (int j = 0; j < m_children.size(); j++)
	{
		btlVector3 localAabbMin,localAabbMax;
		m_children[j].m_childShape->getAabb(m_children[j].m_transform, localAabbMin, localAabbMax);
		for (int i=0;i<3;i++)
		{
			if (m_localAabbMin[i] > localAabbMin[i])
				m_localAabbMin[i] = localAabbMin[i];
			if (m_localAabbMax[i] < localAabbMax[i])
				m_localAabbMax[i] = localAabbMax[i];
		}
	}
}

///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
void btlCompoundShape::getAabb(const btlTransform& trans,btlVector3& aabbMin,btlVector3& aabbMax) const
{
	btlVector3 localHalfExtents = btlScalar(0.5)*(m_localAabbMax-m_localAabbMin);
	btlVector3 localCenter = btlScalar(0.5)*(m_localAabbMax+m_localAabbMin);
	
	//avoid an illegal AABB when there are no children
	if (!m_children.size())
	{
		localHalfExtents.setValue(0,0,0);
		localCenter.setValue(0,0,0);
	}
	localHalfExtents += btlVector3(getMargin(),getMargin(),getMargin());
		

	btlMatrix3x3 abs_b = trans.getBasis().absolute();  

	btlVector3 center = trans(localCenter);

    btlVector3 extent = localHalfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
	aabbMin = center-extent;
	aabbMax = center+extent;
	
}

void	btlCompoundShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
	//approximation: take the inertia from the aabb for now
	btlTransform ident;
	ident.setIdentity();
	btlVector3 aabbMin,aabbMax;
	getAabb(ident,aabbMin,aabbMax);

	btlVector3 halfExtents = (aabbMax-aabbMin)*btlScalar(0.5);

	btlScalar lx=btlScalar(2.)*(halfExtents.x());
	btlScalar ly=btlScalar(2.)*(halfExtents.y());
	btlScalar lz=btlScalar(2.)*(halfExtents.z());

	inertia[0] = mass/(btlScalar(12.0)) * (ly*ly + lz*lz);
	inertia[1] = mass/(btlScalar(12.0)) * (lx*lx + lz*lz);
	inertia[2] = mass/(btlScalar(12.0)) * (lx*lx + ly*ly);

}




void btlCompoundShape::calculatePrincipalAxisTransform(btlScalar* masses, btlTransform& principal, btlVector3& inertia) const
{
	int n = m_children.size();

	btlScalar totalMass = 0;
	btlVector3 center(0, 0, 0);
	int k;

	for (k = 0; k < n; k++)
	{
		btlAssert(masses[k]>0);
		center += m_children[k].m_transform.getOrigin() * masses[k];
		totalMass += masses[k];
	}

	btlAssert(totalMass>0);

	center /= totalMass;
	principal.setOrigin(center);

	btlMatrix3x3 tensor(0, 0, 0, 0, 0, 0, 0, 0, 0);
	for ( k = 0; k < n; k++)
	{
		btlVector3 i;
		m_children[k].m_childShape->calculateLocalInertia(masses[k], i);

		const btlTransform& t = m_children[k].m_transform;
		btlVector3 o = t.getOrigin() - center;

		//compute inertia tensor in coordinate system of compound shape
		btlMatrix3x3 j = t.getBasis().transpose();
		j[0] *= i[0];
		j[1] *= i[1];
		j[2] *= i[2];
		j = t.getBasis() * j;

		//add inertia tensor
		tensor[0] += j[0];
		tensor[1] += j[1];
		tensor[2] += j[2];

		//compute inertia tensor of pointmass at o
		btlScalar o2 = o.length2();
		j[0].setValue(o2, 0, 0);
		j[1].setValue(0, o2, 0);
		j[2].setValue(0, 0, o2);
		j[0] += o * -o.x(); 
		j[1] += o * -o.y(); 
		j[2] += o * -o.z();

		//add inertia tensor of pointmass
		tensor[0] += masses[k] * j[0];
		tensor[1] += masses[k] * j[1];
		tensor[2] += masses[k] * j[2];
	}

	tensor.diagonalize(principal.getBasis(), btlScalar(0.00001), 20);
	inertia.setValue(tensor[0][0], tensor[1][1], tensor[2][2]);
}





void btlCompoundShape::setLocalScaling(const btlVector3& scaling)
{

	for(int i = 0; i < m_children.size(); i++)
	{
		btlTransform childTrans = getChildTransform(i);
		btlVector3 childScale = m_children[i].m_childShape->getLocalScaling();
//		childScale = childScale * (childTrans.getBasis() * scaling);
		childScale = childScale * scaling / m_localScaling;
		m_children[i].m_childShape->setLocalScaling(childScale);
		childTrans.setOrigin((childTrans.getOrigin()) * scaling / m_localScaling);
		updateChildTransform(i, childTrans,false);
	}
	
	m_localScaling = scaling;
	recalculateLocalAabb();

}


void btlCompoundShape::createAabbTreeFromChildren()
{
    if ( !m_dynamicAabbTree )
    {
        void* mem = btlAlignedAlloc(sizeof(btlDbvt),16);
        m_dynamicAabbTree = new(mem) btlDbvt();
        btlAssert(mem==m_dynamicAabbTree);

        for ( int index = 0; index < m_children.size(); index++ )
        {
            btlCompoundShapeChild &child = m_children[index];

            //extend the local aabbMin/aabbMax
            btlVector3 localAabbMin,localAabbMax;
            child.m_childShape->getAabb(child.m_transform,localAabbMin,localAabbMax);

            const btlDbvtVolume  bounds=btlDbvtVolume::FromMM(localAabbMin,localAabbMax);
			size_t index2 = index;
            child.m_node = m_dynamicAabbTree->insert(bounds, reinterpret_cast<void*>(index2) );
        }
    }
}


///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlCompoundShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{

	btlCompoundShapeData* shapeData = (btlCompoundShapeData*) dataBuffer;
	btlCollisionShape::serialize(&shapeData->m_collisionShapeData, serializer);

	shapeData->m_collisionMargin = float(m_collisionMargin);
	shapeData->m_numChildShapes = m_children.size();
	shapeData->m_childShapePtr = 0;
	if (shapeData->m_numChildShapes)
	{
		btlChunk* chunk = serializer->allocate(sizeof(btlCompoundShapeChildData),shapeData->m_numChildShapes);
		btlCompoundShapeChildData* memPtr = (btlCompoundShapeChildData*)chunk->m_oldPtr;
		shapeData->m_childShapePtr = (btlCompoundShapeChildData*)serializer->getUniquePointer(memPtr);

		for (int i=0;i<shapeData->m_numChildShapes;i++,memPtr++)
		{
			memPtr->m_childMargin = float(m_children[i].m_childMargin);
			memPtr->m_childShape = (btlCollisionShapeData*)serializer->getUniquePointer(m_children[i].m_childShape);
			//don't serialize shapes that already have been serialized
			if (!serializer->findPointer(m_children[i].m_childShape))
			{
				btlChunk* chunk = serializer->allocate(m_children[i].m_childShape->calculateSerializeBufferSize(),1);
				const char* structType = m_children[i].m_childShape->serialize(chunk->m_oldPtr,serializer);
				serializer->finalizeChunk(chunk,structType,BT_SHAPE_CODE,m_children[i].m_childShape);
			} 

			memPtr->m_childShapeType = m_children[i].m_childShapeType;
			m_children[i].m_transform.serializeFloat(memPtr->m_transform);
		}
		serializer->finalizeChunk(chunk,"btCompoundShapeChildData",BT_ARRAY_CODE,chunk->m_oldPtr);
	}
	return "btCompoundShapeData";
}

