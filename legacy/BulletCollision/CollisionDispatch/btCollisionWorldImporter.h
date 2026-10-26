/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_COLLISION_WORLD_IMPORTER_H
#define BT_COLLISION_WORLD_IMPORTER_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"

class btlCollisionShape;
class btlCollisionObject;
struct btlBulletSerializedArrays;


struct ConstraintInput;
class btlCollisionWorld;
struct btlCollisionShapeData;
class btlTriangleIndexVertexArray;
class btlStridingMeshInterface;
struct btlStridingMeshInterfaceData;
class btlGImpactMeshShape;
class btlOptimizedBvh;
struct btlTriangleInfoMap;
class btlBvhTriangleMeshShape;
class btlPoint2PointConstraint;
class btlHingeConstraint;
class btlConeTwistConstraint;
class btlGeneric6DofConstraint;
class btlGeneric6DofSpringConstraint;
class btlSliderConstraint;
class btlGearConstraint;
struct btlContactSolverInfo;




class btlCollisionWorldImporter
{
protected:
	btlCollisionWorld* m_collisionWorld;

	int m_verboseMode;

	btlAlignedObjectArray<btlCollisionShape*>  m_allocatedCollisionShapes;
	btlAlignedObjectArray<btlCollisionObject*> m_allocatedRigidBodies;

	btlAlignedObjectArray<btlOptimizedBvh*>	 m_allocatedBvhs;
	btlAlignedObjectArray<btlTriangleInfoMap*> m_allocatedTriangleInfoMaps;
	btlAlignedObjectArray<btlTriangleIndexVertexArray*> m_allocatedTriangleIndexArrays;
	btlAlignedObjectArray<btlStridingMeshInterfaceData*> m_allocatedbtStridingMeshInterfaceDatas;
	btlAlignedObjectArray<btlCollisionObject*> m_allocatedCollisionObjects;


	btlAlignedObjectArray<char*>				m_allocatedNames;

	btlAlignedObjectArray<int*>				m_indexArrays;
	btlAlignedObjectArray<short int*>		m_shortIndexArrays;
	btlAlignedObjectArray<unsigned char*>	m_charIndexArrays;

	btlAlignedObjectArray<btlVector3FloatData*>	m_floatVertexArrays;
	btlAlignedObjectArray<btlVector3DoubleData*>	m_doubleVertexArrays;


	btlHashMap<btlHashPtr,btlOptimizedBvh*>	m_bvhMap;
	btlHashMap<btlHashPtr,btlTriangleInfoMap*>	m_timMap;

	btlHashMap<btlHashString,btlCollisionShape*>	m_nameShapeMap;
	btlHashMap<btlHashString,btlCollisionObject*>	m_nameColObjMap;

	btlHashMap<btlHashPtr,const char*>	m_objectNameMap;

	btlHashMap<btlHashPtr,btlCollisionShape*>	m_shapeMap;
	btlHashMap<btlHashPtr,btlCollisionObject*>	m_bodyMap;


	//methods



	char*	duplicateName(const char* name);

	btlCollisionShape* convertCollisionShape(  btlCollisionShapeData* shapeData  );


public:

	btlCollisionWorldImporter(btlCollisionWorld* world);

	virtual ~btlCollisionWorldImporter();

    bool	convertAllObjects( btlBulletSerializedArrays* arrays);

		///delete all memory collision shapes, rigid bodies, constraints etc. allocated during the load.
	///make sure you don't use the dynamics world containing objects after you call this method
	virtual void deleteAllData();

	void	setVerboseMode(int verboseMode)
	{
		m_verboseMode = verboseMode;
	}

	int getVerboseMode() const
	{
		return m_verboseMode;
	}

		// query for data
	int	getNumCollisionShapes() const;
	btlCollisionShape* getCollisionShapeByIndex(int index);
	int getNumRigidBodies() const;
	btlCollisionObject* getRigidBodyByIndex(int index) const;
	int getNumConstraints() const;

	int getNumBvhs() const;
	btlOptimizedBvh*  getBvhByIndex(int index) const;
	int getNumTriangleInfoMaps() const;
	btlTriangleInfoMap* getTriangleInfoMapByIndex(int index) const;

	// queris involving named objects
	btlCollisionShape* getCollisionShapeByName(const char* name);
	btlCollisionObject* getCollisionObjectByName(const char* name);


	const char*	getNameForPointer(const void* ptr) const;

	///those virtuals are called by load and can be overridden by the user



	//bodies

	virtual btlCollisionObject*  createCollisionObject(	const btlTransform& startTransform,	btlCollisionShape* shape,const char* bodyName);

	///shapes

	virtual btlCollisionShape* createPlaneShape(const btlVector3& planeNormal,btlScalar planeConstant);
	virtual btlCollisionShape* createBoxShape(const btlVector3& halfExtents);
	virtual btlCollisionShape* createSphereShape(btlScalar radius);
	virtual btlCollisionShape* createCapsuleShapeX(btlScalar radius, btlScalar height);
	virtual btlCollisionShape* createCapsuleShapeY(btlScalar radius, btlScalar height);
	virtual btlCollisionShape* createCapsuleShapeZ(btlScalar radius, btlScalar height);

	virtual btlCollisionShape* createCylinderShapeX(btlScalar radius,btlScalar height);
	virtual btlCollisionShape* createCylinderShapeY(btlScalar radius,btlScalar height);
	virtual btlCollisionShape* createCylinderShapeZ(btlScalar radius,btlScalar height);
	virtual btlCollisionShape* createConeShapeX(btlScalar radius,btlScalar height);
	virtual btlCollisionShape* createConeShapeY(btlScalar radius,btlScalar height);
	virtual btlCollisionShape* createConeShapeZ(btlScalar radius,btlScalar height);
	virtual class btlTriangleIndexVertexArray*	createTriangleMeshContainer();
	virtual	btlBvhTriangleMeshShape* createBvhTriangleMeshShape(btlStridingMeshInterface* trimesh, btlOptimizedBvh* bvh);
	virtual btlCollisionShape* createConvexTriangleMeshShape(btlStridingMeshInterface* trimesh);
#ifdef SUPPORT_GIMPACT_SHAPE_IMPORT
	virtual btlGImpactMeshShape* createGimpactShape(btlStridingMeshInterface* trimesh);
#endif //SUPPORT_GIMPACT_SHAPE_IMPORT
	virtual btlStridingMeshInterfaceData* createStridingMeshInterfaceData(btlStridingMeshInterfaceData* interfaceData);

	virtual class btlConvexHullShape* createConvexHullShape();
	virtual class btlCompoundShape* createCompoundShape();
	virtual class btlScaledBvhTriangleMeshShape* createScaledTrangleMeshShape(btlBvhTriangleMeshShape* meshShape,const btlVector3& localScalingbtBvhTriangleMeshShape);

	virtual class btlMultiSphereShape* createMultiSphereShape(const btlVector3* positions,const btlScalar* radi,int numSpheres);

	virtual btlTriangleIndexVertexArray* createMeshInterface(btlStridingMeshInterfaceData& meshData);

	///acceleration and connectivity structures
	virtual btlOptimizedBvh*	createOptimizedBvh();
	virtual btlTriangleInfoMap* createTriangleInfoMap();




};


#endif //BT_WORLD_IMPORTER_H
