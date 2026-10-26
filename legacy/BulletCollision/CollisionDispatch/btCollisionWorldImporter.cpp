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

#include "btCollisionWorldImporter.h"
#include "btBulletCollisionCommon.h"
#include "LinearMath/btSerializer.h" //for btBulletSerializedArrays definition

#ifdef SUPPORT_GIMPACT_SHAPE_IMPORT
#include "BulletCollision/Gimpact/btGImpactShape.h"
#endif //SUPPORT_GIMPACT_SHAPE_IMPORT

btlCollisionWorldImporter::btlCollisionWorldImporter(btlCollisionWorld* world)
:m_collisionWorld(world),
m_verboseMode(0)
{

}

btlCollisionWorldImporter::~btlCollisionWorldImporter()
{
}





bool	btlCollisionWorldImporter::convertAllObjects( btlBulletSerializedArrays* arrays)
{

	m_shapeMap.clear();
	m_bodyMap.clear();

	int i;

	for (i=0;i<arrays->m_bvhsDouble.size();i++)
	{
		btlOptimizedBvh* bvh = createOptimizedBvh();
		btlQuantizedBvhDoubleData* bvhData = arrays->m_bvhsDouble[i];
		bvh->deSerializeDouble(*bvhData);
		m_bvhMap.insert(arrays->m_bvhsDouble[i],bvh);
	}
	for (i=0;i<arrays->m_bvhsFloat.size();i++)
    {
        btlOptimizedBvh* bvh = createOptimizedBvh();
   		btlQuantizedBvhFloatData* bvhData = arrays->m_bvhsFloat[i];
		bvh->deSerializeFloat(*bvhData);
		m_bvhMap.insert(arrays->m_bvhsFloat[i],bvh);
	}





	for (i=0;i<arrays->m_colShapeData.size();i++)
	{
		btlCollisionShapeData* shapeData = arrays->m_colShapeData[i];
		btlCollisionShape* shape = convertCollisionShape(shapeData);
		if (shape)
		{
	//		printf("shapeMap.insert(%x,%x)\n",shapeData,shape);
			m_shapeMap.insert(shapeData,shape);
		}

		if (shape&& shapeData->m_name)
		{
			char* newname = duplicateName(shapeData->m_name);
			m_objectNameMap.insert(shape,newname);
			m_nameShapeMap.insert(newname,shape);
		}
	}


	for (i=0;i<arrays->m_collisionObjectDataDouble.size();i++)
	{
        btlCollisionObjectDoubleData* colObjData = arrays->m_collisionObjectDataDouble[i];
        btlCollisionShape** shapePtr = m_shapeMap.find(colObjData->m_collisionShape);
        if (shapePtr && *shapePtr)
        {
            btlTransform startTransform;
            colObjData->m_worldTransform.m_origin.m_floats[3] = 0.f;
            startTransform.deSerializeDouble(colObjData->m_worldTransform);

            btlCollisionShape* shape = (btlCollisionShape*)*shapePtr;
            btlCollisionObject* body = createCollisionObject(startTransform,shape,colObjData->m_name);
            body->setFriction(btlScalar(colObjData->m_friction));
            body->setRestitution(btlScalar(colObjData->m_restitution));

#ifdef USE_INTERNAL_EDGE_UTILITY
            if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
            {
                btlBvhTriangleMeshShape* trimesh = (btlBvhTriangleMeshShape*)shape;
                if (trimesh->getTriangleInfoMap())
                {
                    body->setCollisionFlags(body->getCollisionFlags()  | btlCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
                }
            }
#endif //USE_INTERNAL_EDGE_UTILITY
            m_bodyMap.insert(colObjData,body);
        } else
        {
            printf("error: no shape found\n");
        }
	}
	for (i=0;i<arrays->m_collisionObjectDataFloat.size();i++)
	{
        btlCollisionObjectFloatData* colObjData = arrays->m_collisionObjectDataFloat[i];
        btlCollisionShape** shapePtr = m_shapeMap.find(colObjData->m_collisionShape);
        if (shapePtr && *shapePtr)
        {
            btlTransform startTransform;
            colObjData->m_worldTransform.m_origin.m_floats[3] = 0.f;
            startTransform.deSerializeFloat(colObjData->m_worldTransform);

            btlCollisionShape* shape = (btlCollisionShape*)*shapePtr;
            btlCollisionObject* body = createCollisionObject(startTransform,shape,colObjData->m_name);

#ifdef USE_INTERNAL_EDGE_UTILITY
            if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
            {
                btlBvhTriangleMeshShape* trimesh = (btlBvhTriangleMeshShape*)shape;
                if (trimesh->getTriangleInfoMap())
                {
                    body->setCollisionFlags(body->getCollisionFlags()  | btlCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
                }
            }
#endif //USE_INTERNAL_EDGE_UTILITY
            m_bodyMap.insert(colObjData,body);
        } else
        {
            printf("error: no shape found\n");
        }
    }

	return true;
}



void btlCollisionWorldImporter::deleteAllData()
{
	int i;

	for (i=0;i<m_allocatedCollisionObjects.size();i++)
	{
		if(m_collisionWorld)
			m_collisionWorld->removeCollisionObject(m_allocatedCollisionObjects[i]);
		delete m_allocatedCollisionObjects[i];
	}

	m_allocatedCollisionObjects.clear();


	for (i=0;i<m_allocatedCollisionShapes.size();i++)
	{
		delete m_allocatedCollisionShapes[i];
	}
	m_allocatedCollisionShapes.clear();


	for (i=0;i<m_allocatedBvhs.size();i++)
	{
		delete m_allocatedBvhs[i];
	}
	m_allocatedBvhs.clear();

	for (i=0;i<m_allocatedTriangleInfoMaps.size();i++)
	{
		delete m_allocatedTriangleInfoMaps[i];
	}
	m_allocatedTriangleInfoMaps.clear();
	for (i=0;i<m_allocatedTriangleIndexArrays.size();i++)
	{
		delete m_allocatedTriangleIndexArrays[i];
	}
	m_allocatedTriangleIndexArrays.clear();
	for (i=0;i<m_allocatedNames.size();i++)
	{
		delete[] m_allocatedNames[i];
	}
	m_allocatedNames.clear();

	for (i=0;i<m_allocatedbtStridingMeshInterfaceDatas.size();i++)
	{
		btlStridingMeshInterfaceData* curData = m_allocatedbtStridingMeshInterfaceDatas[i];

		for(int a = 0;a < curData->m_numMeshParts;a++)
		{
			btlMeshPartData* curPart = &curData->m_meshPartsPtr[a];
			if(curPart->m_vertices3f)
				delete [] curPart->m_vertices3f;

			if(curPart->m_vertices3d)
				delete [] curPart->m_vertices3d;

			if(curPart->m_indices32)
				delete [] curPart->m_indices32;

			if(curPart->m_3indices16)
				delete [] curPart->m_3indices16;

			if(curPart->m_indices16)
				delete [] curPart->m_indices16;

			if (curPart->m_3indices8)
				delete [] curPart->m_3indices8;

		}
		delete [] curData->m_meshPartsPtr;
		delete curData;
	}
	m_allocatedbtStridingMeshInterfaceDatas.clear();

	for (i=0;i<m_indexArrays.size();i++)
	{
		btlAlignedFree(m_indexArrays[i]);
	}
  m_indexArrays.clear();

	for (i=0;i<m_shortIndexArrays.size();i++)
	{
		btlAlignedFree(m_shortIndexArrays[i]);
	}
  m_shortIndexArrays.clear();

	for (i=0;i<m_charIndexArrays.size();i++)
	{
		btlAlignedFree(m_charIndexArrays[i]);
	}
  m_charIndexArrays.clear();

	for (i=0;i<m_floatVertexArrays.size();i++)
	{
		btlAlignedFree(m_floatVertexArrays[i]);
	}
  m_floatVertexArrays.clear();

	for (i=0;i<m_doubleVertexArrays.size();i++)
	{
		btlAlignedFree(m_doubleVertexArrays[i]);
	}
   m_doubleVertexArrays.clear();


}



btlCollisionShape* btlCollisionWorldImporter::convertCollisionShape(  btlCollisionShapeData* shapeData  )
{
	btlCollisionShape* shape = 0;

	switch (shapeData->m_shapeType)
		{
	case STATIC_PLANE_PROXYTYPE:
		{
			btlStaticPlaneShapeData* planeData = (btlStaticPlaneShapeData*)shapeData;
			btlVector3 planeNormal,localScaling;
			planeNormal.deSerializeFloat(planeData->m_planeNormal);
			localScaling.deSerializeFloat(planeData->m_localScaling);
			shape = createPlaneShape(planeNormal,planeData->m_planeConstant);
			shape->setLocalScaling(localScaling);

			break;
		}
	case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			btlScaledTriangleMeshShapeData* scaledMesh = (btlScaledTriangleMeshShapeData*) shapeData;
			btlCollisionShapeData* colShapeData = (btlCollisionShapeData*) &scaledMesh->m_trimeshShapeData;
			colShapeData->m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
			btlCollisionShape* childShape = convertCollisionShape(colShapeData);
			btlBvhTriangleMeshShape* meshShape = (btlBvhTriangleMeshShape*)childShape;
			btlVector3 localScaling;
			localScaling.deSerializeFloat(scaledMesh->m_localScaling);

			shape = createScaledTrangleMeshShape(meshShape, localScaling);
			break;
		}
#ifdef SUPPORT_GIMPACT_SHAPE_IMPORT
	case GIMPACT_SHAPE_PROXYTYPE:
		{
			btlGImpactMeshShapeData* gimpactData = (btlGImpactMeshShapeData*) shapeData;
			if (gimpactData->m_gimpactSubType == CONST_GIMPACT_TRIMESH_SHAPE)
			{
				btlStridingMeshInterfaceData* interfaceData = createStridingMeshInterfaceData(&gimpactData->m_meshInterface);
				btlTriangleIndexVertexArray* meshInterface = createMeshInterface(*interfaceData);


				btlGImpactMeshShape* gimpactShape = createGimpactShape(meshInterface);
				btlVector3 localScaling;
				localScaling.deSerializeFloat(gimpactData->m_localScaling);
				gimpactShape->setLocalScaling(localScaling);
				gimpactShape->setMargin(btlScalar(gimpactData->m_collisionMargin));
				gimpactShape->updateBound();
				shape = gimpactShape;
			} else
			{
				printf("unsupported gimpact sub type\n");
			}
			break;
		}
#endif //SUPPORT_GIMPACT_SHAPE_IMPORT
	//The btCapsuleShape* API has issue passing the margin/scaling/halfextents unmodified through the API
	//so deal with this
		case CAPSULE_SHAPE_PROXYTYPE:
		{
			btlCapsuleShapeData* capData = (btlCapsuleShapeData*)shapeData;


			switch (capData->m_upAxis)
			{
			case 0:
				{
					shape = createCapsuleShapeX(1,1);
					break;
				}
			case 1:
				{
					shape = createCapsuleShapeY(1,1);
					break;
				}
			case 2:
				{
					shape = createCapsuleShapeZ(1,1);
					break;
				}
			default:
				{
					printf("error: wrong up axis for btCapsuleShape\n");
				}


			};
			if (shape)
			{
				btlCapsuleShape* cap = (btlCapsuleShape*) shape;
				cap->deSerializeFloat(capData);
			}
			break;
		}
		case CYLINDER_SHAPE_PROXYTYPE:
		case CONE_SHAPE_PROXYTYPE:
		case BOX_SHAPE_PROXYTYPE:
		case SPHERE_SHAPE_PROXYTYPE:
		case MULTI_SPHERE_SHAPE_PROXYTYPE:
		case CONVEX_HULL_SHAPE_PROXYTYPE:
			{
				btlConvexInternalShapeData* bsd = (btlConvexInternalShapeData*)shapeData;
				btlVector3 implicitShapeDimensions;
				implicitShapeDimensions.deSerializeFloat(bsd->m_implicitShapeDimensions);
				btlVector3 localScaling;
				localScaling.deSerializeFloat(bsd->m_localScaling);
				btlVector3 margin(bsd->m_collisionMargin,bsd->m_collisionMargin,bsd->m_collisionMargin);
				switch (shapeData->m_shapeType)
				{
					case BOX_SHAPE_PROXYTYPE:
						{
							btlBoxShape* box= (btlBoxShape*)createBoxShape(implicitShapeDimensions/localScaling+margin);
							//box->initializePolyhedralFeatures();
							shape = box;

							break;
						}
					case SPHERE_SHAPE_PROXYTYPE:
						{
							shape = createSphereShape(implicitShapeDimensions.getX());
							break;
						}

					case CYLINDER_SHAPE_PROXYTYPE:
						{
							btlCylinderShapeData* cylData = (btlCylinderShapeData*) shapeData;
							btlVector3 halfExtents = implicitShapeDimensions+margin;
							switch (cylData->m_upAxis)
							{
							case 0:
								{
									shape = createCylinderShapeX(halfExtents.getY(),halfExtents.getX());
									break;
								}
							case 1:
								{
									shape = createCylinderShapeY(halfExtents.getX(),halfExtents.getY());
									break;
								}
							case 2:
								{
									shape = createCylinderShapeZ(halfExtents.getX(),halfExtents.getZ());
									break;
								}
							default:
								{
									printf("unknown Cylinder up axis\n");
								}

							};



							break;
						}
					case CONE_SHAPE_PROXYTYPE:
						{
							btlConeShapeData* conData = (btlConeShapeData*) shapeData;
							btlVector3 halfExtents = implicitShapeDimensions;//+margin;
							switch (conData->m_upIndex)
							{
							case 0:
								{
									shape = createConeShapeX(halfExtents.getY(),halfExtents.getX());
									break;
								}
							case 1:
								{
									shape = createConeShapeY(halfExtents.getX(),halfExtents.getY());
									break;
								}
							case 2:
								{
									shape = createConeShapeZ(halfExtents.getX(),halfExtents.getZ());
									break;
								}
							default:
								{
									printf("unknown Cone up axis\n");
								}

							};



							break;
						}
					case MULTI_SPHERE_SHAPE_PROXYTYPE:
						{
							btlMultiSphereShapeData* mss = (btlMultiSphereShapeData*)bsd;
							int numSpheres = mss->m_localPositionArraySize;

							btlAlignedObjectArray<btlVector3> tmpPos;
							btlAlignedObjectArray<btlScalar> radii;
							radii.resize(numSpheres);
							tmpPos.resize(numSpheres);
							int i;
							for ( i=0;i<numSpheres;i++)
							{
								tmpPos[i].deSerializeFloat(mss->m_localPositionArrayPtr[i].m_pos);
								radii[i] = mss->m_localPositionArrayPtr[i].m_radius;
							}
							shape = createMultiSphereShape(&tmpPos[0],&radii[0],numSpheres);
							break;
						}
					case CONVEX_HULL_SHAPE_PROXYTYPE:
						{
						//	int sz = sizeof(btConvexHullShapeData);
						//	int sz2 = sizeof(btConvexInternalShapeData);
						//	int sz3 = sizeof(btCollisionShapeData);
							btlConvexHullShapeData* convexData = (btlConvexHullShapeData*)bsd;
							int numPoints = convexData->m_numUnscaledPoints;

							btlAlignedObjectArray<btlVector3> tmpPoints;
							tmpPoints.resize(numPoints);
							int i;
							for ( i=0;i<numPoints;i++)
							{
#ifdef BT_USE_DOUBLE_PRECISION
							if (convexData->m_unscaledPointsDoublePtr)
								tmpPoints[i].deSerialize(convexData->m_unscaledPointsDoublePtr[i]);
							if (convexData->m_unscaledPointsFloatPtr)
								tmpPoints[i].deSerializeFloat(convexData->m_unscaledPointsFloatPtr[i]);
#else
							if (convexData->m_unscaledPointsFloatPtr)
								tmpPoints[i].deSerialize(convexData->m_unscaledPointsFloatPtr[i]);
							if (convexData->m_unscaledPointsDoublePtr)
								tmpPoints[i].deSerializeDouble(convexData->m_unscaledPointsDoublePtr[i]);
#endif //BT_USE_DOUBLE_PRECISION
							}
							btlConvexHullShape* hullShape = createConvexHullShape();
							for (i=0;i<numPoints;i++)
							{
								hullShape->addPoint(tmpPoints[i]);
							}
							hullShape->setMargin(bsd->m_collisionMargin);
							//hullShape->initializePolyhedralFeatures();
							shape = hullShape;
							break;
						}
					default:
						{
							printf("error: cannot create shape type (%d)\n",shapeData->m_shapeType);
						}
				}

				if (shape)
				{
					shape->setMargin(bsd->m_collisionMargin);

					btlVector3 localScaling;
					localScaling.deSerializeFloat(bsd->m_localScaling);
					shape->setLocalScaling(localScaling);

				}
				break;
			}
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			btlTriangleMeshShapeData* trimesh = (btlTriangleMeshShapeData*)shapeData;
			btlStridingMeshInterfaceData* interfaceData = createStridingMeshInterfaceData(&trimesh->m_meshInterface);
			btlTriangleIndexVertexArray* meshInterface = createMeshInterface(*interfaceData);
			if (!meshInterface->getNumSubParts())
			{
				return 0;
			}

			btlVector3 scaling; scaling.deSerializeFloat(trimesh->m_meshInterface.m_scaling);
			meshInterface->setScaling(scaling);


			btlOptimizedBvh* bvh = 0;
#if 1
			if (trimesh->m_quantizedFloatBvh)
			{
				btlOptimizedBvh** bvhPtr = m_bvhMap.find(trimesh->m_quantizedFloatBvh);
				if (bvhPtr && *bvhPtr)
				{
					bvh = *bvhPtr;
				} else
				{
					bvh = createOptimizedBvh();
					bvh->deSerializeFloat(*trimesh->m_quantizedFloatBvh);
				}
			}
			if (trimesh->m_quantizedDoubleBvh)
			{
				btlOptimizedBvh** bvhPtr = m_bvhMap.find(trimesh->m_quantizedDoubleBvh);
				if (bvhPtr && *bvhPtr)
				{
					bvh = *bvhPtr;
				} else
				{
					bvh = createOptimizedBvh();
					bvh->deSerializeDouble(*trimesh->m_quantizedDoubleBvh);
				}
			}
#endif


			btlBvhTriangleMeshShape* trimeshShape = createBvhTriangleMeshShape(meshInterface,bvh);
			trimeshShape->setMargin(trimesh->m_collisionMargin);
			shape = trimeshShape;

			if (trimesh->m_triangleInfoMap)
			{
				btlTriangleInfoMap* map = createTriangleInfoMap();
				map->deSerialize(*trimesh->m_triangleInfoMap);
				trimeshShape->setTriangleInfoMap(map);

#ifdef USE_INTERNAL_EDGE_UTILITY
				btl_gContactAddedCallback = btlAdjustInternalEdgeContactsCallback;
#endif //USE_INTERNAL_EDGE_UTILITY

			}

			//printf("trimesh->m_collisionMargin=%f\n",trimesh->m_collisionMargin);
			break;
		}
		case COMPOUND_SHAPE_PROXYTYPE:
			{
				btlCompoundShapeData* compoundData = (btlCompoundShapeData*)shapeData;
				btlCompoundShape* compoundShape = createCompoundShape();


				btlAlignedObjectArray<btlCollisionShape*> childShapes;
				for (int i=0;i<compoundData->m_numChildShapes;i++)
				{
					btlCollisionShapeData* cd = compoundData->m_childShapePtr[i].m_childShape;

					btlCollisionShape* childShape = convertCollisionShape(cd);
					if (childShape)
					{
						btlTransform localTransform;
						localTransform.deSerializeFloat(compoundData->m_childShapePtr[i].m_transform);
						compoundShape->addChildShape(localTransform,childShape);
					} else
					{
#ifdef _DEBUG
						printf("error: couldn't create childShape for compoundShape\n");
#endif
					}

				}
				shape = compoundShape;

				break;
			}
		case SOFTBODY_SHAPE_PROXYTYPE:
			{
				return 0;
			}
		default:
			{
#ifdef _DEBUG
				printf("unsupported shape type (%d)\n",shapeData->m_shapeType);
#endif
			}
		}

		return shape;

}



char* btlCollisionWorldImporter::duplicateName(const char* name)
{
	if (name)
	{
		int l = (int)strlen(name);
		char* newName = new char[l+1];
		memcpy(newName,name,l);
		newName[l] = 0;
		m_allocatedNames.push_back(newName);
		return newName;
	}
	return 0;
}











btlTriangleIndexVertexArray* btlCollisionWorldImporter::createMeshInterface(btlStridingMeshInterfaceData&  meshData)
{
	btlTriangleIndexVertexArray* meshInterface = createTriangleMeshContainer();

	for (int i=0;i<meshData.m_numMeshParts;i++)
	{
		btlIndexedMesh meshPart;
		meshPart.m_numTriangles = meshData.m_meshPartsPtr[i].m_numTriangles;
		meshPart.m_numVertices = meshData.m_meshPartsPtr[i].m_numVertices;


		if (meshData.m_meshPartsPtr[i].m_indices32)
		{
			meshPart.m_indexType = PHY_INTEGER;
			meshPart.m_triangleIndexStride = 3*sizeof(int);
			int* indexArray = (int*)btlAlignedAlloc(sizeof(int)*3*meshPart.m_numTriangles,16);
			m_indexArrays.push_back(indexArray);
			for (int j=0;j<3*meshPart.m_numTriangles;j++)
			{
				indexArray[j] = meshData.m_meshPartsPtr[i].m_indices32[j].m_value;
			}
			meshPart.m_triangleIndexBase = (const unsigned char*)indexArray;
		} else
		{
			if (meshData.m_meshPartsPtr[i].m_3indices16)
			{
				meshPart.m_indexType = PHY_SHORT;
				meshPart.m_triangleIndexStride = sizeof(short int)*3;//sizeof(btShortIntIndexTripletData);

				short int* indexArray = (short int*)btlAlignedAlloc(sizeof(short int)*3*meshPart.m_numTriangles,16);
				m_shortIndexArrays.push_back(indexArray);

				for (int j=0;j<meshPart.m_numTriangles;j++)
				{
					indexArray[3*j] = meshData.m_meshPartsPtr[i].m_3indices16[j].m_values[0];
					indexArray[3*j+1] = meshData.m_meshPartsPtr[i].m_3indices16[j].m_values[1];
					indexArray[3*j+2] = meshData.m_meshPartsPtr[i].m_3indices16[j].m_values[2];
				}

				meshPart.m_triangleIndexBase = (const unsigned char*)indexArray;
			}
			if (meshData.m_meshPartsPtr[i].m_indices16)
			{
				meshPart.m_indexType = PHY_SHORT;
				meshPart.m_triangleIndexStride = 3*sizeof(short int);
				short int* indexArray = (short int*)btlAlignedAlloc(sizeof(short int)*3*meshPart.m_numTriangles,16);
				m_shortIndexArrays.push_back(indexArray);
				for (int j=0;j<3*meshPart.m_numTriangles;j++)
				{
					indexArray[j] = meshData.m_meshPartsPtr[i].m_indices16[j].m_value;
				}

				meshPart.m_triangleIndexBase = (const unsigned char*)indexArray;
			}

			if (meshData.m_meshPartsPtr[i].m_3indices8)
			{
				meshPart.m_indexType = PHY_UCHAR;
				meshPart.m_triangleIndexStride = sizeof(unsigned char)*3;

				unsigned char* indexArray = (unsigned char*)btlAlignedAlloc(sizeof(unsigned char)*3*meshPart.m_numTriangles,16);
				m_charIndexArrays.push_back(indexArray);

				for (int j=0;j<meshPart.m_numTriangles;j++)
				{
					indexArray[3*j] = meshData.m_meshPartsPtr[i].m_3indices8[j].m_values[0];
					indexArray[3*j+1] = meshData.m_meshPartsPtr[i].m_3indices8[j].m_values[1];
					indexArray[3*j+2] = meshData.m_meshPartsPtr[i].m_3indices8[j].m_values[2];
				}

				meshPart.m_triangleIndexBase = (const unsigned char*)indexArray;
			}
		}

		if (meshData.m_meshPartsPtr[i].m_vertices3f)
		{
			meshPart.m_vertexType = PHY_FLOAT;
			meshPart.m_vertexStride = sizeof(btlVector3FloatData);
			btlVector3FloatData* vertices = (btlVector3FloatData*) btlAlignedAlloc(sizeof(btlVector3FloatData)*meshPart.m_numVertices,16);
			m_floatVertexArrays.push_back(vertices);

			for (int j=0;j<meshPart.m_numVertices;j++)
			{
				vertices[j].m_floats[0] = meshData.m_meshPartsPtr[i].m_vertices3f[j].m_floats[0];
				vertices[j].m_floats[1] = meshData.m_meshPartsPtr[i].m_vertices3f[j].m_floats[1];
				vertices[j].m_floats[2] = meshData.m_meshPartsPtr[i].m_vertices3f[j].m_floats[2];
				vertices[j].m_floats[3] = meshData.m_meshPartsPtr[i].m_vertices3f[j].m_floats[3];
			}
			meshPart.m_vertexBase = (const unsigned char*)vertices;
		} else
		{
			meshPart.m_vertexType = PHY_DOUBLE;
			meshPart.m_vertexStride = sizeof(btlVector3DoubleData);


			btlVector3DoubleData* vertices = (btlVector3DoubleData*) btlAlignedAlloc(sizeof(btlVector3DoubleData)*meshPart.m_numVertices,16);
			m_doubleVertexArrays.push_back(vertices);

			for (int j=0;j<meshPart.m_numVertices;j++)
			{
				vertices[j].m_floats[0] = meshData.m_meshPartsPtr[i].m_vertices3d[j].m_floats[0];
				vertices[j].m_floats[1] = meshData.m_meshPartsPtr[i].m_vertices3d[j].m_floats[1];
				vertices[j].m_floats[2] = meshData.m_meshPartsPtr[i].m_vertices3d[j].m_floats[2];
				vertices[j].m_floats[3] = meshData.m_meshPartsPtr[i].m_vertices3d[j].m_floats[3];
			}
			meshPart.m_vertexBase = (const unsigned char*)vertices;
		}

		if (meshPart.m_triangleIndexBase && meshPart.m_vertexBase)
		{
			meshInterface->addIndexedMesh(meshPart,meshPart.m_indexType);
		}
	}

	return meshInterface;
}


btlStridingMeshInterfaceData* btlCollisionWorldImporter::createStridingMeshInterfaceData(btlStridingMeshInterfaceData* interfaceData)
{
	//create a new btStridingMeshInterfaceData that is an exact copy of shapedata and store it in the WorldImporter
	btlStridingMeshInterfaceData* newData = new btlStridingMeshInterfaceData;

	newData->m_scaling = interfaceData->m_scaling;
	newData->m_numMeshParts = interfaceData->m_numMeshParts;
	newData->m_meshPartsPtr = new btlMeshPartData[newData->m_numMeshParts];

	for(int i = 0;i < newData->m_numMeshParts;i++)
	{
		btlMeshPartData* curPart = &interfaceData->m_meshPartsPtr[i];
		btlMeshPartData* curNewPart = &newData->m_meshPartsPtr[i];

		curNewPart->m_numTriangles = curPart->m_numTriangles;
		curNewPart->m_numVertices = curPart->m_numVertices;

		if(curPart->m_vertices3f)
		{
			curNewPart->m_vertices3f = new btlVector3FloatData[curNewPart->m_numVertices];
			memcpy(curNewPart->m_vertices3f,curPart->m_vertices3f,sizeof(btlVector3FloatData) * curNewPart->m_numVertices);
		}
		else
			curNewPart->m_vertices3f = NULL;

		if(curPart->m_vertices3d)
		{
			curNewPart->m_vertices3d = new btlVector3DoubleData[curNewPart->m_numVertices];
			memcpy(curNewPart->m_vertices3d,curPart->m_vertices3d,sizeof(btlVector3DoubleData) * curNewPart->m_numVertices);
		}
		else
			curNewPart->m_vertices3d = NULL;

		int numIndices = curNewPart->m_numTriangles * 3;
		///the m_3indices8 was not initialized in some Bullet versions, this can cause crashes at loading time
		///we catch it by only dealing with m_3indices8 if none of the other indices are initialized
		bool uninitialized3indices8Workaround =false;

		if(curPart->m_indices32)
		{
			uninitialized3indices8Workaround=true;
			curNewPart->m_indices32 = new btlIntIndexData[numIndices];
			memcpy(curNewPart->m_indices32,curPart->m_indices32,sizeof(btlIntIndexData) * numIndices);
		}
		else
			curNewPart->m_indices32 = NULL;

		if(curPart->m_3indices16)
		{
			uninitialized3indices8Workaround=true;
			curNewPart->m_3indices16 = new btlShortIntIndexTripletData[curNewPart->m_numTriangles];
			memcpy(curNewPart->m_3indices16,curPart->m_3indices16,sizeof(btlShortIntIndexTripletData) * curNewPart->m_numTriangles);
		}
		else
			curNewPart->m_3indices16 = NULL;

		if(curPart->m_indices16)
		{
			uninitialized3indices8Workaround=true;
			curNewPart->m_indices16 = new btlShortIntIndexData[numIndices];
			memcpy(curNewPart->m_indices16,curPart->m_indices16,sizeof(btlShortIntIndexData) * numIndices);
		}
		else
			curNewPart->m_indices16 = NULL;

		if(!uninitialized3indices8Workaround && curPart->m_3indices8)
		{
			curNewPart->m_3indices8 = new btlCharIndexTripletData[curNewPart->m_numTriangles];
			memcpy(curNewPart->m_3indices8,curPart->m_3indices8,sizeof(btlCharIndexTripletData) * curNewPart->m_numTriangles);
		}
		else
			curNewPart->m_3indices8 = NULL;

	}

	m_allocatedbtStridingMeshInterfaceDatas.push_back(newData);

	return(newData);
}

#ifdef USE_INTERNAL_EDGE_UTILITY
extern ContactAddedCallback		btl_gContactAddedCallback;

static bool btlAdjustInternalEdgeContactsCallback(btlManifoldPoint& cp,	const btlCollisionObject* colObj0,int partId0,int index0,const btlCollisionObject* colObj1,int partId1,int index1)
{

	btlAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_BACKFACE_MODE);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_DOUBLE_SIDED+BT_TRIANGLE_CONCAVE_DOUBLE_SIDED);
	return true;
}
#endif //USE_INTERNAL_EDGE_UTILITY


/*
btRigidBody*  btWorldImporter::createRigidBody(bool isDynamic, btScalar mass, const btTransform& startTransform,btCollisionShape* shape,const char* bodyName)
{
	btVector3 localInertia;
	localInertia.setZero();

	if (mass)
		shape->calculateLocalInertia(mass,localInertia);

	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);

	if (m_dynamicsWorld)
		m_dynamicsWorld->addRigidBody(body);

	if (bodyName)
	{
		char* newname = duplicateName(bodyName);
		m_objectNameMap.insert(body,newname);
		m_nameBodyMap.insert(newname,body);
	}
	m_allocatedRigidBodies.push_back(body);
	return body;

}
*/

btlCollisionObject* btlCollisionWorldImporter::getCollisionObjectByName(const char* name)
{
	btlCollisionObject** bodyPtr = m_nameColObjMap.find(name);
	if (bodyPtr && *bodyPtr)
	{
		return *bodyPtr;
	}
	return 0;
}

btlCollisionObject* btlCollisionWorldImporter::createCollisionObject(const btlTransform& startTransform,btlCollisionShape* shape, const char* bodyName)
{
	btlCollisionObject* colObj = new btlCollisionObject();
	colObj->setWorldTransform(startTransform);
	colObj->setCollisionShape(shape);
	m_collisionWorld->addCollisionObject(colObj);//todo: flags etc

	if (bodyName)
	{
		char* newname = duplicateName(bodyName);
		m_objectNameMap.insert(colObj,newname);
		m_nameColObjMap.insert(newname,colObj);
	}
	m_allocatedCollisionObjects.push_back(colObj);

	return colObj;
}



btlCollisionShape* btlCollisionWorldImporter::createPlaneShape(const btlVector3& planeNormal,btlScalar planeConstant)
{
	btlStaticPlaneShape* shape = new btlStaticPlaneShape(planeNormal,planeConstant);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}
btlCollisionShape* btlCollisionWorldImporter::createBoxShape(const btlVector3& halfExtents)
{
	btlBoxShape* shape = new btlBoxShape(halfExtents);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}
btlCollisionShape* btlCollisionWorldImporter::createSphereShape(btlScalar radius)
{
	btlSphereShape* shape = new btlSphereShape(radius);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}


btlCollisionShape* btlCollisionWorldImporter::createCapsuleShapeX(btlScalar radius, btlScalar height)
{
	btlCapsuleShapeX* shape = new btlCapsuleShapeX(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCollisionShape* btlCollisionWorldImporter::createCapsuleShapeY(btlScalar radius, btlScalar height)
{
	btlCapsuleShape* shape = new btlCapsuleShape(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCollisionShape* btlCollisionWorldImporter::createCapsuleShapeZ(btlScalar radius, btlScalar height)
{
	btlCapsuleShapeZ* shape = new btlCapsuleShapeZ(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCollisionShape* btlCollisionWorldImporter::createCylinderShapeX(btlScalar radius,btlScalar height)
{
	btlCylinderShapeX* shape = new btlCylinderShapeX(btlVector3(height,radius,radius));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCollisionShape* btlCollisionWorldImporter::createCylinderShapeY(btlScalar radius,btlScalar height)
{
	btlCylinderShape* shape = new btlCylinderShape(btlVector3(radius,height,radius));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCollisionShape* btlCollisionWorldImporter::createCylinderShapeZ(btlScalar radius,btlScalar height)
{
	btlCylinderShapeZ* shape = new btlCylinderShapeZ(btlVector3(radius,radius,height));
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCollisionShape* btlCollisionWorldImporter::createConeShapeX(btlScalar radius,btlScalar height)
{
	btlConeShapeX* shape = new btlConeShapeX(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCollisionShape* btlCollisionWorldImporter::createConeShapeY(btlScalar radius,btlScalar height)
{
	btlConeShape* shape = new btlConeShape(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCollisionShape* btlCollisionWorldImporter::createConeShapeZ(btlScalar radius,btlScalar height)
{
	btlConeShapeZ* shape = new btlConeShapeZ(radius,height);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlTriangleIndexVertexArray*	btlCollisionWorldImporter::createTriangleMeshContainer()
{
	btlTriangleIndexVertexArray* in = new btlTriangleIndexVertexArray();
	m_allocatedTriangleIndexArrays.push_back(in);
	return in;
}

btlOptimizedBvh*	btlCollisionWorldImporter::createOptimizedBvh()
{
	btlOptimizedBvh* bvh = new btlOptimizedBvh();
	m_allocatedBvhs.push_back(bvh);
	return bvh;
}


btlTriangleInfoMap* btlCollisionWorldImporter::createTriangleInfoMap()
{
	btlTriangleInfoMap* tim = new btlTriangleInfoMap();
	m_allocatedTriangleInfoMaps.push_back(tim);
	return tim;
}

btlBvhTriangleMeshShape* btlCollisionWorldImporter::createBvhTriangleMeshShape(btlStridingMeshInterface* trimesh, btlOptimizedBvh* bvh)
{
	if (bvh)
	{
		btlBvhTriangleMeshShape* bvhTriMesh = new btlBvhTriangleMeshShape(trimesh,bvh->isQuantized(), false);
		bvhTriMesh->setOptimizedBvh(bvh);
		m_allocatedCollisionShapes.push_back(bvhTriMesh);
		return bvhTriMesh;
	}

	btlBvhTriangleMeshShape* ts = new btlBvhTriangleMeshShape(trimesh,true);
	m_allocatedCollisionShapes.push_back(ts);
	return ts;

}
btlCollisionShape* btlCollisionWorldImporter::createConvexTriangleMeshShape(btlStridingMeshInterface* trimesh)
{
	return 0;
}
#ifdef SUPPORT_GIMPACT_SHAPE_IMPORT
btlGImpactMeshShape* btlCollisionWorldImporter::createGimpactShape(btlStridingMeshInterface* trimesh)
{
	btlGImpactMeshShape* shape = new btlGImpactMeshShape(trimesh);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;

}
#endif //SUPPORT_GIMPACT_SHAPE_IMPORT

btlConvexHullShape* btlCollisionWorldImporter::createConvexHullShape()
{
	btlConvexHullShape* shape = new btlConvexHullShape();
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlCompoundShape* btlCollisionWorldImporter::createCompoundShape()
{
	btlCompoundShape* shape = new btlCompoundShape();
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}


btlScaledBvhTriangleMeshShape* btlCollisionWorldImporter::createScaledTrangleMeshShape(btlBvhTriangleMeshShape* meshShape,const btlVector3& localScaling)
{
	btlScaledBvhTriangleMeshShape* shape = new btlScaledBvhTriangleMeshShape(meshShape,localScaling);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}

btlMultiSphereShape* btlCollisionWorldImporter::createMultiSphereShape(const btlVector3* positions,const btlScalar* radi,int numSpheres)
{
	btlMultiSphereShape* shape = new btlMultiSphereShape(positions, radi, numSpheres);
	m_allocatedCollisionShapes.push_back(shape);
	return shape;
}



	// query for data
int	btlCollisionWorldImporter::getNumCollisionShapes() const
{
	return m_allocatedCollisionShapes.size();
}

btlCollisionShape* btlCollisionWorldImporter::getCollisionShapeByIndex(int index)
{
	return m_allocatedCollisionShapes[index];
}

btlCollisionShape* btlCollisionWorldImporter::getCollisionShapeByName(const char* name)
{
	btlCollisionShape** shapePtr = m_nameShapeMap.find(name);
	if (shapePtr&& *shapePtr)
	{
		return *shapePtr;
	}
	return 0;
}


const char*	btlCollisionWorldImporter::getNameForPointer(const void* ptr) const
{
	const char*const * namePtr = m_objectNameMap.find(ptr);
	if (namePtr && *namePtr)
		return *namePtr;
	return 0;
}


int btlCollisionWorldImporter::getNumRigidBodies() const
{
	return m_allocatedRigidBodies.size();
}

btlCollisionObject* btlCollisionWorldImporter::getRigidBodyByIndex(int index) const
{
	return m_allocatedRigidBodies[index];
}


int btlCollisionWorldImporter::getNumBvhs() const
{
	return m_allocatedBvhs.size();
}
 btlOptimizedBvh* btlCollisionWorldImporter::getBvhByIndex(int index) const
{
	return m_allocatedBvhs[index];
}

int btlCollisionWorldImporter::getNumTriangleInfoMaps() const
{
	return m_allocatedTriangleInfoMaps.size();
}

btlTriangleInfoMap* btlCollisionWorldImporter::getTriangleInfoMapByIndex(int index) const
{
	return m_allocatedTriangleInfoMaps[index];
}


