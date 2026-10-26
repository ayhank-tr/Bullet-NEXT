/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_GEOMETRY_UTIL_H
#define BT_GEOMETRY_UTIL_H

#include "btVector3.h"
#include "btAlignedObjectArray.h"

///The btGeometryUtil helper class provides a few methods to convert between plane equations and vertices.
class btlGeometryUtil
{
	public:
	
	
		static void	getPlaneEquationsFromVertices(btlAlignedObjectArray<btlVector3>& vertices, btlAlignedObjectArray<btlVector3>& planeEquationsOut );

		static void	getVerticesFromPlaneEquations(const btlAlignedObjectArray<btlVector3>& planeEquations , btlAlignedObjectArray<btlVector3>& verticesOut );
	
		static bool	isInside(const btlAlignedObjectArray<btlVector3>& vertices, const btlVector3& planeNormal, btlScalar	margin);
		
		static bool	isPointInsidePlanes(const btlAlignedObjectArray<btlVector3>& planeEquations, const btlVector3& point, btlScalar	margin);

		static bool	areVerticesBehindPlane(const btlVector3& planeNormal, const btlAlignedObjectArray<btlVector3>& vertices, btlScalar	margin);

};


#endif //BT_GEOMETRY_UTIL_H

