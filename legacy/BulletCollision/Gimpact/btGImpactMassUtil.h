/*! \file btGImpactMassUtil.h
\author Francisco Leon Najera
*/
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


#ifndef GIMPACT_MASS_UTIL_H
#define GIMPACT_MASS_UTIL_H

#include "LinearMath/btTransform.h"



SIMD_FORCE_INLINE btlVector3 btl_gim_inertia_add_transformed(
	const btlVector3 & source_inertia, const btlVector3 & added_inertia, const btlTransform & transform)
{
	btlMatrix3x3  rotatedTensor = transform.getBasis().scaled(added_inertia) * transform.getBasis().transpose();

	btlScalar x2 = transform.getOrigin()[0];
	x2*= x2;
	btlScalar y2 = transform.getOrigin()[1];
	y2*= y2;
	btlScalar z2 = transform.getOrigin()[2];
	z2*= z2;

	btlScalar ix = rotatedTensor[0][0]*(y2+z2);
	btlScalar iy = rotatedTensor[1][1]*(x2+z2);
	btlScalar iz = rotatedTensor[2][2]*(x2+y2);

	return btlVector3(source_inertia[0]+ix,source_inertia[1]+iy,source_inertia[2] + iz);
}

SIMD_FORCE_INLINE btlVector3 btl_gim_get_point_inertia(const btlVector3 & point, btlScalar mass)
{
	btlScalar x2 = point[0]*point[0];
	btlScalar y2 = point[1]*point[1];
	btlScalar z2 = point[2]*point[2];
	return btlVector3(mass*(y2+z2),mass*(x2+z2),mass*(x2+y2));
}


#endif //GIMPACT_MESH_SHAPE_H
