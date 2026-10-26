#ifndef BT_GIMPACT_QUANTIZATION_H_INCLUDED
#define BT_GIMPACT_QUANTIZATION_H_INCLUDED

/*! \file btQuantization.h
*\author Francisco Leon Najera

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

#include "LinearMath/btTransform.h"






SIMD_FORCE_INLINE void btl_calc_quantization_parameters(
	btlVector3 & outMinBound,
	btlVector3 & outMaxBound,
	btlVector3 & bvhQuantization,
	const btlVector3& srcMinBound,const btlVector3& srcMaxBound,
	btlScalar quantizationMargin)
{
	//enlarge the AABB to avoid division by zero when initializing the quantization values
	btlVector3 clampValue(quantizationMargin,quantizationMargin,quantizationMargin);
	outMinBound = srcMinBound - clampValue;
	outMaxBound = srcMaxBound + clampValue;
	btlVector3 aabbSize = outMaxBound - outMinBound;
	bvhQuantization = btlVector3(btlScalar(65535.0),
								btlScalar(65535.0),
								btlScalar(65535.0)) / aabbSize;
}


SIMD_FORCE_INLINE void btl_quantize_clamp(
	unsigned short* out,
	const btlVector3& point,
	const btlVector3 & min_bound,
	const btlVector3 & max_bound,
	const btlVector3 & bvhQuantization)
{

	btlVector3 clampedPoint(point);
	clampedPoint.setMax(min_bound);
	clampedPoint.setMin(max_bound);

	btlVector3 v = (clampedPoint - min_bound) * bvhQuantization;
	out[0] = (unsigned short)(v.getX()+0.5f);
	out[1] = (unsigned short)(v.getY()+0.5f);
	out[2] = (unsigned short)(v.getZ()+0.5f);
}


SIMD_FORCE_INLINE btlVector3 btl_unquantize(
	const unsigned short* vecIn,
	const btlVector3 & offset,
	const btlVector3 & bvhQuantization)
{
	btlVector3	vecOut;
	vecOut.setValue(
		(btlScalar)(vecIn[0]) / (bvhQuantization.getX()),
		(btlScalar)(vecIn[1]) / (bvhQuantization.getY()),
		(btlScalar)(vecIn[2]) / (bvhQuantization.getZ()));
	vecOut += offset;
	return vecOut;
}



#endif // BT_GIMPACT_QUANTIZATION_H_INCLUDED
