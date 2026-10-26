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



#include "btGeometryUtil.h"


/*
  Make sure this dummy function never changes so that it
  can be used by probes that are checking whether the
  library is actually installed.
*/
extern "C"
{	
	void btlBulletMathProbe ();

	void btlBulletMathProbe () {}
}


bool	btlGeometryUtil::isPointInsidePlanes(const btlAlignedObjectArray<btlVector3>& planeEquations, const btlVector3& point, btlScalar	margin)
{
	int numbrushes = planeEquations.size();
	for (int i=0;i<numbrushes;i++)
	{
		const btlVector3& N1 = planeEquations[i];
		btlScalar dist = btlScalar(N1.dot(point))+btlScalar(N1[3])-margin;
		if (dist>btlScalar(0.))
		{
			return false;
		}
	}
	return true;
		
}


bool	btlGeometryUtil::areVerticesBehindPlane(const btlVector3& planeNormal, const btlAlignedObjectArray<btlVector3>& vertices, btlScalar	margin)
{
	int numvertices = vertices.size();
	for (int i=0;i<numvertices;i++)
	{
		const btlVector3& N1 = vertices[i];
		btlScalar dist = btlScalar(planeNormal.dot(N1))+btlScalar(planeNormal[3])-margin;
		if (dist>btlScalar(0.))
		{
			return false;
		}
	}
	return true;
}

bool btl_notExist(const btlVector3& planeEquation,const btlAlignedObjectArray<btlVector3>& planeEquations);

bool btl_notExist(const btlVector3& planeEquation,const btlAlignedObjectArray<btlVector3>& planeEquations)
{
	int numbrushes = planeEquations.size();
	for (int i=0;i<numbrushes;i++)
	{
		const btlVector3& N1 = planeEquations[i];
		if (planeEquation.dot(N1) > btlScalar(0.999))
		{
			return false;
		} 
	}
	return true;
}

void	btlGeometryUtil::getPlaneEquationsFromVertices(btlAlignedObjectArray<btlVector3>& vertices, btlAlignedObjectArray<btlVector3>& planeEquationsOut )
{
		const int numvertices = vertices.size();
	// brute force:
	for (int i=0;i<numvertices;i++)
	{
		const btlVector3& N1 = vertices[i];
		

		for (int j=i+1;j<numvertices;j++)
		{
			const btlVector3& N2 = vertices[j];
				
			for (int k=j+1;k<numvertices;k++)
			{

				const btlVector3& N3 = vertices[k];

				btlVector3 planeEquation,edge0,edge1;
				edge0 = N2-N1;
				edge1 = N3-N1;
				btlScalar normalSign = btlScalar(1.);
				for (int ww=0;ww<2;ww++)
				{
					planeEquation = normalSign * edge0.cross(edge1);
					if (planeEquation.length2() > btlScalar(0.0001))
					{
						planeEquation.normalize();
						if (btl_notExist(planeEquation,planeEquationsOut))
						{
							planeEquation[3] = -planeEquation.dot(N1);
							
								//check if inside, and replace supportingVertexOut if needed
								if (areVerticesBehindPlane(planeEquation,vertices,btlScalar(0.01)))
								{
									planeEquationsOut.push_back(planeEquation);
								}
						}
					}
					normalSign = btlScalar(-1.);
				}
			
			}
		}
	}

}

void	btlGeometryUtil::getVerticesFromPlaneEquations(const btlAlignedObjectArray<btlVector3>& planeEquations , btlAlignedObjectArray<btlVector3>& verticesOut )
{
	const int numbrushes = planeEquations.size();
	// brute force:
	for (int i=0;i<numbrushes;i++)
	{
		const btlVector3& N1 = planeEquations[i];
		

		for (int j=i+1;j<numbrushes;j++)
		{
			const btlVector3& N2 = planeEquations[j];
				
			for (int k=j+1;k<numbrushes;k++)
			{

				const btlVector3& N3 = planeEquations[k];

				btlVector3 n2n3; n2n3 = N2.cross(N3);
				btlVector3 n3n1; n3n1 = N3.cross(N1);
				btlVector3 n1n2; n1n2 = N1.cross(N2);
				
				if ( ( n2n3.length2() > btlScalar(0.0001) ) &&
					 ( n3n1.length2() > btlScalar(0.0001) ) &&
					 ( n1n2.length2() > btlScalar(0.0001) ) )
				{
					//point P out of 3 plane equations:

					//	d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )  
					//P =  -------------------------------------------------------------------------  
					//   N1 . ( N2 * N3 )  


					btlScalar quotient = (N1.dot(n2n3));
					if (btlFabs(quotient) > btlScalar(0.000001))
					{
						quotient = btlScalar(-1.) / quotient;
						n2n3 *= N1[3];
						n3n1 *= N2[3];
						n1n2 *= N3[3];
						btlVector3 potentialVertex = n2n3;
						potentialVertex += n3n1;
						potentialVertex += n1n2;
						potentialVertex *= quotient;

						//check if inside, and replace supportingVertexOut if needed
						if (isPointInsidePlanes(planeEquations,potentialVertex,btlScalar(0.01)))
						{
							verticesOut.push_back(potentialVertex);
						}
					}
				}
			}
		}
	}
}

