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


#ifndef BT_IDEBUG_DRAW__H
#define BT_IDEBUG_DRAW__H

#include "btVector3.h"
#include "btTransform.h"



///The btIDebugDraw interface class allows hooking up a debug renderer to visually debug simulations.
///Typical use case: create a debug drawer object, and assign it to a btCollisionWorld or btDynamicsWorld using setDebugDrawer and call debugDrawWorld.
///A class that implements the btIDebugDraw interface has to implement the drawLine method at a minimum.
///For color arguments the X,Y,Z components refer to Red, Green and Blue each in the range [0..1]
class	btlIDebugDraw
{
	public:

	ATTRIBUTE_ALIGNED16(struct) DefaultColors
	{
		btlVector3	m_activeObject;
		btlVector3	m_deactivatedObject;
		btlVector3	m_wantsDeactivationObject;
		btlVector3	m_disabledDeactivationObject;
		btlVector3	m_disabledSimulationObject;
		btlVector3	m_aabb;
		btlVector3 m_contactPoint;
		
		DefaultColors()
		:	m_activeObject(1,1,1),
			m_deactivatedObject(0,1,0),
			m_wantsDeactivationObject(0,1,1),
			m_disabledDeactivationObject(1,0,0),
			m_disabledSimulationObject(1,1,0),
			m_aabb(1,0,0),
			m_contactPoint(1,1,0)
		{
		}
	};

	
	enum	DebugDrawModes
	{
		DBG_NoDebug=0,
		DBG_DrawWireframe = 1,
		DBG_DrawAabb=2,
		DBG_DrawFeaturesText=4,
		DBG_DrawContactPoints=8,
		DBG_NoDeactivation=16,
		DBG_NoHelpText = 32,
		DBG_DrawText=64,
		DBG_ProfileTimings = 128,
		DBG_EnableSatComparison = 256,
		DBG_DisableBulletLCP = 512,
		DBG_EnableCCD = 1024,
		DBG_DrawConstraints = (1 << 11),
		DBG_DrawConstraintLimits = (1 << 12),
		DBG_FastWireframe = (1<<13),
		DBG_DrawNormals = (1<<14),
		DBG_DrawFrames = (1<<15),
		DBG_MAX_DEBUG_DRAW_MODE
	};

	virtual ~btlIDebugDraw() {};

	
	virtual DefaultColors	getDefaultColors() const	{	DefaultColors colors;	return colors;	}
	///the default implementation for setDefaultColors has no effect. A derived class can implement it and store the colors.
	virtual void setDefaultColors(const DefaultColors& /*colors*/) {}
	
	virtual void	drawLine(const btlVector3& from,const btlVector3& to,const btlVector3& color)=0;
		
	virtual void    drawLine(const btlVector3& from,const btlVector3& to, const btlVector3& fromColor, const btlVector3& toColor)
	{
        (void) toColor;
		drawLine (from, to, fromColor);
	}

	virtual void	drawSphere(btlScalar radius, const btlTransform& transform, const btlVector3& color)
	{
		
		btlVector3 center = transform.getOrigin();
		btlVector3 up = transform.getBasis().getColumn(1);
		btlVector3 axis = transform.getBasis().getColumn(0);
		btlScalar minTh = -SIMD_HALF_PI;
		btlScalar maxTh = SIMD_HALF_PI;
		btlScalar minPs = -SIMD_HALF_PI;
		btlScalar maxPs = SIMD_HALF_PI;
		btlScalar stepDegrees = 30.f;
		drawSpherePatch(center, up, axis, radius,minTh, maxTh, minPs, maxPs, color, stepDegrees ,false);
		drawSpherePatch(center, up, -axis, radius,minTh, maxTh, minPs, maxPs, color, stepDegrees,false );
	}
	
	virtual void	drawSphere (const btlVector3& p, btlScalar radius, const btlVector3& color)
	{
		btlTransform tr;
		tr.setIdentity();
		tr.setOrigin(p);
		drawSphere(radius,tr,color);
	}
	
	virtual	void	drawTriangle(const btlVector3& v0,const btlVector3& v1,const btlVector3& v2,const btlVector3& /*n0*/,const btlVector3& /*n1*/,const btlVector3& /*n2*/,const btlVector3& color, btlScalar alpha)
	{
		drawTriangle(v0,v1,v2,color,alpha);
	}
	virtual	void	drawTriangle(const btlVector3& v0,const btlVector3& v1,const btlVector3& v2,const btlVector3& color, btlScalar /*alpha*/)
	{
		drawLine(v0,v1,color);
		drawLine(v1,v2,color);
		drawLine(v2,v0,color);
	}

	virtual void	drawContactPoint(const btlVector3& PointOnB,const btlVector3& normalOnB,btlScalar distance,int lifeTime,const btlVector3& color)=0;

	virtual void	reportErrorWarning(const char* warningString) = 0;

	virtual void	draw3dText(const btlVector3& location,const char* textString) = 0;
	
	virtual void	setDebugMode(int debugMode) =0;
	
	virtual int		getDebugMode() const = 0;

	virtual void drawAabb(const btlVector3& from,const btlVector3& to,const btlVector3& color)
	{

		btlVector3 halfExtents = (to-from)* 0.5f;
		btlVector3 center = (to+from) *0.5f;
		int i,j;

		btlVector3 edgecoord(1.f,1.f,1.f),pa,pb;
		for (i=0;i<4;i++)
		{
			for (j=0;j<3;j++)
			{
				pa = btlVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],		
					edgecoord[2]*halfExtents[2]);
				pa+=center;

				int othercoord = j%3;
				edgecoord[othercoord]*=-1.f;
				pb = btlVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],	
					edgecoord[2]*halfExtents[2]);
				pb+=center;

				drawLine(pa,pb,color);
			}
			edgecoord = btlVector3(-1.f,-1.f,-1.f);
			if (i<3)
				edgecoord[i]*=-1.f;
		}
	}
	virtual void drawTransform(const btlTransform& transform, btlScalar orthoLen)
	{
		btlVector3 start = transform.getOrigin();
		drawLine(start, start+transform.getBasis() * btlVector3(orthoLen, 0, 0), btlVector3(0.7f,0,0));
		drawLine(start, start+transform.getBasis() * btlVector3(0, orthoLen, 0), btlVector3(0,0.7f,0));
		drawLine(start, start+transform.getBasis() * btlVector3(0, 0, orthoLen), btlVector3(0,0,0.7f));
	}

	virtual void drawArc(const btlVector3& center, const btlVector3& normal, const btlVector3& axis, btlScalar radiusA, btlScalar radiusB, btlScalar minAngle, btlScalar maxAngle, 
				const btlVector3& color, bool drawSect, btlScalar stepDegrees = btlScalar(10.f))
	{
		const btlVector3& vx = axis;
		btlVector3 vy = normal.cross(axis);
		btlScalar step = stepDegrees * SIMD_RADS_PER_DEG;
		int nSteps = (int)btlFabs((maxAngle - minAngle) / step);
		if(!nSteps) nSteps = 1;
		btlVector3 prev = center + radiusA * vx * btlCos(minAngle) + radiusB * vy * btlSin(minAngle);
		if(drawSect)
		{
			drawLine(center, prev, color);
		}
		for(int i = 1; i <= nSteps; i++)
		{
			btlScalar angle = minAngle + (maxAngle - minAngle) * btlScalar(i) / btlScalar(nSteps);
			btlVector3 next = center + radiusA * vx * btlCos(angle) + radiusB * vy * btlSin(angle);
			drawLine(prev, next, color);
			prev = next;
		}
		if(drawSect)
		{
			drawLine(center, prev, color);
		}
	}
	virtual void drawSpherePatch(const btlVector3& center, const btlVector3& up, const btlVector3& axis, btlScalar radius, 
		btlScalar minTh, btlScalar maxTh, btlScalar minPs, btlScalar maxPs, const btlVector3& color, btlScalar stepDegrees = btlScalar(10.f),bool drawCenter = true)
	{
		btlVector3 vA[74];
		btlVector3 vB[74];
		btlVector3 *pvA = vA, *pvB = vB, *pT;
		btlVector3 npole = center + up * radius;
		btlVector3 spole = center - up * radius;
		btlVector3 arcStart;
		btlScalar step = stepDegrees * SIMD_RADS_PER_DEG;
		const btlVector3& kv = up;
		const btlVector3& iv = axis;
		btlVector3 jv = kv.cross(iv);
		bool drawN = false;
		bool drawS = false;
		if(minTh <= -SIMD_HALF_PI)
		{
			minTh = -SIMD_HALF_PI + step;
			drawN = true;
		}
		if(maxTh >= SIMD_HALF_PI)
		{
			maxTh = SIMD_HALF_PI - step;
			drawS = true;
		}
		if(minTh > maxTh)
		{
			minTh = -SIMD_HALF_PI + step;
			maxTh =  SIMD_HALF_PI - step;
			drawN = drawS = true;
		}
		int n_hor = (int)((maxTh - minTh) / step) + 1;
		if(n_hor < 2) n_hor = 2;
		btlScalar step_h = (maxTh - minTh) / btlScalar(n_hor - 1);
		bool isClosed = false;
		if(minPs > maxPs)
		{
			minPs = -SIMD_PI + step;
			maxPs =  SIMD_PI;
			isClosed = true;
		}
		else if((maxPs - minPs) >= SIMD_PI * btlScalar(2.f))
		{
			isClosed = true;
		}
		else
		{
			isClosed = false;
		}
		int n_vert = (int)((maxPs - minPs) / step) + 1;
		if(n_vert < 2) n_vert = 2;
		btlScalar step_v = (maxPs - minPs) / btlScalar(n_vert - 1);
		for(int i = 0; i < n_hor; i++)
		{
			btlScalar th = minTh + btlScalar(i) * step_h;
			btlScalar sth = radius * btlSin(th);
			btlScalar cth = radius * btlCos(th);
			for(int j = 0; j < n_vert; j++)
			{
				btlScalar psi = minPs + btlScalar(j) * step_v;
				btlScalar sps = btlSin(psi);
				btlScalar cps = btlCos(psi);
				pvB[j] = center + cth * cps * iv + cth * sps * jv + sth * kv;
				if(i)
				{
					drawLine(pvA[j], pvB[j], color);
				}
				else if(drawS)
				{
					drawLine(spole, pvB[j], color);
				}
				if(j)
				{
					drawLine(pvB[j-1], pvB[j], color);
				}
				else
				{
					arcStart = pvB[j];
				}
				if((i == (n_hor - 1)) && drawN)
				{
					drawLine(npole, pvB[j], color);
				}
				
				if (drawCenter)
				{
					if(isClosed)
					{
						if(j == (n_vert-1))
						{
							drawLine(arcStart, pvB[j], color);
						}
					}
					else
					{
						if(((!i) || (i == (n_hor-1))) && ((!j) || (j == (n_vert-1))))
						{
							drawLine(center, pvB[j], color);
						}
					}
				}
			}
			pT = pvA; pvA = pvB; pvB = pT;
		}
	}
	
  
	virtual void drawBox(const btlVector3& bbMin, const btlVector3& bbMax, const btlVector3& color)
	{
		drawLine(btlVector3(bbMin[0], bbMin[1], bbMin[2]), btlVector3(bbMax[0], bbMin[1], bbMin[2]), color);
		drawLine(btlVector3(bbMax[0], bbMin[1], bbMin[2]), btlVector3(bbMax[0], bbMax[1], bbMin[2]), color);
		drawLine(btlVector3(bbMax[0], bbMax[1], bbMin[2]), btlVector3(bbMin[0], bbMax[1], bbMin[2]), color);
		drawLine(btlVector3(bbMin[0], bbMax[1], bbMin[2]), btlVector3(bbMin[0], bbMin[1], bbMin[2]), color);
		drawLine(btlVector3(bbMin[0], bbMin[1], bbMin[2]), btlVector3(bbMin[0], bbMin[1], bbMax[2]), color);
		drawLine(btlVector3(bbMax[0], bbMin[1], bbMin[2]), btlVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(btlVector3(bbMax[0], bbMax[1], bbMin[2]), btlVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(btlVector3(bbMin[0], bbMax[1], bbMin[2]), btlVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(btlVector3(bbMin[0], bbMin[1], bbMax[2]), btlVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(btlVector3(bbMax[0], bbMin[1], bbMax[2]), btlVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(btlVector3(bbMax[0], bbMax[1], bbMax[2]), btlVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(btlVector3(bbMin[0], bbMax[1], bbMax[2]), btlVector3(bbMin[0], bbMin[1], bbMax[2]), color);
	}
	virtual void drawBox(const btlVector3& bbMin, const btlVector3& bbMax, const btlTransform& trans, const btlVector3& color)
	{
		drawLine(trans * btlVector3(bbMin[0], bbMin[1], bbMin[2]), trans * btlVector3(bbMax[0], bbMin[1], bbMin[2]), color);
		drawLine(trans * btlVector3(bbMax[0], bbMin[1], bbMin[2]), trans * btlVector3(bbMax[0], bbMax[1], bbMin[2]), color);
		drawLine(trans * btlVector3(bbMax[0], bbMax[1], bbMin[2]), trans * btlVector3(bbMin[0], bbMax[1], bbMin[2]), color);
		drawLine(trans * btlVector3(bbMin[0], bbMax[1], bbMin[2]), trans * btlVector3(bbMin[0], bbMin[1], bbMin[2]), color);
		drawLine(trans * btlVector3(bbMin[0], bbMin[1], bbMin[2]), trans * btlVector3(bbMin[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * btlVector3(bbMax[0], bbMin[1], bbMin[2]), trans * btlVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * btlVector3(bbMax[0], bbMax[1], bbMin[2]), trans * btlVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * btlVector3(bbMin[0], bbMax[1], bbMin[2]), trans * btlVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * btlVector3(bbMin[0], bbMin[1], bbMax[2]), trans * btlVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * btlVector3(bbMax[0], bbMin[1], bbMax[2]), trans * btlVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * btlVector3(bbMax[0], bbMax[1], bbMax[2]), trans * btlVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * btlVector3(bbMin[0], bbMax[1], bbMax[2]), trans * btlVector3(bbMin[0], bbMin[1], bbMax[2]), color);
	}

	virtual void drawCapsule(btlScalar radius, btlScalar halfHeight, int upAxis, const btlTransform& transform, const btlVector3& color)
	{
		int stepDegrees = 30;

		btlVector3 capStart(0.f,0.f,0.f);
		capStart[upAxis] = -halfHeight;

		btlVector3 capEnd(0.f,0.f,0.f);
		capEnd[upAxis] = halfHeight;

		// Draw the ends
		{

			btlTransform childTransform = transform;
			childTransform.getOrigin() = transform * capStart;
			{
				btlVector3 center = childTransform.getOrigin();
				btlVector3 up = childTransform.getBasis().getColumn((upAxis+1)%3);
				btlVector3 axis = -childTransform.getBasis().getColumn(upAxis);
				btlScalar minTh = -SIMD_HALF_PI;
				btlScalar maxTh = SIMD_HALF_PI;
				btlScalar minPs = -SIMD_HALF_PI;
				btlScalar maxPs = SIMD_HALF_PI;
				
				drawSpherePatch(center, up, axis, radius,minTh, maxTh, minPs, maxPs, color, btlScalar(stepDegrees) ,false);
			}



		}

		{
			btlTransform childTransform = transform;
			childTransform.getOrigin() = transform * capEnd;
			{
				btlVector3 center = childTransform.getOrigin();
				btlVector3 up = childTransform.getBasis().getColumn((upAxis+1)%3);
				btlVector3 axis = childTransform.getBasis().getColumn(upAxis);
				btlScalar minTh = -SIMD_HALF_PI;
				btlScalar maxTh = SIMD_HALF_PI;
				btlScalar minPs = -SIMD_HALF_PI;
				btlScalar maxPs = SIMD_HALF_PI;
				drawSpherePatch(center, up, axis, radius,minTh, maxTh, minPs, maxPs, color, btlScalar(stepDegrees) ,false);
			}
		}

		// Draw some additional lines
		btlVector3 start = transform.getOrigin();

		for (int i=0;i<360;i+=stepDegrees)
		{
			capEnd[(upAxis+1)%3] = capStart[(upAxis+1)%3] = btlSin(btlScalar(i)*SIMD_RADS_PER_DEG)*radius;
			capEnd[(upAxis+2)%3] = capStart[(upAxis+2)%3]  = btlCos(btlScalar(i)*SIMD_RADS_PER_DEG)*radius;
			drawLine(start+transform.getBasis() * capStart,start+transform.getBasis() * capEnd, color);
		}
		
	}

	virtual void drawCylinder(btlScalar radius, btlScalar halfHeight, int upAxis, const btlTransform& transform, const btlVector3& color)
	{
		btlVector3 start = transform.getOrigin();
		btlVector3	offsetHeight(0,0,0);
		offsetHeight[upAxis] = halfHeight;
		int stepDegrees=30;
		btlVector3 capStart(0.f,0.f,0.f);
		capStart[upAxis] = -halfHeight;
		btlVector3 capEnd(0.f,0.f,0.f);
		capEnd[upAxis] = halfHeight;

		for (int i=0;i<360;i+=stepDegrees)
		{
			capEnd[(upAxis+1)%3] = capStart[(upAxis+1)%3] = btlSin(btlScalar(i)*SIMD_RADS_PER_DEG)*radius;
			capEnd[(upAxis+2)%3] = capStart[(upAxis+2)%3]  = btlCos(btlScalar(i)*SIMD_RADS_PER_DEG)*radius;
			drawLine(start+transform.getBasis() * capStart,start+transform.getBasis() * capEnd, color);
		}
		// Drawing top and bottom caps of the cylinder
		btlVector3 yaxis(0,0,0);
		yaxis[upAxis] = btlScalar(1.0);
		btlVector3 xaxis(0,0,0);
		xaxis[(upAxis+1)%3] = btlScalar(1.0);
		drawArc(start-transform.getBasis()*(offsetHeight),transform.getBasis()*yaxis,transform.getBasis()*xaxis,radius,radius,0,SIMD_2_PI,color,false,btlScalar(10.0));
		drawArc(start+transform.getBasis()*(offsetHeight),transform.getBasis()*yaxis,transform.getBasis()*xaxis,radius,radius,0,SIMD_2_PI,color,false,btlScalar(10.0));
	}

	virtual void drawCone(btlScalar radius, btlScalar height, int upAxis, const btlTransform& transform, const btlVector3& color)
	{
		int stepDegrees = 30;
		btlVector3 start = transform.getOrigin();

		btlVector3	offsetHeight(0,0,0);
		btlScalar halfHeight = height * btlScalar(0.5);
		offsetHeight[upAxis] = halfHeight;
		btlVector3	offsetRadius(0,0,0);
		offsetRadius[(upAxis+1)%3] = radius;
		btlVector3	offset2Radius(0,0,0);
		offset2Radius[(upAxis+2)%3] = radius;


		btlVector3 capEnd(0.f,0.f,0.f);
		capEnd[upAxis] = -halfHeight;

		for (int i=0;i<360;i+=stepDegrees)
		{
			capEnd[(upAxis+1)%3] = btlSin(btlScalar(i)*SIMD_RADS_PER_DEG)*radius;
			capEnd[(upAxis+2)%3] = btlCos(btlScalar(i)*SIMD_RADS_PER_DEG)*radius;
			drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * capEnd, color);
		}

		drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * (-offsetHeight+offsetRadius),color);
		drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * (-offsetHeight-offsetRadius),color);
		drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * (-offsetHeight+offset2Radius),color);
		drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * (-offsetHeight-offset2Radius),color);

		// Drawing the base of the cone
		btlVector3 yaxis(0,0,0);
		yaxis[upAxis] = btlScalar(1.0);
		btlVector3 xaxis(0,0,0);
		xaxis[(upAxis+1)%3] = btlScalar(1.0);
		drawArc(start-transform.getBasis()*(offsetHeight),transform.getBasis()*yaxis,transform.getBasis()*xaxis,radius,radius,0,SIMD_2_PI,color,false,10.0);
	}

	virtual void drawPlane(const btlVector3& planeNormal, btlScalar planeConst, const btlTransform& transform, const btlVector3& color)
	{
		btlVector3 planeOrigin = planeNormal * planeConst;
		btlVector3 vec0,vec1;
		btlPlaneSpace1(planeNormal,vec0,vec1);
		btlScalar vecLen = 100.f;
		btlVector3 pt0 = planeOrigin + vec0*vecLen;
		btlVector3 pt1 = planeOrigin - vec0*vecLen;
		btlVector3 pt2 = planeOrigin + vec1*vecLen;
		btlVector3 pt3 = planeOrigin - vec1*vecLen;
		drawLine(transform*pt0,transform*pt1,color);
		drawLine(transform*pt2,transform*pt3,color);
	}

	virtual void flushLines()
	{
	}
};


#endif //BT_IDEBUG_DRAW__H

