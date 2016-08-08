/*
* Copyright (2006 as c)-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions: dynamic 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

 part of rockdot_physics;
	



class b2PositionSolverManifold
{
	 b2PositionSolverManifold()
	{
		m_normal = new b2Vec2();
		m_separations = new List<double>(b2Settings.b2_maxManifoldPoints);
		m_points = new List<b2Vec2>(b2Settings.b2_maxManifoldPoints);
		for (int i = 0; i < b2Settings.b2_maxManifoldPoints; i++)
		{
			m_points[i] = new b2Vec2();
		}
	}
	
	 static b2Vec2 circlePointA = new b2Vec2();
	 static b2Vec2 circlePointB = new b2Vec2();
	  void Initialize(b2ContactConstraint cc)
	{
		b2Settings.b2Assert(cc.pointCount > 0);
		
		int i = 0;
		double clipPointX = 0.0;
		double clipPointY = 0.0;
		b2Mat22 tMat;
		b2Vec2 tVec;
		double planePointX = 0.0;
		double planePointY = 0.0;
		
		switch(cc.type)
		{
			case b2Manifold.e_circles:
			{
				//b2Vec2 pointA = cc.bodyA.GetWorldPoint(cc.localPoint);
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPoint;
				double pointAX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				double pointAY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				//b2Vec2 pointB = cc.bodyB.GetWorldPoint(cc.points[0].localPoint);
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.points[0].localPoint;
				double pointBX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				double pointBY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				double dX = pointBX - pointAX;
				double dY = pointBY - pointAY;
				double d2 = dX * dX + dY * dY;
				if (d2 > double.MIN_POSITIVE*double.MIN_POSITIVE)
				{
					double d = sqrt(d2);
					m_normal.x = dX/d;
					m_normal.y = dY/d;
				}
				else
				{
					m_normal.x = 1.0;
					m_normal.y = 0.0;
				}
				m_points[0].x = 0.5 * (pointAX + pointBX);
				m_points[0].y = 0.5 * (pointAY + pointBY);
				m_separations[0] = dX * m_normal.x + dY * m_normal.y - cc.radius;
			}
			break;
			case b2Manifold.e_faceA:
			{
				//m_normal = cc.bodyA.GetWorldListcc.localPlaneNormal);
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPlaneNormal;
				m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				//planePoint = cc.bodyA.GetWorldPoint(cc.localPoint);
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPoint;
				planePointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				planePointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				tMat = cc.bodyB.m_xf.R;
				for (i = 0; i < cc.pointCount;++i)
				{
					//clipPoint = cc.bodyB.GetWorldPoint(cc.points[i].localPoint);
					tVec = cc.points[i].localPoint;
					clipPointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
					clipPointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
					m_separations[i] = (clipPointX - planePointX) * m_normal.x + (clipPointY - planePointY) * m_normal.y - cc.radius;
					m_points[i].x = clipPointX;
					m_points[i].y = clipPointY;
				}
			}
			break;
			case b2Manifold.e_faceB:
			{
				//m_normal = cc.bodyB.GetWorldListcc.localPlaneNormal);
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.localPlaneNormal;
				m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				//planePoint = cc.bodyB.GetWorldPoint(cc.localPoint);
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.localPoint;
				planePointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				planePointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				tMat = cc.bodyA.m_xf.R;
				for (i = 0; i < cc.pointCount;++i)
				{
					//clipPoint = cc.bodyA.GetWorldPoint(cc.points[i].localPoint);
					tVec = cc.points[i].localPoint;
					clipPointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
					clipPointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
					m_separations[i] = (clipPointX - planePointX) * m_normal.x + (clipPointY - planePointY) * m_normal.y - cc.radius;
					m_points[i].Set(clipPointX, clipPointY);
				}
				
				// Ensure normal points from A to B
				m_normal.x *= -1;
				m_normal.y *= -1;
			}
			break;
		}
	}
	
	 b2Vec2 m_normal;
	 List<b2Vec2> m_points;
	 List<double> m_separations;
}
	
