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

 part of stagexl_box2d;
	


class b2SeparationFunction
{
	//edouble Type
	 static const int e_points = 0x01;
	 static const int e_faceA = 0x02;
	 static const int e_faceB = 0x04;
	
	  void Initialize(b2SimplexCache cache,b2DistanceProxy proxyA,b2Transform transformA,b2DistanceProxy proxyB,b2Transform transformB)
	{
		m_proxyA = proxyA;
		m_proxyB = proxyB;
		int count = cache.count;
		b2Settings.b2Assert(0 < count && count < 3);
		
		b2Vec2 localPointA;
		b2Vec2 localPointA1;
		b2Vec2 localPointA2;
		b2Vec2 localPointB;
		b2Vec2 localPointB1;
		b2Vec2 localPointB2;
		double pointAX = 0.0;
		double pointAY = 0.0;
		double pointBX = 0.0;
		double pointBY = 0.0;
		double normalX = 0.0;
		double normalY = 0.0;
		b2Mat22 tMat;
		b2Vec2 tVec;
		double s = 0.0;
		double sgn = 0.0;
		
		if (count == 1)
		{
			m_type = e_points;
			localPointA = m_proxyA.GetVertex(cache.indexA[0]);
			localPointB = m_proxyB.GetVertex(cache.indexB[0]);
			//pointA = b2Math.b2MulX(transformA, localPointA);
			tVec = localPointA;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			//pointB = b2Math.b2MulX(transformB, localPointB);
			tVec = localPointB;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			//m_axis = b2Math.SubtractVV(pointB, pointA);
			m_axis.x = pointBX - pointAX;
			m_axis.y = pointBY - pointAY;
			m_axis.Normalize();
		}
		else if (cache.indexB[0] == cache.indexB[1])
		{
			// Two points on A and one on B
			m_type = e_faceA;
			localPointA1 = m_proxyA.GetVertex(cache.indexA[0]);
			localPointA2 = m_proxyA.GetVertex(cache.indexA[1]);
			localPointB = m_proxyB.GetVertex(cache.indexB[0]);
			m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
			m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
			m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
			m_axis.Normalize();
			
			//normal = b2Math.b2MulMV(transformA.R, m_axis);
			tVec = m_axis;
			tMat = transformA.R;
			normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//pointA = b2Math.b2MulX(transformA, m_localPoint);
			tVec = m_localPoint;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			//pointB = b2Math.b2MulX(transformB, localPointB);
			tVec = localPointB;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			
			//float32 s = b2Dot(pointB - pointA, normal);
			s = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
			if (s < 0.0)
			{
				m_axis.NegativeSelf();
			}
		}
		else if (cache.indexA[0] == cache.indexA[0])
		{
			// Two points on B and one on A
			m_type = e_faceB;
			localPointB1 = m_proxyB.GetVertex(cache.indexB[0]);
			localPointB2 = m_proxyB.GetVertex(cache.indexB[1]);
			localPointA = m_proxyA.GetVertex(cache.indexA[0]);
			m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
			m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
			m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
			m_axis.Normalize();
			
			//normal = b2Math.b2MulMV(transformB.R, m_axis);
			tVec = m_axis;
			tMat = transformB.R;
			normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//pointB = b2Math.b2MulX(transformB, m_localPoint);
			tVec = m_localPoint;
			tMat = transformB.R;
			pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			//pointA = b2Math.b2MulX(transformA, localPointA);
			tVec = localPointA;
			tMat = transformA.R;
			pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			
			//float32 s = b2Dot(pointA - pointB, normal);
			s = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
			if (s < 0.0)
			{
				m_axis.NegativeSelf();
			}
		}
		else
		{
			// Two points on B and two points on A.
			// The faces are parallel.
			localPointA1 = m_proxyA.GetVertex(cache.indexA[0]);
			localPointA2 = m_proxyA.GetVertex(cache.indexA[1]);
			localPointB1 = m_proxyB.GetVertex(cache.indexB[0]);
			localPointB2 = m_proxyB.GetVertex(cache.indexB[1]);
			
			b2Vec2 pA = b2Math.MulX(transformA, localPointA);
			b2Vec2 dA = b2Math.MulMV(transformA.R, b2Math.SubtractVV(localPointA2, localPointA1));
			b2Vec2 pB = b2Math.MulX(transformB, localPointB);
			b2Vec2 dB = b2Math.MulMV(transformB.R, b2Math.SubtractVV(localPointB2, localPointB1));
			
			double a = dA.x * dA.x + dA.y * dA.y;
			double e = dB.x * dB.x + dB.y * dB.y;
			b2Vec2 r = b2Math.SubtractVV(dB, dA);
			double c = dA.x * r.x + dA.y * r.y;
			double f = dB.x * r.x + dB.y * r.y;
			
			double b = dA.x * dB.x + dA.y * dB.y;
			double denom = a * e-b * b;
			
			s = 0.0;
			if (denom != 0.0)
			{
				s = b2Math.Clamp((b * f - c * e) / denom, 0.0, 1.0);
			}
			
			double t = (b * s + f) / e;
			if (t < 0.0)
			{
				t = 0.0;
				s = b2Math.Clamp((b - c) / a, 0.0, 1.0);
			}
			
			//b2Vec2 localPointA = localPointA1 + s * (localPointA2 - localPointA1);
			localPointA = new b2Vec2();
			localPointA.x = localPointA1.x + s * (localPointA2.x - localPointA1.x);
			localPointA.y = localPointA1.y + s * (localPointA2.y - localPointA1.y);
			//b2Vec2 localPointB = localPointB1 + s * (localPointB2 - localPointB1);
			localPointB = new b2Vec2();
			localPointB.x = localPointB1.x + s * (localPointB2.x - localPointB1.x);
			localPointB.y = localPointB1.y + s * (localPointB2.y - localPointB1.y);
			
			if (s == 0.0 || s == 1.0)
			{
				m_type = e_faceB;
				m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
				m_axis.Normalize();
                
				m_localPoint = localPointB;
				
				//normal = b2Math.b2MulMV(transformB.R, m_axis);
				tVec = m_axis;
				tMat = transformB.R;
				normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				//pointB = b2Math.b2MulX(transformB, m_localPoint);
				tVec = m_localPoint;
				tMat = transformB.R;
				pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				//pointA = b2Math.b2MulX(transformA, localPointA);
				tVec = localPointA;
				tMat = transformA.R;
				pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				//float32 sgn = b2Dot(pointA - pointB, normal);
				sgn = (pointAX - pointBX) * normalX + (pointAY - pointBY) * normalY;
				if (s < 0.0)
				{
					m_axis.NegativeSelf();
				}
			}
			else
			{
				m_type = e_faceA;
				m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
				
				m_localPoint = localPointA;
				
				//normal = b2Math.b2MulMV(transformA.R, m_axis);
				tVec = m_axis;
				tMat = transformA.R;
				normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				//pointA = b2Math.b2MulX(transformA, m_localPoint);
				tVec = m_localPoint;
				tMat = transformA.R;
				pointAX = transformA.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointAY = transformA.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				//pointB = b2Math.b2MulX(transformB, localPointB);
				tVec = localPointB;
				tMat = transformB.R;
				pointBX = transformB.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				pointBY = transformB.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				//float32 sgn = b2Dot(pointB - pointA, normal);
				sgn = (pointBX - pointAX) * normalX + (pointBY - pointAY) * normalY;
				if (s < 0.0)
				{
					m_axis.NegativeSelf();
				}
			}
		}
	}
	
	  double Evaluate(b2Transform transformA,b2Transform transformB)
	{
		b2Vec2 axisA;
		b2Vec2 axisB;
		b2Vec2 localPointA;
		b2Vec2 localPointB;
		b2Vec2 pointA;
		b2Vec2 pointB;
		double seperation = 0.0;
		b2Vec2 normal;
		switch(m_type)
		{
			case e_points:
			{
				axisA = b2Math.MulTMV(transformA.R, m_axis);
				axisB = b2Math.MulTMV(transformB.R, m_axis.GetNegative());
				localPointA = m_proxyA.GetSupportVertex(axisA);
				localPointB = m_proxyB.GetSupportVertex(axisB);
				pointA = b2Math.MulX(transformA, localPointA);
				pointB = b2Math.MulX(transformB, localPointB);
				//float32 separation = b2Dot(pointB - pointA, m_axis);
				seperation = (pointB.x - pointA.x) * m_axis.x + (pointB.y - pointA.y) * m_axis.y;
				return seperation;
			}
			case e_faceA:
			{
				normal = b2Math.MulMV(transformA.R, m_axis);
				pointA = b2Math.MulX(transformA, m_localPoint);
				
				axisB = b2Math.MulTMV(transformB.R, normal.GetNegative());
				
				localPointB = m_proxyB.GetSupportVertex(axisB);
				pointB = b2Math.MulX(transformB, localPointB);
				
				//float32 separation = b2Dot(pointB - pointA, normal);
				seperation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
				return seperation;
			}
			case e_faceB:
			{
				normal = b2Math.MulMV(transformB.R, m_axis);
				pointB = b2Math.MulX(transformB, m_localPoint);
				
				axisA = b2Math.MulTMV(transformA.R, normal.GetNegative());
				
				localPointA = m_proxyA.GetSupportVertex(axisA);
				pointA = b2Math.MulX(transformA, localPointA);
				
				//float32 separation = b2Dot(pointA - pointB, normal);
				seperation = (pointA.x - pointB.x) * normal.x + (pointA.y - pointB.y) * normal.y;
				return seperation;
			}
			default:
			b2Settings.b2Assert(false);
			return 0.0;
		}
	}
	
	 b2DistanceProxy m_proxyA;
	 b2DistanceProxy m_proxyB;
	 int m_type = 0;
	 b2Vec2 m_localPoint = new b2Vec2();
	 b2Vec2 m_axis = new b2Vec2();
}
	
