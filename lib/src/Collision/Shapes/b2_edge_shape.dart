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








/**
 * An edge shape.
 * @
 * @see b2EdgeChainDef
 */
 class b2EdgeShape extends b2Shape
{
	/**
	* Returns false. Edges cannot contain points. 
	*/
	 @override 
		 bool TestPoint(b2Transform transform,b2Vec2 p){
		return false;
	}

	/**
	* @inheritDoc
	*/
	 @override 
		 bool RayCast(b2RayCastOutput output,b2RayCastInput input,b2Transform transform)
	{
		b2Mat22 tMat;
		double rX = input.p2.x - input.p1.x;
		double rY = input.p2.y - input.p1.y;
		
		//b2Vec2 v1 = b2Mul(transform, m_v1);
		tMat = transform.R;
		double v1X = transform.position.x + (tMat.col1.x * m_v1.x + tMat.col2.x * m_v1.y);
		double v1Y = transform.position.y + (tMat.col1.y * m_v1.x + tMat.col2.y * m_v1.y);
		
		//b2Vec2 n = b2Cross(d, 1.0);
		double nX = transform.position.y + (tMat.col1.y * m_v2.x + tMat.col2.y * m_v2.y) - v1Y;
		double nY = -(transform.position.x + (tMat.col1.x * m_v2.x + tMat.col2.x * m_v2.y) - v1X);
		
		double k_slop = 100.0 * double.MIN_POSITIVE;
		double denom = -(rX * nX + rY * nY);
	
		// Cull back facing collision and ignore parallel segments.
		if (denom > k_slop)
		{
			// Does the segment intersect the infinite line associated with this segment?
			double bX = input.p1.x - v1X;
			double bY = input.p1.y - v1Y;
			double a = (bX * nX + bY * nY);
	
			if (0.0 <= a && a <= input.maxFraction * denom)
			{
				double mu2 = -rX * bY + rY * bX;
	
				// Does the segment intersect this segment?
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop))
				{
					a /= denom;
					output.fraction = a;
					double nLen = sqrt(nX * nX + nY * nY);
					output.normal.x = nX / nLen;
					output.normal.y = nY / nLen;
					return true;
				}
			}
		}
		
		return false;
	}

	/**
	* @inheritDoc
	*/
	 @override 
		 void ComputeAABB(b2AABB aabb,b2Transform transform){
		b2Mat22 tMat = transform.R;
		//b2Vec2 v1 = b2Mul(transform, m_v1);
		double v1X = transform.position.x + (tMat.col1.x * m_v1.x + tMat.col2.x * m_v1.y);
		double v1Y = transform.position.y + (tMat.col1.y * m_v1.x + tMat.col2.y * m_v1.y);
		//b2Vec2 v2 = b2Mul(transform, m_v2);
		double v2X = transform.position.x + (tMat.col1.x * m_v2.x + tMat.col2.x * m_v2.y);
		double v2Y = transform.position.y + (tMat.col1.y * m_v2.x + tMat.col2.y * m_v2.y);
		if (v1X < v2X) {
			aabb.lowerBound.x = v1X;
			aabb.upperBound.x = v2X;
		} else {
			aabb.lowerBound.x = v2X;
			aabb.upperBound.x = v1X;
		}
		if (v1Y < v2Y) {
			aabb.lowerBound.y = v1Y;
			aabb.upperBound.y = v2Y;
		} else {
			aabb.lowerBound.y = v2Y;
			aabb.upperBound.y = v1Y;
		}
	}

	/**
	* @inheritDoc
	*/
	 @override 
		 void ComputeMass(b2MassData massData,double density){
		massData.mass = 0.0;
		massData.center.SetV(m_v1);
		massData.I = 0.0;
	}
	
	/**
	* @inheritDoc
	*/
	 @override 
		 double ComputeSubmergedArea(b2Vec2 normal,double offset,b2Transform xf,b2Vec2 c)
	{
		// Note that v0 is independant of any details of the specific edge
		// We are relying on v0 being consistent between multiple edges of the same body
		//b2Vec2 v0 = offset * normal;
		b2Vec2 v0 = new b2Vec2(normal.x * offset, normal.y * offset);
		
		b2Vec2 v1 = b2Math.MulX(xf, m_v1);
		b2Vec2 v2 = b2Math.MulX(xf, m_v2);
		
		double d1 = b2Math.Dot(normal, v1) - offset;
		double d2 = b2Math.Dot(normal, v2) - offset;
		if (d1 > 0)
		{
			if (d2 > 0)
			{
				return 0.0;
			}
			else
			{
				//v1 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
				v1.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
				v1.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y;
			}
		}
		else
		{
			if (d2 > 0)
			{
				//v2 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
				v2.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
				v2.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y;
			}
			else
			{
				// Nothing
			}
		}
		// v0,v1,v2 represents a fully submerged triangle
		// Area weighted centroid
		c.x = (v0.x + v1.x + v2.x) / 3;
		c.y = (v0.y + v1.y + v2.y) / 3;
		
		//b2Vec2 e1 = v1 - v0;
		//b2Vec2 e2 = v2 - v0;
		//return 0.5f * b2Cross(e1, e2);
		return 0.5 * ( (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x) );
	}

	/**
	* Get the distance from vertex1 to vertex2.
	*/
	  double GetLength()
	{
		return m_length;
	}

	/**
	* Get the local position of vertex1 in parent body.
	*/
	  b2Vec2 GetVertex1()
	{
		return m_v1;
	}

	/**
	* Get the local position of vertex2 in parent body.
	*/
	  b2Vec2 GetVertex2()
	{
		return m_v2;
	}

	/**
	* Get a core vertex in local coordinates. These vertices
	* represent a smaller edge that is used for time of impact
	* computations.
	*/
	  b2Vec2 GetCoreVertex1()
	{
		return m_coreV1;
	}

	/**
	* Get a core vertex in local coordinates. These vertices
	* represent a smaller edge that is used for time of impact
	* computations.
	*/
	  b2Vec2 GetCoreVertex2()
	{
		return m_coreV2;
	}
	
	/**
	* Get a perpendicular unit vector, pointing
	* from the solid side to the empty side.
	*/
	  b2Vec2 GetNormalVector()
	{
		return m_normal;
	}
	
	
	/**
	* Get a parallel unit vector, pointing
	* from vertex1 to vertex2.
	*/
	  b2Vec2 GetDirectionVector()
	{
		return m_direction;
	}
	
	/**
	* Returns a unit vector halfway between 
	* m_direction and m_prevEdge.m_direction.
	*/
	  b2Vec2 GetCorner1Vector()
	{
		return m_cornerDir1;
	}
	
	/**
	* Returns a unit vector halfway between 
	* m_direction and m_nextEdge.m_direction.
	*/
	  b2Vec2 GetCorner2Vector()
	{
		return m_cornerDir2;
	}
	
	/**
	* Returns true if the first corner of this edge
	* bends towards the solid side.
	*/
	  bool Corner1IsConvex()
	{
		return m_cornerConvex1;
	}
	
	/**
	* Returns true if the second corner of this edge
	* bends towards the solid side. 
	*/
	  bool Corner2IsConvex()
	{
		return m_cornerConvex2;
	}

	/**
	* Get the first vertex and apply the supplied transform.
	*/
	  b2Vec2 GetFirstVertex(b2Transform xf)
	{
		//return b2Mul(xf, m_coreV1);
		b2Mat22 tMat = xf.R;
		return new b2Vec2(xf.position.x + (tMat.col1.x * m_coreV1.x + tMat.col2.x * m_coreV1.y),
		                  xf.position.y + (tMat.col1.y * m_coreV1.x + tMat.col2.y * m_coreV1.y));
	}
	
	/**
	* Get the next edge in the chain.
	*/
	  b2EdgeShape GetNextEdge()
	{
		return m_nextEdge;
	}
	
	/**
	* Get the previous edge in the chain.
	*/
	  b2EdgeShape GetPrevEdge()
	{
		return m_prevEdge;
	}

	 b2Vec2 s_supportVec = new b2Vec2();
	/**
	* Get the support point in the given world direction.
	* Use the supplied transform.
	*/
	  b2Vec2 Support(b2Transform xf,double dX,double dY){
		b2Mat22 tMat = xf.R;
		//b2Vec2 v1 = b2Mul(xf, m_coreV1);
		double v1X = xf.position.x + (tMat.col1.x * m_coreV1.x + tMat.col2.x * m_coreV1.y);
		double v1Y = xf.position.y + (tMat.col1.y * m_coreV1.x + tMat.col2.y * m_coreV1.y);
		
		//b2Vec2 v2 = b2Mul(xf, m_coreV2);
		double v2X = xf.position.x + (tMat.col1.x * m_coreV2.x + tMat.col2.x * m_coreV2.y);
		double v2Y = xf.position.y + (tMat.col1.y * m_coreV2.x + tMat.col2.y * m_coreV2.y);
		
		if ((v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)) {
			s_supportVec.x = v1X;
			s_supportVec.y = v1Y;
		} else {
			s_supportVec.x = v2X;
			s_supportVec.y = v2Y;
		}
		return s_supportVec;
	}
	
	//--------------- Internals Below -------------------

	/**
	* @
	*/
	 b2EdgeShape(b2Vec2 v1,b2Vec2 v2): super() {
		m_type = b2Shape.e_edgeShape;
		
		m_prevEdge = null;
		m_nextEdge = null;
		
		m_v1 = v1;
		m_v2 = v2;
		
		m_direction.Set(m_v2.x - m_v1.x, m_v2.y - m_v1.y);
		m_length = m_direction.Normalize();
		m_normal.Set(m_direction.y, -m_direction.x);
		
		m_coreV1.Set(-b2Settings.b2_toiSlop * (m_normal.x - m_direction.x) + m_v1.x,
		             -b2Settings.b2_toiSlop * (m_normal.y - m_direction.y) + m_v1.y);
		m_coreV2.Set(-b2Settings.b2_toiSlop * (m_normal.x + m_direction.x) + m_v2.x,
		             -b2Settings.b2_toiSlop * (m_normal.y + m_direction.y) + m_v2.y);
		
		m_cornerDir1 = m_normal;
		m_cornerDir2.Set(-m_normal.x, -m_normal.y);
	}

	/**
	* @
	*/
	 void SetPrevEdge(b2EdgeShape edge,b2Vec2 core,b2Vec2 cornerDir,bool convex)
	{
		m_prevEdge = edge;
		m_coreV1 = core;
		m_cornerDir1 = cornerDir;
		m_cornerConvex1 = convex;
	}
	
	/**
	* @
	*/
	 void SetNextEdge(b2EdgeShape edge,b2Vec2 core,b2Vec2 cornerDir,bool convex)
	{
		m_nextEdge = edge;
		m_coreV2 = core;
		m_cornerDir2 = cornerDir;
		m_cornerConvex2 = convex;
	}

	b2Vec2 m_v1 = new b2Vec2();
	b2Vec2 m_v2 = new b2Vec2();
	
	b2Vec2 m_coreV1 = new b2Vec2();
	b2Vec2 m_coreV2 = new b2Vec2();
	
	double m_length = 0.0;
	
	b2Vec2 m_normal = new b2Vec2();
	
	b2Vec2 m_direction = new b2Vec2();
	
	b2Vec2 m_cornerDir1 = new b2Vec2();
	
	b2Vec2 m_cornerDir2 = new b2Vec2();
	
	bool m_cornerConvex1;
	bool m_cornerConvex2;
	
	b2EdgeShape m_nextEdge;
	b2EdgeShape m_prevEdge;
	
}

