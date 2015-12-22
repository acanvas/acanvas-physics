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

 part of rockdot_box2d;




/**
* Convex polygon. The vertices must be in CCW order for a right-handed
* coordinate system with the z-axis coming out of the screen.
* @see b2PolygonDef
*/

 class b2PolygonShape extends b2Shape
{
	 @override 
		 b2Shape Copy() 
	{
		b2PolygonShape s = new b2PolygonShape();
		s.Set(this);
		return s;
	}
	
	 @override 
		 void Set(b2Shape other) 
	{
		super.Set(other);
		if (other is b2PolygonShape)
		{
			b2PolygonShape other2 = other as b2PolygonShape;
			m_centroid.SetV(other2.m_centroid);
			m_vertexCount = other2.m_vertexCount;
			Reserve(m_vertexCount);
			for (int i = 0; i < m_vertexCount; i++)
			{
				m_vertices[i].SetV(other2.m_vertices[i]);
				m_normals[i].SetV(other2.m_normals[i]);
			}
		}
	}
	
	/**
	 * Copy vertices. This assumes the vertices define a convex polygon.
	 * It is assumed that the exterior is the the right of each edge.
	 */
	  void SetAsArray(List vertices,[int vertexCount=0])
	{
		List<b2Vec2> v = new List<b2Vec2>();
		for(b2Vec2 tVec in vertices)
		{
			v.add(tVec);
		}
		SetAsVector(v, vertexCount);
	}
	
	 static  b2PolygonShape AsArray(List vertices,int vertexCount)
	{
		b2PolygonShape polygonShape = new b2PolygonShape();
		polygonShape.SetAsVector(vertices, vertexCount);
		return polygonShape;
	}
	
	/**
	 * Copy vertices. This assumes the vertices define a convex polygon.
	 * It is assumed that the exterior is the the right of each edge.
	 */
	 void SetAsVector(List<b2Vec2> vertices, [int vertexCount = 0])
	{
		if (vertexCount == 0)
			vertexCount = vertices.length;
			
		b2Settings.b2Assert(2 <= vertexCount);
		m_vertexCount = vertexCount;
		
		Reserve(vertexCount);
		
		int i = 0;
		
		// Copy vertices
		for (i = 0; i < m_vertexCount; i++)
		{
			m_vertices[i].SetV(vertices[i]);
		}
		
		// Compute normals. Ensure the edges have non-zero length.
		for (i = 0; i < m_vertexCount; ++i)
		{
			int i1 = i;
			int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			b2Vec2 edge = b2Math.SubtractVV(m_vertices[i2], m_vertices[i1]);
			b2Settings.b2Assert(edge.LengthSquared() > double.MIN_POSITIVE /* * double.MIN_POSITIVE*/);
			m_normals[i].SetV(b2Math.CrossVF(edge, 1.0));
			m_normals[i].Normalize();
		}
		
//#ifdef _DEBUG
		// Ensure the polygon is convex and the interior
		// is to the left of each edge.
		//for (int32 i = 0; i < m_vertexCount; ++i)
		//{
			//int32 i1 = i;
			//int32 i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			//b2Vec2 edge = m_vertices[i2] - m_vertices[i1];
			//for (int32 j = 0; j < m_vertexCount; ++j)
			//{
				// Don't check vertices on the current edge.
				//if (j == i1 || j == i2)
				//{
					//continue;
				//}
				//
				//b2Vec2 r = m_vertices[j] - m_vertices[i1];
				// Your polygon is non-convex (it has an indentation) or
				// has colinear edges.
				//float32 s = b2Cross(edge, r);
				//b2Assert(s > 0.0f);
			//}
		//}
//#endif

		// Compute the polygon centroid
		m_centroid = ComputeCentroid(m_vertices, m_vertexCount);
	}
	
	 static b2PolygonShape AsVector(List<b2Vec2> vertices, int vertexCount)
	{
		b2PolygonShape polygonShape = new b2PolygonShape();
		polygonShape.SetAsVector(vertices, vertexCount);
		return polygonShape;
	}
	
	/**
	* Build vertices to represent an axis-aligned box.
	* @param hx the half-width.
	* @param hy the half-height.
	*/
	  void SetAsBox(double hx,double hy) 
	{
		m_vertexCount = 4;
		Reserve(4);
		m_vertices[0].Set(-hx, -hy);
		m_vertices[1].Set( hx, -hy);
		m_vertices[2].Set( hx,  hy);
		m_vertices[3].Set(-hx,  hy);
		m_normals[0].Set(0.0, -1.0);
		m_normals[1].Set(1.0, 0.0);
		m_normals[2].Set(0.0, 1.0);
		m_normals[3].Set(-1.0, 0.0);
		m_centroid.SetZero();
	}
	
	 static  b2PolygonShape AsBox(double hx,double hy)
	{
		b2PolygonShape polygonShape = new b2PolygonShape();
		polygonShape.SetAsBox(hx, hy);
		return polygonShape;
	}
	
	/**
	* Build vertices to represent an oriented box.
	* @param hx the half-width.
	* @param hy the half-height.
	* @param center the center of the box in local coordinates.
	* @param angle the rotation of the box in local coordinates.
	*/
	static  b2Mat22 s_mat = new b2Mat22();
	  void SetAsOrientedBox(double hx,double hy,[b2Vec2 center=null, double angle=0.0])
	{
		m_vertexCount = 4;
		Reserve(4);
		m_vertices[0].Set(-hx, -hy);
		m_vertices[1].Set( hx, -hy);
		m_vertices[2].Set( hx,  hy);
		m_vertices[3].Set(-hx,  hy);
		m_normals[0].Set(0.0, -1.0);
		m_normals[1].Set(1.0, 0.0);
		m_normals[2].Set(0.0, 1.0);
		m_normals[3].Set(-1.0, 0.0);
		m_centroid = center;

		b2Transform xf = new b2Transform();
		xf.position = center;
		xf.R.Set(angle);

		// Transform vertices and normals.
		for (int i = 0; i < m_vertexCount; ++i)
		{
			m_vertices[i] = b2Math.MulX(xf, m_vertices[i]);
			m_normals[i] = b2Math.MulMV(xf.R, m_normals[i]);
		}
	}
	
	 static  b2PolygonShape AsOrientedBox(double hx,double hy,[b2Vec2 center=null, double angle=0.0])
	{
		b2PolygonShape polygonShape = new b2PolygonShape();
		polygonShape.SetAsOrientedBox(hx, hy, center, angle);
		return polygonShape;
	}
	
	/**
	 * Set this as a single edge.
	 */
	  void SetAsEdge(b2Vec2 v1,b2Vec2 v2)
	{
		m_vertexCount = 2;
		Reserve(2);
		m_vertices[0].SetV(v1);
		m_vertices[1].SetV(v2);
		m_centroid.x = 0.5 * (v1.x + v2.x);
		m_centroid.y = 0.5 * (v1.y + v2.y);
		m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1.0);
		m_normals[0].Normalize();
		m_normals[1].x = -m_normals[0].x;
		m_normals[1].y = -m_normals[0].y;
	}
	
	/**
	 * Set this as a single edge.
	 */
	static   b2PolygonShape AsEdge(b2Vec2 v1,b2Vec2 v2)
	{
		b2PolygonShape polygonShape = new b2PolygonShape();
		polygonShape.SetAsEdge(v1, v2);
		return polygonShape;
	}
	
	
	/**
	* @inheritDoc
	*/
	 @override 
		 bool TestPoint(b2Transform xf,b2Vec2 p){
		b2Vec2 tVec;
		
		//b2Vec2 pLocal = b2MulT(xf.R, p - xf.position);
		b2Mat22 tMat = xf.R;
		double tX = p.x - xf.position.x;
		double tY = p.y - xf.position.y;
		double pLocalX = (tX*tMat.col1.x + tY*tMat.col1.y);
		double pLocalY = (tX*tMat.col2.x + tY*tMat.col2.y);
		
		for (int i = 0; i < m_vertexCount; ++i)
		{
			//float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
			tVec = m_vertices[i];
			tX = pLocalX - tVec.x;
			tY = pLocalY - tVec.y;
			tVec = m_normals[i];
			double dot = (tVec.x * tX + tVec.y * tY);
			if (dot > 0.0)
			{
				return false;
			}
		}
		
		return true;
	}

	/**
	 * @inheritDoc
	 */
	 @override 
		 bool RayCast(b2RayCastOutput output,b2RayCastInput input,b2Transform transform)
	{
		double lower = 0.0;
		double upper = input.maxFraction;
		
		double tX = 0.0;
		double tY = 0.0;
		b2Mat22 tMat;
		b2Vec2 tVec;
		
		// Put the ray into the polygon's frame of reference. (AS3 Port Manual inlining follows)
		//b2Vec2 p1 = b2MulT(transform.R, segment.p1 - transform.position);
		tX = input.p1.x - transform.position.x;
		tY = input.p1.y - transform.position.y;
		tMat = transform.R;
		double p1X = (tX * tMat.col1.x + tY * tMat.col1.y);
		double p1Y = (tX * tMat.col2.x + tY * tMat.col2.y);
		//b2Vec2 p2 = b2MulT(transform.R, segment.p2 - transform.position);
		tX = input.p2.x - transform.position.x;
		tY = input.p2.y - transform.position.y;
		tMat = transform.R;
		double p2X = (tX * tMat.col1.x + tY * tMat.col1.y);
		double p2Y = (tX * tMat.col2.x + tY * tMat.col2.y);
		//b2Vec2 d = p2 - p1;
		double dX = p2X - p1X;
		double dY = p2Y - p1Y;
		int index = -1;
		
		for (int i = 0; i < m_vertexCount; ++i)
		{
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
			
			//float32 doubleerator = b2Dot(m_normals[i], m_vertices[i] - p1);
			tVec = m_vertices[i];
			tX = tVec.x - p1X;
			tY = tVec.y - p1Y;
			tVec = m_normals[i];
			double doubleerator = (tVec.x*tX + tVec.y*tY);
			//float32 denominator = b2Dot(m_normals[i], d);
			double denominator = (tVec.x * dX + tVec.y * dY);
			
			if (denominator == 0.0)
			{
				if (doubleerator < 0.0)
				{
					return false;
				}
			}
			else
			{
				// Note: we want this predicate without division:
				// lower < doubleerator / denominator, where denominator < 0
				// Since denominator < 0, we have to flip the inequality:
				// lower < doubleerator / denominator <==> denominator * lower > doubleerator.
				if (denominator < 0.0 && doubleerator < lower * denominator)
				{
					// Increase lower.
					// The segment enters this half-space.
					lower = doubleerator / denominator;
					index = i;
				}
				else if (denominator > 0.0 && doubleerator < upper * denominator)
				{
					// Decrease upper.
					// The segment exits this half-space.
					upper = doubleerator / denominator;
				}
			}
			
			if (upper < lower - double.MIN_POSITIVE)
			{
				return false;
			}
		}
		
		//b2Settings.b2Assert(0.0 <= lower && lower <= input.maxLambda);
		
		if (index >= 0)
		{
			output.fraction = lower;
			//output.normal = b2Mul(transform.R, m_normals[index]);
			tMat = transform.R;
			tVec = m_normals[index];
			output.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			output.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			return true;
		}
		
		return false;
	}


	/**
	 * @inheritDoc
	 */
	 @override 
		 void ComputeAABB(b2AABB aabb,b2Transform xf)
	{
		//b2Vec2 lower = b2Math.MulX(xf, m_vertices[0]);
		b2Mat22 tMat = xf.R;
		b2Vec2 tVec = m_vertices[0];
		double lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		double lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		double upperX = lowerX;
		double upperY = lowerY;
		
		for (int i = 1; i < m_vertexCount; ++i)
		{
			tVec = m_vertices[i];
			double vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			double vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			lowerX = lowerX < vX ? lowerX : vX;
			lowerY = lowerY < vY ? lowerY : vY;
			upperX = upperX > vX ? upperX : vX;
			upperY = upperY > vY ? upperY : vY;
		}

		aabb.lowerBound.x = lowerX - m_radius;
		aabb.lowerBound.y = lowerY - m_radius;
		aabb.upperBound.x = upperX + m_radius;
		aabb.upperBound.y = upperY + m_radius;
	}


	/**
	* @inheritDoc
	*/
	 @override 
		 void ComputeMass(b2MassData massData,double density){
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho *(dA).toInt()
		// centroid.x = (1/mass) * rho *(x * dA).toInt()
		// centroid.y = (1/mass) * rho *(y * dA).toInt()
		// I = rho *((x*x + y*y) * dA).toInt()
		//
		// We can compute these integrals by summing all the integrals
		// for triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.
		
		//b2Settings.b2Assert(m_vertexCount >= 2);
		
		// A line segment has zero mass.
		if (m_vertexCount == 2)
		{
			massData.center.x = 0.5 * (m_vertices[0].x + m_vertices[1].x);
			massData.center.y = 0.5 * (m_vertices[0].y + m_vertices[1].y);
			massData.mass = 0.0;
			massData.I = 0.0;
			return;
		}
		
		//b2Vec2 center; center.Set(0.0f, 0.0f);
		double centerX = 0.0;
		double centerY = 0.0;
		double area = 0.0;
		double I = 0.0;
		
		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		//b2Vec2 pRef(0.0f, 0.0f);
		double p1X = 0.0;
		double p1Y = 0.0;
		/*#if 0
		// This code would put the reference point inside the polygon.
		for (int32 i = 0; i < m_vertexCount; ++i)
		{
			pRef += m_vertices[i];
		}
		pRef *= 1.0f / count;
		#endif*/
		
		double k_inv3 = 1.0 / 3.0;
		
		for (int i = 0; i < m_vertexCount; ++i)
		{
			// Triangle vertices.
			//b2Vec2 p1 = pRef;
			//
			//b2Vec2 p2 = m_vertices[i];
			b2Vec2 p2 = m_vertices[i];
			//b2Vec2 p3 = i + 1 < m_vertexCount ? m_vertices[i+1] : m_vertices[0];
			b2Vec2 p3 = i + 1 < m_vertexCount ? m_vertices[(i+1).toInt()] : m_vertices[0];
			
			//b2Vec2 e1 = p2 - p1;
			double e1X = p2.x - p1X;
			double e1Y = p2.y - p1Y;
			//b2Vec2 e2 = p3 - p1;
			double e2X = p3.x - p1X;
			double e2Y = p3.y - p1Y;
			
			//float32 D = b2Cross(e1, e2);
			double D = e1X * e2Y - e1Y * e2X;
			
			//float32 triangleArea = 0.5f * D;
			double triangleArea = 0.5 * D;
			area += triangleArea;
			
			// Area weighted centroid
			//center += triangleArea * k_inv3 * (p1 + p2 + p3);
			centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
			centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
			
			//float32 px = p1.x, py = p1.y;
			double px = p1X;
			double py = p1Y;
			//float32 ex1 = e1.x, ey1 = e1.y;
			double ex1 = e1X;
			double ey1 = e1Y;
			//float32 ex2 = e2.x, ey2 = e2.y;
			double ex2 = e2X;
			double ey2 = e2Y;
			
			//float32 intx2 = k_inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
			double intx2 = k_inv3 * (0.25 * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5*px*px;
			//float32 inty2 = k_inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;
			double inty2 = k_inv3 * (0.25 * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5*py*py;
			
			I += D * (intx2 + inty2);
		}
		
		// Total mass
		massData.mass = density * area;
		
		// Center of mass
		//b2Settings.b2Assert(area > double.MIN_POSITIVE);
		//center *= 1.0f / area;
		centerX *= 1.0 / area;
		centerY *= 1.0 / area;
		//massData->center = center;
		massData.center.Set(centerX, centerY);
		
		// Inertia tensor relative to the local origin.
		massData.I = density * I;
	}

	/**
	* @inheritDoc
	*/
	 @override 
		 double ComputeSubmergedArea(b2Vec2 normal,double offset,b2Transform xf,b2Vec2 c)
	{
		// Transform plane into shape co-ordinates
		b2Vec2 normalL = b2Math.MulTMV(xf.R, normal);
		double offsetL = offset - b2Math.Dot(normal, xf.position);
		
		List<double> depths = new List<double>();
		int diveCount = 0;
		int intoIndex = -1;
		int outoIndex = -1;
		
		bool lastSubmerged = false;
		int i = 0;
		for (i = 0; i < m_vertexCount;++i)
		{
			depths.add(b2Math.Dot(normalL, m_vertices[i]) - offsetL);
			bool isSubmerged = depths[i] < -double.MIN_POSITIVE;
			if (i > 0)
			{
				if( isSubmerged)
				{
					if(lastSubmerged == false)
					{
						intoIndex = i - 1;
						diveCount++;
					}
				}
				else
				{
					if( lastSubmerged)
					{
						outoIndex = i - 1;
						diveCount++;
					}
				}
			}
			lastSubmerged = isSubmerged;
		}
		switch(diveCount)
		{
			case 0:
			if( lastSubmerged)
			{
				// Completely submerged
				b2MassData md = new b2MassData();
				ComputeMass(md, 1.0);
				c.SetV(b2Math.MulX(xf, md.center));
				return md.mass;
			}
			else
			{
				//Completely dry
				return 0.0;
			}
			break;
			case 1:
			if (intoIndex == -1)
			{
				intoIndex = m_vertexCount - 1;
			}
			else
			{
				outoIndex = m_vertexCount - 1;
			}
			break;
		}
		int intoIndex2 = (intoIndex + 1) % m_vertexCount;
		int outoIndex2 = (outoIndex + 1) % m_vertexCount;
		double intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
		double outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
		
		b2Vec2 intoVec = new b2Vec2(m_vertices[intoIndex].x * (1 - intoLamdda) + m_vertices[intoIndex2].x * intoLamdda,
										m_vertices[intoIndex].y * (1 - intoLamdda) + m_vertices[intoIndex2].y * intoLamdda);
		b2Vec2 outoVec = new b2Vec2(m_vertices[outoIndex].x * (1 - outoLamdda) + m_vertices[outoIndex2].x * outoLamdda,
										m_vertices[outoIndex].y * (1 - outoLamdda) + m_vertices[outoIndex2].y * outoLamdda);
										
		// Initialize accumulator
		double area = 0.0;
		b2Vec2 center = new b2Vec2();
		b2Vec2 p2 = m_vertices[intoIndex2];
		b2Vec2 p3;
		
		// An awkward loop from intoIndex2+1 to outIndex2
		i = intoIndex2;
		while (i != outoIndex2)
		{
			i = (i + 1) % m_vertexCount;
			if(i == outoIndex2)
				p3 = outoVec;
			else
				p3 = m_vertices[i];
			
			double triangleArea = 0.5 * ( (p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x) );
			area += triangleArea;
			// Area weighted centroid
			center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
			center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
			
			p2 = p3;
		}
		
		//Normalize and transform centroid
		center.Multiply(1 / area);
		c.SetV(b2Math.MulX(xf, center));
		
		return area;
	}
	
	/**
	* Get the vertex count.
	*/
	  int GetVertexCount(){
		return m_vertexCount;
	}

	/**
	* Get the vertices in local coordinates.
	*/
	  List<b2Vec2> GetVertices(){
		return m_vertices;
	}
	
	/**
	* Get the edge normal vectors. There is one for vertex.
	*/
	  List<b2Vec2> GetNormals()
	{
		return m_normals;
	}
	
	/**
	 * Get the supporting vertex index in the given direction.
	 */
	  int GetSupport(b2Vec2 d)
	{
		int bestIndex = 0;
		double bestValue = m_vertices[0].x * d.x + m_vertices[0].y * d.y;
		for (int i= 1; i < m_vertexCount; ++i)
		{
			double value = m_vertices[i].x * d.x + m_vertices[i].y * d.y;
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}
		return bestIndex;
	}
	
	  b2Vec2 GetSupportVertex(b2Vec2 d)
	{
		int bestIndex = 0;
		double bestValue = m_vertices[0].x * d.x + m_vertices[0].y * d.y;
		for (int i= 1; i < m_vertexCount; ++i)
		{
			double value = m_vertices[i].x * d.x + m_vertices[i].y * d.y;
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}
		return m_vertices[bestIndex];
	}

	// TODO: Expose this
	  bool Validate()
	{
		/*
		// Ensure the polygon is convex.
		for (int32 i = 0; i < m_vertexCount; ++i)
		{
			for (int32 j = 0; j < m_vertexCount; ++j)
			{
				// Don't check vertices on the current edge.
				if (j == i || j == (i + 1) % m_vertexCount)
				{
					continue;
				}
				
				// Your polygon is non-convex (it has an indentation).
				// Or your polygon is too skinny.
				float32 s = b2Dot(m_normals[i], m_vertices[j] - m_vertices[i]);
				b2Assert(s < -b2_linearSlop);
			}
		}
		
		// Ensure the polygon is counter-clockwise.
		for (i = 1; i < m_vertexCount; ++i)
		{
			double cross = b2Math.b2CrossVV(m_normals[int(i-1)], m_normals[i]);
			
			// Keep asinf happy.
			cross = b2Math.b2Clamp(cross, -1.0, 1.0);
			
			// You have consecutive edges that are almost parallel on your polygon.
			double angle = Math.asin(cross);
			//b2Assert(angle > b2_angularSlop);
			print(angle > b2Settings.b2_angularSlop);
		}
		*/
		return false;
	}
	//--------------- Internals Below -------------------
	
	/**
	 * @
	 */
	 b2PolygonShape(){
		
		//b2Settings.b2Assert(def.type == e_polygonShape);
		m_type = b2Shape.e_polygonShape;
		
		m_centroid = new b2Vec2();
		m_vertices = new List<b2Vec2>();
		m_normals = new List<b2Vec2>();
	}
	
	  void Reserve(int count)
	{
		for (int i = m_vertices.length; i < count; i++)
		{
			m_vertices.add( new b2Vec2() );
			m_normals.add( new b2Vec2() );
		}
	}

	// Local position of the polygon centroid.
	b2Vec2 m_centroid;

	List<b2Vec2> m_vertices;
	List<b2Vec2> m_normals;
	
	int m_vertexCount = 0;
	
	
	
	/**
	 * Computes the centroid of the given polygon
	 * @param	vs		vector of b2Vec specifying a polygon
	 * @param	count	length of vs
	 * @return the polygon centroid
	 */
	static   b2Vec2 ComputeCentroid(List<b2Vec2> vs,int count)
	{
		//b2Settings.b2Assert(count >= 3);
		
		//b2Vec2 c; c.Set(0.0f, 0.0f);
		b2Vec2 c = new b2Vec2();
		double area = 0.0;
		
		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		//b2Vec2 pRef(0.0f, 0.0f);
		double p1X = 0.0;
		double p1Y = 0.0;
	/*#if 0
		// This code would put the reference point inside the polygon.
		for (int32 i = 0; i < count; ++i)
		{
			pRef += vs[i];
		}
		pRef *= 1.0f / count;
	#endif*/
		
		double inv3 = 1.0 / 3.0;
		
		for (int i = 0; i < count; ++i)
		{
			// Triangle vertices.
			//b2Vec2 p1 = pRef;
				// 0.0, 0.0
			//b2Vec2 p2 = vs[i];
			b2Vec2 p2 = vs[i];
			//b2Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];
			b2Vec2 p3 = i + 1 < count ? vs[(i+1).toInt()] : vs[0];
			
			//b2Vec2 e1 = p2 - p1;
			double e1X = p2.x - p1X;
			double e1Y = p2.y - p1Y;
			//b2Vec2 e2 = p3 - p1;
			double e2X = p3.x - p1X;
			double e2Y = p3.y - p1Y;
			
			//float32 D = b2Cross(e1, e2);
			double D = (e1X * e2Y - e1Y * e2X);
			
			//float32 triangleArea = 0.5f * D;
			double triangleArea = 0.5 * D;
			area += triangleArea;
			
			// Area weighted centroid
			//c += triangleArea * inv3 * (p1 + p2 + p3);
			c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
			c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
		}
		
		// Centroid
		//beSettings.b2Assert(area > double.MIN_POSITIVE);
		//c *= 1.0 / area;
		c.x *= 1.0 / area;
		c.y *= 1.0 / area;
		return c;
	}

	/**
	 * Computes a polygon's OBB
	 * @see http://www.geometrictools.com/Documentation/MinimumAreaRectangle.pdf
	 */
	static  void ComputeOBB(b2OBB obb,List<b2Vec2> vs,int count)
	{
		int i = 0;
		List<b2Vec2> p = new List<b2Vec2>(count + 1);
		for (i = 0; i < count; ++i)
		{
			p[i] = vs[i];
		}
		p[count] = p[0];
		
		double minArea = double.MAX_FINITE;
		
		for (i = 1; i <= count; ++i)
		{
			b2Vec2 root = p[(i-1).toInt()];
			//b2Vec2 ux = p[i] - root;
			double uxX = p[i].x - root.x;
			double uxY = p[i].y - root.y;
			//double length = ux.Normalize();
			double length = sqrt(uxX*uxX + uxY*uxY);
			uxX /= length;
			uxY /= length;
			//b2Settings.b2Assert(length > double.MIN_POSITIVE);
			//b2Vec2 uy(-ux.y, ux.x);
			double uyX = -uxY;
			double uyY = uxX;
			//b2Vec2 lower(FLT_MAX, FLT_MAX);
			double lowerX = double.MAX_FINITE;
			double lowerY = double.MAX_FINITE;
			//b2Vec2 upper(-FLT_MAX, -FLT_MAX);
			double upperX = -double.MAX_FINITE;
			double upperY = -double.MAX_FINITE;
			
			for (int j = 0; j < count; ++j)
			{
				//b2Vec2 d = p[j] - root;
				double dX = p[j].x - root.x;
				double dY = p[j].y - root.y;
				//b2Vec2 r;
				//double rX = b2Dot(ux, d);
				double rX = (uxX*dX + uxY*dY);
				//double rY = b2Dot(uy, d);
				double rY = (uyX*dX + uyY*dY);
				//lower = b2Min(lower, r);
				if (rX < lowerX) lowerX = rX;
				if (rY < lowerY) lowerY = rY;
				//upper = b2Max(upper, r);
				if (rX > upperX) upperX = rX;
				if (rY > upperY) upperY = rY;
			}
			
			double area = (upperX - lowerX) * (upperY - lowerY);
			if (area < 0.95 * minArea)
			{
				minArea = area;
				//obb->R.col1 = ux;
				obb.R.col1.x = uxX;
				obb.R.col1.y = uxY;
				//obb->R.col2 = uy;
				obb.R.col2.x = uyX;
				obb.R.col2.y = uyY;
				//b2Vec2 center = 0.5f * (lower + upper);
				double centerX = 0.5 * (lowerX + upperX);
				double centerY = 0.5 * (lowerY + upperY);
				//obb->center = root + b2Mul(obb->R, center);
				b2Mat22 tMat = obb.R;
				obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
				obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
				//obb->extents = 0.5f * (upper - lower);
				obb.extents.x = 0.5 * (upperX - lowerX);
				obb.extents.y = 0.5 * (upperY - lowerY);
			}
		}
		
		//b2Settings.b2Assert(minArea < double.MAX_FINITE);
	}
	
	
}

