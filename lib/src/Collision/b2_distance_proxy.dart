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
	 * A distance proxy is used by the GJK algorithm.
 	 * It encapsulates any shape.
 	 */
	 class b2DistanceProxy 
	{
 		/**
 		 * Initialize the proxy using the given shape. The shape
 		 * must remain in scope while the proxy is in use.
 		 */
 		  void Set(b2Shape shape)
		{
			switch(shape.GetType())
			{
				case b2Shape.e_circleShape:
				{
					b2CircleShape circle = shape as b2CircleShape;
					m_vertices = new List<b2Vec2>(1/*, true*/);
					m_vertices[0] = circle.m_p;
					m_count = 1;
					m_radius = circle.m_radius;
				}
				break;
				case b2Shape.e_polygonShape:
				{
					b2PolygonShape polygon =  shape as b2PolygonShape;
					m_vertices = polygon.m_vertices;
					m_count = polygon.m_vertexCount;
					m_radius = polygon.m_radius;
				}
				break;
				default:
				b2Settings.b2Assert(false);
			}
		}
		
 		/**
 		 * Get the supporting vertex index in the given direction.
 		 */
 		  int GetSupport(b2Vec2 d)
		{
			int bestIndex = 0;
			double bestValue = m_vertices[0].x * d.x + m_vertices[0].y * d.y;
			for (int i= 1; i < m_count; ++i)
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
		
 		/**
 		 * Get the supporting vertex in the given direction.
 		 */
 		  b2Vec2 GetSupportVertex(b2Vec2 d)
		{
			int bestIndex = 0;
			double bestValue = m_vertices[0].x * d.x + m_vertices[0].y * d.y;
			for (int i= 1; i < m_count; ++i)
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
 		/**
 		 * Get the vertex count.
 		 */
 		  int GetVertexCount()
		{
			return m_count;
		}
		
 		/**
 		 * Get a vertex by index. Used by b2Distance.
 		 */
 		  b2Vec2 GetVertex(int index)
		{
			b2Settings.b2Assert(0 <= index && index < m_count);
			return m_vertices[index];
		}
		
 		 List<b2Vec2> m_vertices;
 		 int m_count = 0;
 		 double m_radius = 0.0;
	}
	
