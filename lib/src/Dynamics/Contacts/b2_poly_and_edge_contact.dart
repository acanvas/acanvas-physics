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
* @
*/
 class b2PolyAndEdgeContact extends b2Contact{
	
	static   b2Contact Create(dynamic allocator){
		return new b2PolyAndEdgeContact();
	}
	static   void Destroy(b2Contact contact,dynamic allocator){
	}

	  void Reset([b2Fixture fixtureA=null, b2Fixture fixtureB=null]){
		super.Reset(fixtureA, fixtureB);
		b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
		b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_edgeShape);
	}
	//~b2PolyAndEdgeContact() {}

	@override 
		 void Evaluate(){
		b2Body bA = m_fixtureA.GetBody();
		b2Body bB = m_fixtureB.GetBody();
		
		b2CollidePolyAndEdge(m_manifold,
					m_fixtureA.GetShape() as b2PolygonShape, bA.m_xf, 
					m_fixtureB.GetShape() as b2EdgeShape, bB.m_xf);
	}
	
	  void b2CollidePolyAndEdge(b2Manifold manifold,b2PolygonShape polygon,b2Transform xf1,b2EdgeShape edge,b2Transform xf2)
	{
		//TODO_BORIS
		/*
		manifold.pointCount = 0;
		b2Mat22 tMat;
		b2Vec2 tVec1;
		b2Vec2 tVec2;
		double tX = 0.0;
		double tY = 0.0;
		b2ManifoldPoint tPoint = 0;
		double ratio = 0.0;
		
		//b2Vec2 v1 = b2Mul(xf2, edge->GetVertex1());
		tMat = xf2.R;
		tVec1 = edge.m_v1;
		double v1X = xf2.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
		double v1Y = xf2.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
		
		//b2Vec2 v2 = b2Mul(xf2, edge->GetVertex2());
		tVec1 = edge.m_v2;
		double v2X = xf2.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
		double v2Y = xf2.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
		
		//b2Vec2 n = b2Mul(xf2.R, edge->GetNormalList));
		tVec1 = edge.m_normal;
		double nX = (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
		double nY = (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
		
		//b2Vec2 v1Local = b2MulT(xf1, v1);
		tMat = xf1.R;
		tX = v1X - xf1.position.x;
		tY = v1Y - xf1.position.y;
		double v1LocalX = (tX * tMat.col1.x + tY * tMat.col1.y );
		double v1LocalY = (tX * tMat.col2.x + tY * tMat.col2.y );
		
		//b2Vec2 v2Local = b2MulT(xf1, v2);
		tX = v2X - xf1.position.x;
		tY = v2Y - xf1.position.y;
		double v2LocalX = (tX * tMat.col1.x + tY * tMat.col1.y );
		double v2LocalY = (tX * tMat.col2.x + tY * tMat.col2.y );
		
		//b2Vec2 nLocal = b2MulT(xf1.R, n);
		double nLocalX = (nX * tMat.col1.x + nY * tMat.col1.y );
		double nLocalY = (nX * tMat.col2.x + nY * tMat.col2.y );
		
		double separation1 = 0.0;
		int separationIndex1 = -1; // which normal on the poly found the shallowest depth?
		double separationMax1 = -double.MAX_VALUE; // the shallowest depth of edge in poly
		double separation2 = 0.0;
		int separationIndex2 = -1; // which normal on the poly found the shallowest depth?
		double separationMax2 = -double.MAX_VALUE; // the shallowest depth of edge in poly
		double separationMax = -double.MAX_VALUE; // the shallowest depth of edge in poly
		bool separationV1 = false; // is the shallowest depth from edge's v1 or v2 vertex?
		int separationIndex = -1; // which normal on the poly found the shallowest depth?
		
		int vertexCount = polygon.m_vertexCount;
		List vertices = polygon.m_vertices;
		List normals = polygon.m_normals;
		
		int enterStartIndex = -1; // the last poly vertex above the edge
		int enterEndIndex = -1; // the first poly vertex below the edge
		int exitStartIndex = -1; // the last poly vertex below the edge
		int exitEndIndex = -1; // the first poly vertex above the edge
		
		// the "N" in the following variables refers to the edge's normal. 
		// these are projections of poly vertices along the edge's normal, 
		// a.k.a. they are the separation of the poly from the edge. 
		double prevSepN = 0.0;
		double nextSepN = 0.0;
		double enterSepN = 0.0; // the depth of enterEndIndex under the edge (stored as a separation, so it's negative)
		double exitSepN = 0.0; // the depth of exitStartIndex under the edge (stored as a separation, so it's negative)
		double deepestSepN = double.MAX_VALUE; // the depth of the deepest poly vertex under the end (stored as a separation, so it's negative)
		
		
		// for poly normal, get the edge's depth into the poly. 
		// for poly vertex, get the vertex's depth into the edge. 
		// use these calculations to define the remaining variables declared above.
		tVec1 = vertices[vertexCount-1];
		prevSepN = (tVec1.x - v1LocalX) * nLocalX + (tVec1.y - v1LocalY) * nLocalY;
		for (int i = 0; i < vertexCount; i++)
		{
			tVec1 = vertices[i];
			tVec2 = normals[i];
			separation1 = (v1LocalX - tVec1.x) * tVec2.x + (v1LocalY - tVec1.y) * tVec2.y;
			separation2 = (v2LocalX - tVec1.x) * tVec2.x + (v2LocalY - tVec1.y) * tVec2.y;
			if (separation2 < separation1) {
				if (separation2 > separationMax) {
					separationMax = separation2;
					separationV1 = false;
					separationIndex = i;
				}
			} else {
				if (separation1 > separationMax) {
					separationMax = separation1;
					separationV1 = true;
					separationIndex = i;
				}
			}
			if (separation1 > separationMax1) {
				separationMax1 = separation1;
				separationIndex1 = i;
			}
			if (separation2 > separationMax2) {
				separationMax2 = separation2;
				separationIndex2 = i;
			}
			
			nextSepN = (tVec1.x - v1LocalX) * nLocalX + (tVec1.y - v1LocalY) * nLocalY;
			if (nextSepN >= 0.0 && prevSepN < 0.0) {
				exitStartIndex = (i == 0) ? vertexCount-1 : i-1;
				exitEndIndex = i;
				exitSepN = prevSepN;
			} else if (nextSepN < 0.0 && prevSepN >= 0.0) {
				enterStartIndex = (i == 0) ? vertexCount-1 : i-1;
				enterEndIndex = i;
				enterSepN = nextSepN;
			}
			if (nextSepN < deepestSepN) {
				deepestSepN = nextSepN;
			}
			prevSepN = nextSepN;
		}
		
		if (enterStartIndex == -1) {
			// poly is entirely below or entirely above edge, return with no contact:
			return;
		}
		if (separationMax > 0.0) {
			// poly is laterally disjoint with edge, return with no contact:
			return;
		}
		
		// if the poly is near a convex corner on the edge
		if ((separationV1 && edge.m_cornerConvex1) || (!separationV1 && edge.m_cornerConvex2)) {
			// if shallowest depth was from edge into poly, 
			// use the edge's vertex as the contact point:
			if (separationMax > deepestSepN + b2Settings.b2_linearSlop) {
				// if -normal angle is closer to adjacent edge than this edge, 
				// let the adjacent edge handle it and return with no contact:
				if( separationV1 != null || separationV1 == true) {
					tMat = xf2.R;
					tVec1 = edge.m_cornerDir1;
					tX = (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
					tY = (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
					tMat = xf1.R;
					v1X = (tMat.col1.x * tX + tMat.col2.x * tY); // note abuse of v1... 
					v1Y = (tMat.col1.y * tX + tMat.col2.y * tY);
					tVec2 = normals[separationIndex1];
					if (tVec2.x * v1X + tVec2.y * v1Y >= 0.0) {
						return;
					}
				} else {
					tMat = xf2.R;
					tVec1 = edge.m_cornerDir2;
					tX = (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y);
					tY = (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y);
					tMat = xf1.R;
					v1X = (tMat.col1.x * tX + tMat.col2.x * tY); // note abuse of v1... 
					v1Y = (tMat.col1.y * tX + tMat.col2.y * tY);
					tVec2 = normals[separationIndex2];
					if (tVec2.x * v1X + tVec2.y * v1Y <= 0.0) {
						return;
					}
				}
				
				tPoint = manifold.points[0];
				manifold.pointCount = 1;
				
				//manifold->normal = b2Mul(xf1.R, normals[separationIndex]);
				tMat = xf1.R;
				tVec2 = normals[separationIndex];
				manifold.normal.x = (tMat.col1.x * tVec2.x + tMat.col2.x * tVec2.y);
				manifold.normal.y = (tMat.col1.y * tVec2.x + tMat.col2.y * tVec2.y);
				
				tPoint.separation = separationMax;
				tPoint.id.features.incidentEdge = separationIndex;
				tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
				tPoint.id.features.referenceEdge = 0;
				tPoint.id.features.flip = 0;
				if( separationV1 != null || separationV1 == true) {
					tPoint.localPoint1.x = v1LocalX;
					tPoint.localPoint1.y = v1LocalY;
					tPoint.localPoint2.x = edge.m_v1.x;
					tPoint.localPoint2.y = edge.m_v1.y;
				} else {
					tPoint.localPoint1.x = v2LocalX;
					tPoint.localPoint1.y = v2LocalY;
					tPoint.localPoint2.x = edge.m_v2.x;
					tPoint.localPoint2.y = edge.m_v2.y;
				}
				return;
			}
		}
		
		// We're going to use the edge's normal now.
		manifold.normal.x = -nX;
		manifold.normal.y = -nY;
		
		// Check whether we only need one contact point.
		if (enterEndIndex == exitStartIndex) {
			tPoint = manifold.points[0];
			manifold.pointCount = 1;
			tPoint.id.features.incidentEdge = enterEndIndex;
			tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
			tPoint.id.features.referenceEdge = 0;
			tPoint.id.features.flip = 0;
			tVec1 = vertices[enterEndIndex];
			tPoint.localPoint1.x = tVec1.x;
			tPoint.localPoint1.y = tVec1.y;
			
			tMat = xf1.R;
			tX = xf1.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y) - xf2.position.x;
			tY = xf1.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y) - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y );
			tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y );
			
			tPoint.separation = enterSepN;
			return;
		}
			
		manifold.pointCount = 2;
		
		// the edge's direction vector, but in the frame of the polygon:
		tX = -nLocalY;
		tY = nLocalX;
		
		tVec1 = vertices[enterEndIndex];
		double dirProj1 = tX * (tVec1.x - v1LocalX) + tY * (tVec1.y - v1LocalY);
		double dirProj2 = 0.0;
		
		// The contact resolution is more robust if the two manifold points are 
		// adjacent to each other on the polygon. So pick the first two poly
		// vertices that are under the edge:
		exitEndIndex = (enterEndIndex == vertexCount - 1) ? 0 : enterEndIndex + 1;
		tVec1 = vertices[exitStartIndex];
		if (exitEndIndex != exitStartIndex) {
			exitStartIndex = exitEndIndex;
			
			exitSepN = nLocalX * (tVec1.x - v1LocalX) + nLocalY * (tVec1.y - v1LocalY);
		}
		dirProj2 = tX * (tVec1.x - v1LocalX) + tY * (tVec1.y - v1LocalY);
		
		tPoint = manifold.points[0];
		tPoint.id.features.incidentEdge = enterEndIndex;
		tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
		tPoint.id.features.referenceEdge = 0;
		tPoint.id.features.flip = 0;
		
		if (dirProj1 > edge.m_length) {
			tPoint.localPoint1.x = v2LocalX;
			tPoint.localPoint1.y = v2LocalY;
			tPoint.localPoint2.x = edge.m_v2.x;
			tPoint.localPoint2.y = edge.m_v2.y;
			ratio = (edge.m_length - dirProj2) / (dirProj1 - dirProj2);
			if (ratio > 100.0 * double.MIN_VALUE && ratio < 1.0) {
				tPoint.separation = exitSepN * (1.0 - ratio) + enterSepN * ratio;
			} else {
				tPoint.separation = enterSepN;
			}
		} else {
			tVec1 = vertices[enterEndIndex];
			tPoint.localPoint1.x = tVec1.x;
			tPoint.localPoint1.y = tVec1.y;
			
			tMat = xf1.R;
			tX = xf1.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y) - xf2.position.x;
			tY = xf1.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y) - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y);
			tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y);
			
			tPoint.separation = enterSepN;
		}
		
		tPoint = manifold.points[1];
		tPoint.id.features.incidentEdge = exitStartIndex;
		tPoint.id.features.incidentVertex = b2Collision.b2_nullFeature;
		tPoint.id.features.referenceEdge = 0;
		tPoint.id.features.flip = 0;
		
		if (dirProj2 < 0.0) {
			tPoint.localPoint1.x = v1LocalX;
			tPoint.localPoint1.y = v1LocalY;
			tPoint.localPoint2.x = edge.m_v1.x;
			tPoint.localPoint2.y = edge.m_v1.y;
			ratio = (-dirProj1) / (dirProj2 - dirProj1);
			if (ratio > 100.0 * double.MIN_VALUE && ratio < 1.0) {
				tPoint.separation = enterSepN * (1.0 - ratio) + exitSepN * ratio;
			} else {
				tPoint.separation = exitSepN;
			}
		} else {
			tVec1 = vertices[exitStartIndex];
			tPoint.localPoint1.x = tVec1.x;
			tPoint.localPoint1.y = tVec1.y;
			
			tMat = xf1.R;
			tX = xf1.position.x + (tMat.col1.x * tVec1.x + tMat.col2.x * tVec1.y) - xf2.position.x;
			tY = xf1.position.y + (tMat.col1.y * tVec1.x + tMat.col2.y * tVec1.y) - xf2.position.y;
			tMat = xf2.R;
			tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y);
			tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y);
			
			tPoint.separation = exitSepN;
		}
		*/
	}
}

