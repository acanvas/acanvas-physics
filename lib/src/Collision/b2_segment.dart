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
	



/**
* A line in space between two given vertices.
*/
 class b2Segment
{
	/**
	* Ray cast against this segment with another segment
	* @param xf the shape world transform.
	* @param lambda returns the hit fraction. You can use this to compute the contact point
	* p = (1 - lambda) * segment.p1 + lambda * segment.p2.
	* @param normal returns the normal at the contact point. If there is no intersection, the normal
	* is not set.
	* @param segment defines the begin and end point of the ray cast.
	* @param maxLambda a doubleber typically in the range [0,1].
	* @return true if there was an intersection.
	* @see Box2D.Collision.Shapes.b2Shape#TestSegment
	*/
	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.4.1
	// x = mu1 * p1 + mu2 * p2
	// mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
	// mu1 = 1 - mu2;
	// x = (1 - mu2) * p1 + mu2 * p2
	//   = p1 + mu2 * (p2 - p1)
	// x = s + a * r (s := start, r := end - start)
	// s + a * r = p1 + mu2 * d (d := p2 - p1)
	// -a * r + mu2 * d = b (b := s - p1)
	// [-r d] * [a; mu2] = b
	// Cramer's rule:
	// denom = det[-r d]
	// a = det[b d] / denom
	// mu2 = det[-r b] / denom
	  bool TestSegment(List lambda, // float pointer
	                   b2Vec2 normal, // pointer
	                   b2Segment segment, 
	                   double maxLambda){
		//b2Vec2 s = segment.p1;
		b2Vec2 s = segment.p1;
		//b2Vec2 r = segment.p2 - s;
		double rX = segment.p2.x - s.x;
		double rY = segment.p2.y - s.y;
		//b2Vec2 d = p2 - p1;
		double dX = p2.x - p1.x;
		double dY = p2.y - p1.y;
		//b2Vec2 n = b2Cross(d, 1.0f);
		double nX = dY;
		double nY = -dX;
		
		double k_slop = 100.0 * double.MIN_POSITIVE;
		//double denom = -b2Dot(r, n);
		double denom = -(rX*nX + rY*nY);
		
		// Cull back facing collision and ignore parallel segments.
		if (denom > k_slop)
		{
			// Does the segment intersect the infinite line associated with this segment?
			//b2Vec2 b = s - p1;
			double bX = s.x - p1.x;
			double bY = s.y - p1.y;
			//double a = b2Dot(b, n);
			double a = (bX*nX + bY*nY);
			
			if (0.0 <= a && a <= maxLambda * denom)
			{
				double mu2 = -rX * bY + rY * bX;
				
				// Does the segment intersect this segment?
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop))
				{
					a /= denom;
					//n.Normalize();
					double nLen = sqrt(nX*nX + nY*nY);
					nX /= nLen;
					nY /= nLen;
					//*lambda = a;
					lambda[0] = a;
					//*normal = n;
					normal.Set(nX, nY);
					return true;
				}
			}
		}
		
		return false;
	}
	
	/**
	* Extends or clips the segment so that it's ends lie on the boundary of the AABB
	*/
	  void Extend(b2AABB aabb){
		ExtendForward(aabb);
		ExtendBackward(aabb);
	}
	
	/**
	* @see Extend
	*/
	  void ExtendForward(b2AABB aabb){
		double dX = p2.x-p1.x;
		double dY = p2.y-p1.y;
		
		double lambda = /*Math.*/min(	dX>0?(aabb.upperBound.x-p1.x)/dX: dX<0?(aabb.lowerBound.x-p1.x)/dX:double.INFINITY,
										dY>0?(aabb.upperBound.y-p1.y)/dY: dY<0?(aabb.lowerBound.y-p1.y)/dY:double.INFINITY);
		
		p2.x = p1.x + dX * lambda;
		p2.y = p1.y + dY * lambda;
		
	}
	
	/**
	* @see Extend
	*/
	  void ExtendBackward(b2AABB aabb){
		double dX = -p2.x+p1.x;
		double dY = -p2.y+p1.y;
		
		double lambda = /*Math.*/min(	dX>0?(aabb.upperBound.x-p2.x)/dX: dX<0?(aabb.lowerBound.x-p2.x)/dX:double.INFINITY,
										dY>0?(aabb.upperBound.y-p2.y)/dY: dY<0?(aabb.lowerBound.y-p2.y)/dY:double.INFINITY);
		
		p1.x = p2.x + dX * lambda;
		p1.y = p2.y + dY * lambda;
		
	}
	
	/** The starting point */
	 b2Vec2 p1 = new b2Vec2();
	/** The ending point */
	 b2Vec2 p2 = new b2Vec2();
}


