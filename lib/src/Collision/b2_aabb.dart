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
* An axis aligned bounding box.
*/
 class b2AABB
{
	/**
	* Verify that the bounds are sorted.
	*/
	  bool IsValid(){
		//b2Vec2 d = upperBound - lowerBound;;
		double dX = upperBound.x - lowerBound.x;
		double dY = upperBound.y - lowerBound.y;
		bool valid = dX >= 0.0 && dY >= 0.0;
		valid = valid && lowerBound.IsValid() && upperBound.IsValid();
		return valid;
	}
	
	/** Get the center of the AABB. */
	  b2Vec2 GetCenter()
	{
		return new b2Vec2( (lowerBound.x + upperBound.x) / 2,
		                   (lowerBound.y + upperBound.y) / 2);
	}
	
	/** Get the extents of the AABB (half-widths). */
	  b2Vec2 GetExtents()
	{
		return new b2Vec2( (upperBound.x - lowerBound.x) / 2,
		                   (upperBound.y - lowerBound.y) / 2);
	}
	
	/**
	 * Is an AABB contained within this one.
	 */
	  bool Contains(b2AABB aabb)
	{
		bool result = true;
		result = lowerBound.x <= aabb.lowerBound.x;
		result = lowerBound.y <= aabb.lowerBound.y;
		result = aabb.upperBound.x <= upperBound.x;
		result = aabb.upperBound.y <= upperBound.y;
		return result;
	}
	
	// From Real-time Collision Detection, p179.
	/**
	 * Perform a precise raycast against the AABB.
	 */
	  bool RayCast(b2RayCastOutput output,b2RayCastInput input)
	{
		double tmin = -double.MAX_FINITE;
		double tmax = double.MAX_FINITE;
		
		double pX = input.p1.x;
		double pY = input.p1.y;
		double dX = input.p2.x - input.p1.x;
		double dY = input.p2.y - input.p1.y;
		double absDX = (dX).abs();
		double absDY = (dY).abs();
		
		b2Vec2 normal = output.normal;
		
		double inv_d = 0.0;
		double t1 = 0.0;
		double t2 = 0.0;
		double t3 = 0.0;
		double s = 0.0;
		
		//x
		{
			if (absDX < double.MIN_POSITIVE)
			{
				// Parallel.
				if (pX < lowerBound.x || upperBound.x < pX)
					return false;
			}
			else
			{
				inv_d = 1.0 / dX;
				t1 = (lowerBound.x - pX) * inv_d;
				t2 = (upperBound.x - pX) * inv_d;
				
				// Sign of the normal vector
				s = -1.0;
				
				if (t1 > t2)
				{
					t3 = t1;
					t1 = t2;
					t2 = t3;
					s = 1.0;
				}
				
				// Push the min up
				if (t1 > tmin)
				{
					normal.x = s;
					normal.y = 0.0;
					tmin = t1;
				}
				
				// Pull the max down
				tmax = /*Math.*/min(tmax, t2);
				
				if (tmin > tmax)
					return false;
			}
		}
		//y
		{
			if (absDY < double.MIN_POSITIVE)
			{
				// Parallel.
				if (pY < lowerBound.y || upperBound.y < pY)
					return false;
			}
			else
			{
				inv_d = 1.0 / dY;
				t1 = (lowerBound.y - pY) * inv_d;
				t2 = (upperBound.y - pY) * inv_d;
				
				// Sign of the normal vector
				s = -1.0;
				
				if (t1 > t2)
				{
					t3 = t1;
					t1 = t2;
					t2 = t3;
					s = 1.0;
				}
				
				// Push the min up
				if (t1 > tmin)
				{
					normal.y = s;
					normal.x = 0.0;
					tmin = t1;
				}
				
				// Pull the max down
				tmax = /*Math.*/min(tmax, t2);
				
				if (tmin > tmax)
					return false;
			}
		}
		
		output.fraction = tmin;
		return true;
	}
	
	/**
	 * Tests if another AABB overlaps this one.
	 */
	  bool TestOverlap(b2AABB other)
	{
		double d1X = other.lowerBound.x - upperBound.x;
		double d1Y = other.lowerBound.y - upperBound.y;
		double d2X = lowerBound.x - other.upperBound.x;
		double d2Y = lowerBound.y - other.upperBound.y;

		if (d1X > 0.0 || d1Y > 0.0)
			return false;

		if (d2X > 0.0 || d2Y > 0.0)
			return false;

		return true;
	}
	
	/** Combine two AABBs into one. */
	 static  b2AABB Combine(b2AABB aabb1,b2AABB aabb2)
	{
		b2AABB aabb = new b2AABB();
		aabb.CombineI(aabb1, aabb2);
		return aabb;
	}
	
	/** Combine two AABBs into one. */
	  void CombineI(b2AABB aabb1,b2AABB aabb2)
	{
		lowerBound.x = /*Math.*/min(aabb1.lowerBound.x, aabb2.lowerBound.x);
		lowerBound.y = /*Math.*/min(aabb1.lowerBound.y, aabb2.lowerBound.y);
		upperBound.x = /*Math.*/max(aabb1.upperBound.x, aabb2.upperBound.x);
		upperBound.y = /*Math.*/max(aabb1.upperBound.y, aabb2.upperBound.y);
	}

	/** The lower vertex */
	 b2Vec2 lowerBound = new b2Vec2();
	/** The upper vertex */
	 b2Vec2 upperBound = new b2Vec2();
}


