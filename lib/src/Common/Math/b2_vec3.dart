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
* A 2D column vector with 3 elements.
*/

 class b2Vec3
{
	/**
	 * Construct using co-ordinates
	 */
	 b2Vec3([double x=0.0, double y=0.0, double z=0.0])
	{
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	/**
	 * Sets this vector to all zeros
	 */
	  void SetZero()
	{
		x = y = z = 0.0;
	}
	
	/**
	 * Set this vector to some specified coordinates.
	 */
	  void Set(double x,double y,double z)
	{
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	  void SetV(b2Vec3 v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}
	
	/**
	 * Negate this vector
	 */
	  b2Vec3 GetNegative() { return new b2Vec3( -x, -y, -z); }
	
	  void NegativeSelf() { x = -x; y = -y; z = -z; }
	
	  b2Vec3 Copy(){
		return new b2Vec3(x,y,z);
	}
	
	  void Add(b2Vec3 v)
	{
		x += v.x; y += v.y; z += v.z;
	}
	
	  void Subtract(b2Vec3 v)
	{
		x -= v.x; y -= v.y; z -= v.z;
	}

	  void Multiply(double a)
	{
		x *= a; y *= a; z *= a;
	}
	
	 double x;
	 double y;
	 double z;
	
}
