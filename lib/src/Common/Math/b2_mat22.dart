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
* A 2-by-2 matrix. Stored in column-major order.
*/
 class b2Mat22
{
	 b2Mat22()
	{
		col1.x = col2.y = 1.0;
	}
	
	 static  b2Mat22 FromAngle(double angle)
	{
		b2Mat22 mat = new b2Mat22();
		mat.Set(angle);
		return mat;
	}
	
	 static  b2Mat22 FromVV(b2Vec2 c1,b2Vec2 c2)
	{
		b2Mat22 mat = new b2Mat22();
		mat.SetVV(c1, c2);
		return mat;
	}

	  void Set(double angle)
	{
		double c = /*Math.*/cos(angle);
		double s = /*Math.*/sin(angle);
		col1.x = c; col2.x = -s;
		col1.y = s; col2.y = c;
	}
	
	  void SetVV(b2Vec2 c1,b2Vec2 c2)
	{
		col1.SetV(c1);
		col2.SetV(c2);
	}
	
	  b2Mat22 Copy(){
		b2Mat22 mat = new b2Mat22();
		mat.SetM(this);
		return mat;
	}
	
	  void SetM(b2Mat22 m)
	{
		col1.SetV(m.col1);
		col2.SetV(m.col2);
	}
	
	  void AddM(b2Mat22 m)
	{
		col1.x += m.col1.x;
		col1.y += m.col1.y;
		col2.x += m.col2.x;
		col2.y += m.col2.y;
	}
	
	  void SetIdentity()
	{
		col1.x = 1.0; col2.x = 0.0;
		col1.y = 0.0; col2.y = 1.0;
	}

	  void SetZero()
	{
		col1.x = 0.0; col2.x = 0.0;
		col1.y = 0.0; col2.y = 0.0;
	}
	
	  double GetAngle()
	{
		return atan2(col1.y, col1.x);
	}

	/**
	 * Compute the inverse of this matrix, such that inv(A) * A = identity.
	 */
	  b2Mat22 GetInverse(b2Mat22 out)
	{
		double a = col1.x; 
		double b = col2.x; 
		double c = col1.y; 
		double d = col2.y;
		//b2Mat22 B = new b2Mat22();
		double det = a * d - b * c;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		out.col1.x =  det * d;	out.col2.x = -det * b;
		out.col1.y = -det * c;	out.col2.y =  det * a;
		return out;
	}
	
	// Solve A * x = b
	  b2Vec2 Solve(b2Vec2 out,double bX,double bY)
	{
		//float32 a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
		double a11 = col1.x;
		double a12 = col2.x;
		double a21 = col1.y;
		double a22 = col2.y;
		//float32 det = a11 * a22 - a12 * a21;
		double det = a11 * a22 - a12 * a21;
		if (det != 0.0)
		{
			det = 1.0 / det;
		}
		out.x = det * (a22 * bX - a12 * bY);
		out.y = det * (a11 * bY - a21 * bX);
		
		return out;
	}
	
	  void Abs()
	{
		col1.Abs();
		col2.Abs();
	}

	 b2Vec2 col1 = new b2Vec2();
	 b2Vec2 col2 = new b2Vec2();
}

