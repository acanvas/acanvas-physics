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
* @
*/
 class b2Math{

	/**
	* This function is used to ensure that a floating point doubleber is
	* not a NaN or infinity.
	*/
	static   bool IsValid(double x)
	{
		return x.isFinite;
	}
	
	/*static   double b2InvSqrt(double x){
		union
		{
			float32 x;
			int32 i;
		} convert;
		
		convert.x = x;
		float32 xhalf = 0.5f * x;
		convert.i = 0x5f3759df - (convert.i >> 1);
		x = convert.x;
		x = x * (1.5f - xhalf * x * x);
		return x;
	}*/

	static   double Dot(b2Vec2 a,b2Vec2 b)
	{
		return a.x * b.x + a.y * b.y;
	}

	static   double CrossVV(b2Vec2 a,b2Vec2 b)
	{
		return a.x * b.y - a.y * b.x;
	}

	static   b2Vec2 CrossVF(b2Vec2 a,double s)
	{
		b2Vec2 v = new b2Vec2(s * a.y, -s * a.x);
		return v;
	}

	static   b2Vec2 CrossFV(double s,b2Vec2 a)
	{
		b2Vec2 v = new b2Vec2(-s * a.y, s * a.x);
		return v;
	}

	static   b2Vec2 MulMV(b2Mat22 A,b2Vec2 v)
	{
		// (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
		// (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
		b2Vec2 u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
		return u;
	}

	static   b2Vec2 MulTMV(b2Mat22 A,b2Vec2 v)
	{
		// (tVec.x * tMat.col1.x + tVec.y * tMat.col1.y)
		// (tVec.x * tMat.col2.x + tVec.y * tMat.col2.y)
		b2Vec2 u = new b2Vec2(Dot(v, A.col1), Dot(v, A.col2));
		return u;
	}
	
	static   b2Vec2 MulX(b2Transform T,b2Vec2 v)
	{
		b2Vec2 a = MulMV(T.R, v);
		a.x += T.position.x;
		a.y += T.position.y;
		//return T.position + b2Mul(T.R, v);
		return a;
	}

	static   b2Vec2 MulXT(b2Transform T,b2Vec2 v)
	{
		b2Vec2 a = SubtractVV(v, T.position);
		//return b2MulT(T.R, v - T.position);
		double tX = (a.x * T.R.col1.x + a.y * T.R.col1.y );
		a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y );
		a.x = tX;
		return a;
	}

	static   b2Vec2 AddVV(b2Vec2 a,b2Vec2 b)
	{
		b2Vec2 v = new b2Vec2(a.x + b.x, a.y + b.y);
		return v;
	}

	static   b2Vec2 SubtractVV(b2Vec2 a,b2Vec2 b)
	{
		b2Vec2 v = new b2Vec2(a.x - b.x, a.y - b.y);
		return v;
	}
	
	static   double Distance(b2Vec2 a,b2Vec2 b){
		double cX = a.x-b.x;
		double cY = a.y-b.y;
		return sqrt(cX*cX + cY*cY);
	}
	
	static   double DistanceSquared(b2Vec2 a,b2Vec2 b){
		double cX = a.x-b.x;
		double cY = a.y-b.y;
		return (cX*cX + cY*cY);
	}

	static   b2Vec2 MulFV(double s,b2Vec2 a)
	{
		b2Vec2 v = new b2Vec2(s * a.x, s * a.y);
		return v;
	}

	static   b2Mat22 AddMM(b2Mat22 A,b2Mat22 B)
	{
		b2Mat22 C = b2Mat22.FromVV(AddVV(A.col1, B.col1), AddVV(A.col2, B.col2));
		return C;
	}

	// A * B
	static   b2Mat22 MulMM(b2Mat22 A,b2Mat22 B)
	{
		b2Mat22 C = b2Mat22.FromVV(MulMV(A, B.col1), MulMV(A, B.col2));
		return C;
	}

	// A^T * B
	static   b2Mat22 MulTMM(b2Mat22 A,b2Mat22 B)
	{
		b2Vec2 c1 = new b2Vec2(Dot(A.col1, B.col1), Dot(A.col2, B.col1));
		b2Vec2 c2 = new b2Vec2(Dot(A.col1, B.col2), Dot(A.col2, B.col2));
		b2Mat22 C = b2Mat22.FromVV(c1, c2);
		return C;
	}

	static   double Abs(double a)
	{
		return a > 0.0 ? a : -a;
	}

	static   b2Vec2 AbsV(b2Vec2 a)
	{
		b2Vec2 b = new b2Vec2(Abs(a.x), Abs(a.y));
		return b;
	}

	static   b2Mat22 AbsM(b2Mat22 A)
	{
		b2Mat22 B = b2Mat22.FromVV(AbsV(A.col1), AbsV(A.col2));
		return B;
	}

	static   double Min(double a,double b)
	{
		return a < b ? a : b;
	}

	static   b2Vec2 MinV(b2Vec2 a,b2Vec2 b)
	{
		b2Vec2 c = new b2Vec2(Min(a.x, b.x), Min(a.y, b.y));
		return c;
	}

	static   double Max(double a,double b)
	{
		return a > b ? a : b;
	}

	static   b2Vec2 MaxV(b2Vec2 a,b2Vec2 b)
	{
		b2Vec2 c = new b2Vec2(Max(a.x, b.x), Max(a.y, b.y));
		return c;
	}

	static   double Clamp(double a,double low,double high)
	{
		return a < low ? low : a > high ? high : a;
	}

	static   b2Vec2 ClampV(b2Vec2 a,b2Vec2 low,b2Vec2 high)
	{
		return MaxV(low, MinV(a, high));
	}

	static   void Swap(List a,List b)
	{
		dynamic tmp = a[0];
		a[0] = b[0];
		b[0] = tmp;
	}

	// b2Random doubleber in range [-1,1]
	static double GetRandom()
	{
		return new Random().nextDouble() * 2 - 1;
	}

	static double RandomRange(double lo,double hi)
	{
		double r = new Random().nextDouble();
		r = (hi - lo) * r + lo;
		return r;
	}

	// "Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value:"
	static   int NextPowerOfTwo(int x)
	{
		x |= (x >> 1) & 0x7FFFFFFF;
		x |= (x >> 2) & 0x3FFFFFFF;
		x |= (x >> 4) & 0x0FFFFFFF;
		x |= (x >> 8) & 0x00FFFFFF;
		x |= (x >> 16)& 0x0000FFFF;
		return x + 1;
	}

	static   bool IsPowerOfTwo(int x)
	{
		bool result = x > 0 && (x & (x - 1)) == 0;
		return result;
	}
	
	
	// Temp vector functions to reduce calls to 'new'
	/*static  b2Vec2 tempVec = new b2Vec2();
	static  b2Vec2 tempVec2 = new b2Vec2();
	static  b2Vec2 tempVec3 = new b2Vec2();
	static  b2Vec2 tempVec4 = new b2Vec2();
	static  b2Vec2 tempVec5 = new b2Vec2();
	
	static  b2Mat22 tempMat = new b2Mat22();	
	
	static  b2AABB tempAABB = new b2AABB();	*/
	
	static final b2Vec2 b2Vec2_zero = new b2Vec2(0.0, 0.0);
	static final b2Mat22 b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));
	static final b2Transform b2Transform_identity = new b2Transform(b2Vec2_zero, b2Mat22_identity);
	

}
