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
* This describes the motion of a body/shape for TOI computation.
* Shapes are defined with respect to the body origin, which may
* no coincide with the center of mass. However, to support dynamics
* we must interpolate the center of mass position.
*/
 class b2Sweep
{
	  void Set(b2Sweep other)
	{
		localCenter.SetV(other.localCenter);
		c0.SetV(other.c0);
		c.SetV(other.c);
		a0 = other.a0;
		a = other.a;
		t0 = other.t0;
	}
	
	  b2Sweep Copy()
	{
		b2Sweep copy = new b2Sweep();
		copy.localCenter.SetV(localCenter);
		copy.c0.SetV(c0);
		copy.c.SetV(c);
		copy.a0 = a0;
		copy.a = a;
		copy.t0 = t0;
		return copy;
	}
	
	/**
	* Get the interpolated transform at a specific time.
	* @param alpha is a factor in [0,1], where 0 indicates t0.
	*/
	  void GetTransform(b2Transform xf,double alpha)
	{
		xf.position.x = (1.0 - alpha) * c0.x + alpha * c.x;
		xf.position.y = (1.0 - alpha) * c0.y + alpha * c.y;
		double angle = (1.0 - alpha) * a0 + alpha * a;
		xf.R.Set(angle);
		
		// Shift to origin
		//xf->position -= b2Mul(xf->R, localCenter);
		b2Mat22 tMat = xf.R;
		xf.position.x -= (tMat.col1.x * localCenter.x + tMat.col2.x * localCenter.y);
		xf.position.y -= (tMat.col1.y * localCenter.x + tMat.col2.y * localCenter.y);
	}
	
	/**
	* Advance the sweep forward, yielding a new initial state.
	* @param t the new initial time.
	*/
	  void Advance(double t){
		if (t0 < t && 1.0 - t0 > double.MIN_POSITIVE)
		{
			double alpha = (t - t0) / (1.0 - t0);
			//c0 = (1.0f - alpha) * c0 + alpha * c;
			c0.x = (1.0 - alpha) * c0.x + alpha * c.x;
			c0.y = (1.0 - alpha) * c0.y + alpha * c.y;
			a0 = (1.0 - alpha) * a0 + alpha * a;
			t0 = t;
		}
	}

	/** Local center of mass position */
	 b2Vec2 localCenter = new b2Vec2();
	/** Center world position */
	 b2Vec2 c0 = new b2Vec2();
	/** Center world position */
	 b2Vec2 c = new b2Vec2();
	/** World angle */
	 double a0 = 0.0;
	/** World angle */
	 double a = 0.0;
	/** Time interval = [t0,1], where t0 is in [0,1] */
	 double t0 = 0.0;
}

