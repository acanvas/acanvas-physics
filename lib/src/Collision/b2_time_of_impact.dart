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
 class b2TimeOfImpact
{
	
	 static int b2_toiCalls = 0;
	 static int b2_toiIters = 0;
	 static int b2_toiMaxIters = 0;
	 static int b2_toiRootIters = 0;
	 static int b2_toiMaxRootIters = 0;

	 static b2SimplexCache s_cache = new b2SimplexCache();
	 static b2DistanceInput s_distanceInput = new b2DistanceInput();
	 static b2Transform s_xfA = new b2Transform();
	 static b2Transform s_xfB = new b2Transform();
	 static b2SeparationFunction s_fcn = new b2SeparationFunction();
	 static b2DistanceOutput s_distanceOutput = new b2DistanceOutput();
	 static  double TimeOfImpact(b2TOIInput input)
	{
		++b2_toiCalls;
		
		b2DistanceProxy proxyA = input.proxyA;
		b2DistanceProxy proxyB = input.proxyB;
		
		b2Sweep sweepA = input.sweepA;
		b2Sweep sweepB = input.sweepB;
		
		b2Settings.b2Assert(sweepA.t0 == sweepB.t0);
		b2Settings.b2Assert(1.0 - sweepA.t0 > double.MIN_POSITIVE);
		
		double radius = proxyA.m_radius + proxyB.m_radius;
		double tolerance = input.tolerance;
		
		double alpha = 0.0;
		
		const int k_maxIterations = 1000; //TODO_ERIN b2Settings
		int iter = 0;
		double target = 0.0;
		
		// Prepare input for distance query.
		s_cache.count = 0;
		s_distanceInput.useRadii = false;
		
		for (;; )
		{
			sweepA.GetTransform(s_xfA, alpha);
			sweepB.GetTransform(s_xfB, alpha);
			
			// Get the distance between shapes
			s_distanceInput.proxyA = proxyA;
			s_distanceInput.proxyB = proxyB;
			s_distanceInput.transformA = s_xfA;
			s_distanceInput.transformB = s_xfB;
			
			b2Distance.Distance(s_distanceOutput, s_cache, s_distanceInput);
			
			if (s_distanceOutput.distance <= 0.0)
			{
				alpha = 1.0;
				break;
			}
			
			s_fcn.Initialize(s_cache, proxyA, s_xfA, proxyB, s_xfB);
			
			double separation = s_fcn.Evaluate(s_xfA, s_xfB);
			if (separation <= 0.0)
			{
				alpha = 1.0;
				break;
			}
			
			if (iter == 0)
			{
				// Compute a reasonable target distance to give some breathing room
				// for conservative advancement. We take advantage of the shape radii
				// to create additional clearance
				if (separation > radius)
				{
					target = b2Math.Max(radius - tolerance, 0.75 * radius);
				}
				else
				{
					target = b2Math.Max(separation - tolerance, 0.02 * radius);
				}
			}
			
			if (separation - target < 0.5 * tolerance)
			{
				if (iter == 0)
				{
					alpha = 1.0;
					break;
				}
				break;
			}
			
//#if 0
			// Dump the curve seen by the root finder
			//{
				//const int N = 100;
				//double dx = 1.0 / N;
				//List<double> xs = new List(N + 1);
				//List<double> fs = new List(N + 1);
				//
				//double x = 0.0;
				//for (int i = 0; i <= N; i++)
				//{
					//sweepA.GetTransform(xfA, x);
					//sweepB.GetTransform(xfB, x);
					//double f = fcn.Evaluate(xfA, xfB) - target;
					//
					//print(x, f);
					//xs[i] = x;
					//fx[i] = f'
					//
					//x += dx;
				//}
			//}
//#endif
			// Compute 1D root of f(x) - target = 0
			double newAlpha = alpha;
			{
				double x1 = alpha;
				double x2 = 1.0;
				
				double f1 = separation;
				
				sweepA.GetTransform(s_xfA, x2);
				sweepB.GetTransform(s_xfB, x2);
				
				double f2 = s_fcn.Evaluate(s_xfA, s_xfB);
				
				// If intervals don't overlap at t2, then we are done
				if (f2 >= target)
				{
					alpha = 1.0;
					break;
				}
				
				// Determine when intervals intersect
				int rootIterCount = 0;
				for (;; )
				{
					// Use a mis of the secand rule and bisection
					double x = 0.0;
					if ((rootIterCount & 1) > 0)
					{
						// Secant rule to improve convergence
						x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
					}
					else
					{
						// Bisection to guarantee progress
						x = 0.5 * (x1 + x2);
					}
					
					sweepA.GetTransform(s_xfA, x);
					sweepB.GetTransform(s_xfB, x);
					
					double f = s_fcn.Evaluate(s_xfA, s_xfB);
					
					if (b2Math.Abs(f - target) < 0.025 * tolerance)
					{
						newAlpha = x;
						break;
					}
					
					// Ensure we continue to bracket the root
					if (f > target)
					{
						x1 = x;
						f1 = f;
					}
					else
					{
						x2 = x;
						f2 = f;
					}
					
					++rootIterCount;
					++b2_toiRootIters;
					if (rootIterCount == 50)
					{
						break;
					}
				}
				
				b2_toiMaxRootIters = b2Math.Max(b2_toiMaxRootIters.toDouble(), rootIterCount.toDouble()).toInt();
			}
			
			// Ensure significant advancement
			if (newAlpha < (1.0 + 100.0 * double.MIN_POSITIVE) * alpha)
			{
				break;
			}
			
			alpha = newAlpha;
			
			iter++;
			++b2_toiIters;
			
			if (iter == k_maxIterations)
			{
				break;
			}
		}
		
		b2_toiMaxIters = b2Math.Max(b2_toiMaxIters.toDouble(), iter.toDouble()).toInt();

		return alpha;
	}

}

