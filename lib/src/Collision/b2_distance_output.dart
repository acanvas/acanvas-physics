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
	 * Output for b2Distance.
	 */
	 class b2DistanceOutput 
	{
	/** Closest point on shapea */	 b2Vec2 pointA = new b2Vec2();	/** Closest point on shapeb */	 b2Vec2 pointB = new b2Vec2();		 double distance;
	/** double of gjk iterations used */	 int iterations;	}
	
