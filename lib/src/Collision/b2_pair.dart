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

// The pair manager is used by the broad-phase to quickly add/remove/find pairs
// of overlapping proxies. It is based closely on code provided by Pierre Terdiman.
// http://www.codercorner.com/IncrementalSAP.txt

 part of stagexl_box2d;






/**
 * A Pair represents a pair of overlapping b2Proxy in the broadphse.
 * @
 */
 class b2Pair
{
	

	  void SetBuffered()	{ status |= e_pairBuffered; }
	  void ClearBuffered()	{ status &= ~e_pairBuffered; }
	  bool IsBuffered()	{ return (status & e_pairBuffered) == e_pairBuffered; }

	  void SetRemoved()		{ status |= e_pairRemoved; }
	  void ClearRemoved()	{ status &= ~e_pairRemoved; }
	  bool IsRemoved()		{ return (status & e_pairRemoved) == e_pairRemoved; }
	
	  void SetFinal()		{ status |= e_pairFinal; }
	  bool IsFinal()		{ return (status & e_pairFinal) == e_pairFinal; }

	 dynamic userData = null;
	 b2Proxy proxy1;
	 b2Proxy proxy2;
	 b2Pair next;
	 int status = 0;
	
	// STATIC
	static  int b2_nullProxy = b2Settings.USHRT_MAX;
	
	// edouble
	static  int e_pairBuffered = 0x0001;
	static  int e_pairRemoved = 0x0002;
	static  int e_pairFinal = 0x0004;

}


