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
 class b2Bound{
	  bool IsLower() { return (value & 1) == 0; }
	  bool IsUpper() { return (value & 1) == 1; }
	  void Swap(b2Bound b){
		int tempValue = value;
		b2Proxy tempProxy = proxy;
		int tempStabbingCount = stabbingCount;
		
		value = b.value;
		proxy = b.proxy;
		stabbingCount = b.stabbingCount;
		
		b.value = tempValue;
		b.proxy = tempProxy;
		b.stabbingCount = tempStabbingCount;
	}

	 int value = 0;
	 b2Proxy proxy;
	 int stabbingCount = 0;
}
	
	
