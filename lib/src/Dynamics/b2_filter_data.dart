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
* This holds contact filtering data.
*/
class b2FilterData {
  b2FilterData Copy() {
    b2FilterData copy = new b2FilterData();
    copy.categoryBits = categoryBits;
    copy.maskBits = maskBits;
    copy.groupIndex = groupIndex;
    return copy;
  }

  /**
	* The collision category bits. Normally you would just set one bit.
	*/
  int categoryBits = 0x0001;

  /**
	* The collision mask bits. This states the categories that this
	* shape would accept for collision.
	*/
  int maskBits = 0xFFFF;

  /**
	* Collision groups allow a certain group of objects to never collide (negative)
	* or always collide (positive). Zero means no collision group. Non-zero group
	* filtering always wins against the mask bits.
	*/
  int groupIndex = 0;
}
