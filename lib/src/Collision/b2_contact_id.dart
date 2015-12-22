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

 part of rockdot_box2d;
	



// 
/**
* We use contact ids to facilitate warm starting.
*/
 class b2ContactID
{
	 b2ContactID(){
		features._m_id = this;
		
	}
	  void Set(b2ContactID id){
		key = id._key;
	}
	  b2ContactID Copy(){
		b2ContactID id = new b2ContactID();
		id.key = key;
		return id;
	}
	  int get key {
		return _key;
	}
	  void set key(int value) {
		_key = value;
		features._referenceEdge = _key & 0x000000ff;
		features._incidentEdge = ((_key & 0x0000ff00) >> 8) & 0x000000ff;
		features._incidentVertex = ((_key & 0x00ff0000) >> 16) & 0x000000ff;
		features._flip = ((_key & 0xff000000) >> 24) & 0x000000ff;
	}
	 Features features = new Features();
	/** Used to quickly compare contact ids. */
	int _key = 0;
}


