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
* We use contact ids to facilitate warm starting.
*/
 class Features
{
	/**
	* The edge that defines the outward contact normal.
	*/
	  int get referenceEdge{
		return _referenceEdge;
	}
	  void set referenceEdge(int value){
		_referenceEdge = value;
		_m_id._key = (_m_id._key & 0xffffff00) | (_referenceEdge & 0x000000ff);
	}
	int _referenceEdge;
	
	/**
	* The edge most anti-parallel to the reference edge.
	*/
	  int get incidentEdge{
		return _incidentEdge;
	}
	  void set incidentEdge(int value){
		_incidentEdge = value;
		_m_id._key = (_m_id._key & 0xffff00ff) | ((_incidentEdge << 8) & 0x0000ff00);
	}
	int _incidentEdge;
	
	/**
	* The vertex (0 or 1) on the incident edge that was clipped.
	*/
	  int get incidentVertex{
		return _incidentVertex;
	}
	  void set incidentVertex(int value){
		_incidentVertex = value;
		_m_id._key = (_m_id._key & 0xff00ffff) | ((_incidentVertex << 16) & 0x00ff0000);
	}
	int _incidentVertex;
	
	/**
	* A value of 1 indicates that the reference edge is on shape2.
	*/
	  int get flip{
		return _flip;
	}
	  void set flip(int value){
		_flip = value;
		_m_id._key = (_m_id._key & 0x00ffffff) | ((_flip << 24) & 0xff000000);
	}
	int _flip;
	
	
	b2ContactID _m_id;
}


