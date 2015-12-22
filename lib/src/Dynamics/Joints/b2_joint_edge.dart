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






/**
* A joint edge is used to connect bodies and joints together
* in a joint graph where each body is a node and each joint
* is an edge. A joint edge belongs to a doubly linked list
* maintained in each attached body. Each joint has two joint
* nodes, one for attached body.
*/

 class b2JointEdge
{
	
	/** Provides quick access to the other body attached. */
	 b2Body other;
	/** The joint */
	 b2Joint joint;
	/** The previous joint edge in the body's joint list */
	 b2JointEdge prev;
	/** The next joint edge in the body's joint list */
	 b2JointEdge next;	
}

