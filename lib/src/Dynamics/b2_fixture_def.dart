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
 * A fixture definition is used to create a fixture. This class defines an
 * abstract fixture definition. You can reuse fixture definitions safely.
 */
 class b2FixtureDef
{
	/**
	 * The constructor sets the default fixture definition values.
	 */
	 b2FixtureDef()
	{
		shape = null;
		userData = null;
		friction = 0.2;
		restitution = 0.0;
		density = 0.0;
		filter.categoryBits = 0x0001;
		filter.maskBits = 0xFFFF;
		filter.groupIndex = 0;
		isSensor = false;
	}
	
	/**
	 * The shape, this must be set. The shape will be cloned, so you
	 * can create the shape on the stack.
	 */
	 b2Shape shape;

	/**
	 * Use this to store application specific fixture data.
	 */
	 dynamic userData;

	/**
	 * The friction coefficient, usually in the range [0,1].
	 */
	 double friction = 0.0;

	/**
	 * The restitution (usually as elasticity) in the range [0,1].
	 */
	 double restitution = 0.0;

	/**
	 * The density, usually in kg/m^2.
	 */
	 double density = 0.0;

	/**
	 * A sensor shape collects contact information but never generates a collision
	 * response.
	 */
	 bool isSensor;

	/**
	 * Contact filtering data.
	 */
	 b2FilterData filter = new b2FilterData();
}



