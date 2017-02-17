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
* This structure is used to report contact points.
*/
class b2ContactPoint {
  /** The first shape */
  b2Shape shape1;
  /** The second shape */
  b2Shape shape2;
  /** Position in world coordinates */
  b2Vec2 position = new b2Vec2();
  /** Velocity of point on body2 relative to point on body1 (pre-solver) */
  b2Vec2 velocity = new b2Vec2();
  /** Points from shape1 to shape2 */
  b2Vec2 normal = new b2Vec2();
  /** The separation is negative when shapes are touching */
  double separation = 0.0;
  /** The combined friction coefficient */
  double friction = 0.0;
  /** The combined restitution coefficient */
  double restitution = 0.0;
  /** The contact id identifies the features in contact */
  b2ContactID id = new b2ContactID();
}
