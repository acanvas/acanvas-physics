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

class b2SimplexVertex {
  void Set(b2SimplexVertex other) {
    wA.SetV(other.wA);
    wB.SetV(other.wB);
    w.SetV(other.w);
    a = other.a;
    indexA = other.indexA;
    indexB = other.indexB;
  }

  b2Vec2 wA; // support point in proxyA
  b2Vec2 wB; // support point in proxyB
  b2Vec2 w; // wB - wA
  double a = 0.0; // barycentric coordinate for closest point
  int indexA = 0; // wA index
  int indexB = 0; // wB index
}
