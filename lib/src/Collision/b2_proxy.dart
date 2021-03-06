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

part of acanvas_physics;

/**
* @
*/
class b2Proxy {
  bool IsValid() {
    return overlapCount != b2BroadPhase.b2_invalid;
  }

  List<int> lowerBounds = new List<int>(2);
  List<int> upperBounds = new List<int>(2);
  int overlapCount = 0;
  int timeStamp = 0;

  // Maps from the other b2Proxy to their mutual b2Pair.
  Map pairs = new Map();

  b2Proxy next;

  dynamic userData = null;
}
