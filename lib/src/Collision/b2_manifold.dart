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
 * A manifold for two touching convex shapes.
 * Box2D supports multiple types of contact: dynamic - clip point versus plane with radius
 * - point versus point with radius (circles)
 * The local point usage depends on the manifold type: dynamic -e_circles: the local center of circleA
 * -e_faceA: the center of faceA
 * -e_faceB: the center of faceB
 * Similarly the local normal usage: dynamic -e_circles: not used
 * -e_faceA: the normal on polygonA
 * -e_faceB: the normal on polygonB
 * We store contacts in this way so that position correction can
 * account for movement, which is critical for continuous physics.
 * All contact scenarios must be expressed in one of these types.
 * This structure is stored across time steps, so we keep it small.
 */
class b2Manifold {
  b2Manifold() {
    _m_points = new List<b2ManifoldPoint>(b2Settings.b2_maxManifoldPoints);
    for (int i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
      _m_points[i] = new b2ManifoldPoint();
    }
    m_localPlaneNormal = new b2Vec2();
    m_localPoint = new b2Vec2();
  }
  void Reset() {
    for (int i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
      (_m_points[i] as b2ManifoldPoint).Reset();
    }
    m_localPlaneNormal.SetZero();
    m_localPoint.SetZero();
    m_type = 0;
    _m_pointCount = 0;
  }
  void Set(b2Manifold m) {
    _m_pointCount = m._m_pointCount;
    for (int i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
      (_m_points[i] as b2ManifoldPoint).Set(m._m_points[i]);
    }
    m_localPlaneNormal.SetV(m.m_localPlaneNormal);
    m_localPoint.SetV(m.m_localPoint);
    m_type = m.m_type;
  }
  b2Manifold Copy() {
    b2Manifold copy = new b2Manifold();
    copy.Set(this);
    return copy;
  }
  /** The points of contact */
  List<b2ManifoldPoint> _m_points;
  void set m_points(List<b2ManifoldPoint> p){
_m_points = p;
  }
  List<b2ManifoldPoint> get m_points {
    return _m_points;
  }
  
  int _m_pointCount = 0;
  void set m_pointCount(int p){
    _m_pointCount = p;
  }
  int get m_pointCount => _m_pointCount;
  
  /** Not used for Type e_points*/
  b2Vec2 m_localPlaneNormal;
  /** Usage depends on manifold type */
  b2Vec2 m_localPoint;
  int m_type = 0;
  /** The doubleber of manifold points */

  //edouble Type
  static const int e_circles = 0x0001;
  static const int e_faceA = 0x0002;
  static const int e_faceB = 0x0004;
}

