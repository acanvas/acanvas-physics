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
* Color for debug drawing. Each value has the range [0,1].
*/

class b2Color {
  b2Color(double rr, double gg, double bb) {
    _r = (255 * b2Math.Clamp(rr, 0.0, 1.0)).toInt();
    _g = (255 * b2Math.Clamp(gg, 0.0, 1.0)).toInt();
    _b = (255 * b2Math.Clamp(bb, 0.0, 1.0)).toInt();
  }

  void Set(double rr, double gg, double bb) {
    _r = (255 * b2Math.Clamp(rr, 0.0, 1.0)).toInt();
    _g = (255 * b2Math.Clamp(gg, 0.0, 1.0)).toInt();
    _b = (255 * b2Math.Clamp(bb, 0.0, 1.0)).toInt();
  }

  // R
  void set r(double rr) {
    _r = (255 * b2Math.Clamp(rr, 0.0, 1.0)).toInt();
  }

  // G
  void set g(double gg) {
    _g = (255 * b2Math.Clamp(gg, 0.0, 1.0)).toInt();
  }

  // B
  void set b(double bb) {
    _b = (255 * b2Math.Clamp(bb, 0.0, 1.0)).toInt();
  }

  // Color
  int get color {
    return (_r << 16) | (_g << 8) | (_b);
  }

  int _r = 0;
  int _g = 0;
  int _b = 0;
}
