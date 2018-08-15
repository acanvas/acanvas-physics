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
* A 2D column vector.
*/

class b2Vec2 {
  b2Vec2([double x_ = 0.0, double y_ = 0.0]) {
    x = x_;
    y = y_;
  }

  void SetZero() {
    x = 0.0;
    y = 0.0;
  }

  void Set([double x_ = 0.0, double y_ = 0.0]) {
    x = x_;
    y = y_;
  }

  void SetV(b2Vec2 v) {
    x = v.x;
    y = v.y;
  }

  b2Vec2 GetNegative() {
    return new b2Vec2(-x, -y);
  }

  void NegativeSelf() {
    x = -x;
    y = -y;
  }

  static b2Vec2 Make(double x_, double y_) {
    return new b2Vec2(x_, y_);
  }

  b2Vec2 Copy() {
    return new b2Vec2(x, y);
  }

  void Add(b2Vec2 v) {
    x += v.x;
    y += v.y;
  }

  void Subtract(b2Vec2 v) {
    x -= v.x;
    y -= v.y;
  }

  void Multiply(double a) {
    x *= a;
    y *= a;
  }

  void MulM(b2Mat22 A) {
    double tX = x;
    x = A.col1.x * tX + A.col2.x * y;
    y = A.col1.y * tX + A.col2.y * y;
  }

  void MulTM(b2Mat22 A) {
    double tX = b2Math.Dot(this, A.col1);
    y = b2Math.Dot(this, A.col2);
    x = tX;
  }

  void CrossVF(double s) {
    double tX = x;
    x = s * y;
    y = -s * tX;
  }

  void CrossFV(double s) {
    double tX = x;
    x = -s * y;
    y = s * tX;
  }

  void MinV(b2Vec2 b) {
    x = x < b.x ? x : b.x;
    y = y < b.y ? y : b.y;
  }

  void MaxV(b2Vec2 b) {
    x = x > b.x ? x : b.x;
    y = y > b.y ? y : b.y;
  }

  void Abs() {
    if (x < 0) x = -x;
    if (y < 0) y = -y;
  }

  double Length() {
    return sqrt(x * x + y * y);
  }

  double LengthSquared() {
    return (x * x + y * y);
  }

  double Normalize() {
    double length = sqrt(x * x + y * y);
    if (length < double.minPositive) {
      return 0.0;
    }
    double invLength = 1.0 / length;
    x *= invLength;
    y *= invLength;

    return length;
  }

  bool IsValid() {
    return b2Math.IsValid(x) && b2Math.IsValid(y);
  }

  double x = 0.0;
  double y = 0.0;
}
