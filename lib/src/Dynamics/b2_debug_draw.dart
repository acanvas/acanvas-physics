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
* Implement and register this class with a b2World to provide debug drawing of physics
* entities in your game.
*/
class b2DebugDraw {
  b2DebugDraw() {
    m_drawFlags = 0;
  }

  //virtual ~b2DebugDraw() {}

  //edouble
  //{
  /** Draw shapes */
  static int e_shapeBit = 0x0001;
  /** Draw joint connections */
  static int e_jointBit = 0x0002;
  /** Draw axis aligned bounding boxes */
  static int e_aabbBit = 0x0004;
  /** Draw broad-phase pairs */
  static int e_pairBit = 0x0008;
  /** Draw center of mass frame */
  static int e_centerOfMassBit = 0x0010;
  /** Draw controllers */
  static int e_controllerBit = 0x0020;
  //}

  /**
	* Set the drawing flags.
	*/
  void SetFlags(int flags) {
    m_drawFlags = flags;
  }

  /**
	* Get the drawing flags.
	*/
  int GetFlags() {
    return m_drawFlags;
  }

  /**
	* Append flags to the current flags.
	*/
  void AppendFlags(int flags) {
    m_drawFlags |= flags;
  }

  /**
	* Clear flags from the current flags.
	*/
  void ClearFlags(int flags) {
    m_drawFlags &= ~flags;
  }

  /**
	* Set the sprite
	*/
  void SetSprite(Sprite sprite) {
    m_sprite = sprite;
  }

  /**
	* Get the sprite
	*/
  Sprite GetSprite() {
    return m_sprite;
  }

  /**
	* Set the draw scale
	*/
  void SetDrawScale(double drawScale) {
    m_drawScale = drawScale;
  }

  /**
	* Get the draw
	*/
  double GetDrawScale() {
    return m_drawScale;
  }

  /**
	* Set the line thickness
	*/
  void SetLineThickness(double lineThickness) {
    m_lineThickness = lineThickness;
  }

  /**
	* Get the line thickness
	*/
  double GetLineThickness() {
    return m_lineThickness;
  }

  /**
	* Set the alpha value used for lines
	*/
  void SetAlpha(double alpha) {
    m_alpha = alpha;
  }

  /**
	* Get the alpha value used for lines
	*/
  double GetAlpha() {
    return m_alpha;
  }

  /**
	* Set the alpha value used for fills
	*/
  void SetFillAlpha(double alpha) {
    m_fillAlpha = alpha;
  }

  /**
	* Get the alpha value used for fills
	*/
  double GetFillAlpha() {
    return m_fillAlpha;
  }

  /**
	* Set the scale used for drawing XForms
	*/
  void SetXFormScale(double xformScale) {
    m_xformScale = xformScale;
  }

  /**
	* Get the scale used for drawing XForms
	*/
  double GetXFormScale() {
    return m_xformScale;
  }

  /**
	* Draw a closed polygon provided in CCW order.
	*/
  void DrawPolygon(List vertices, int vertexCount, b2Color color) {
    m_sprite.graphics.moveTo(vertices[0].x * m_drawScale, vertices[0].y * m_drawScale);
    m_sprite.graphics.beginPath();
    for (int i = 1; i < vertexCount; i++) {
      m_sprite.graphics.lineTo(vertices[i].x * m_drawScale, vertices[i].y * m_drawScale);
    }
    m_sprite.graphics.lineTo(vertices[0].x * m_drawScale, vertices[0].y * m_drawScale);
    m_sprite.graphics.closePath();
    m_sprite.graphics.strokeColor(((m_alpha * 255).round() << 24) | color.color, m_lineThickness);
  }

  /**
	* Draw a solid closed polygon provided in CCW order.
	*/
  void DrawSolidPolygon(List<b2Vec2> vertices, int vertexCount, b2Color color) {
    m_sprite.graphics.moveTo(vertices[0].x * m_drawScale, vertices[0].y * m_drawScale);
    m_sprite.graphics.beginPath();
    for (int i = 1; i < vertexCount; i++) {
      m_sprite.graphics.lineTo(vertices[i].x * m_drawScale, vertices[i].y * m_drawScale);
    }
    m_sprite.graphics.lineTo(vertices[0].x * m_drawScale, vertices[0].y * m_drawScale);
    m_sprite.graphics.closePath();
    m_sprite.graphics.strokeColor(((m_alpha * 255).round() << 24) | color.color, m_lineThickness);
    m_sprite.graphics.fillColor(((m_fillAlpha * 255).round() << 24) & color.color);
    //m_sprite.graphics.endFill() //not supported in StageXL;
  }

  /**
	* Draw a circle.
	*/
  void DrawCircle(b2Vec2 center, double radius, b2Color color) {
    m_sprite.graphics.circle(center.x * m_drawScale, center.y * m_drawScale, radius * m_drawScale);
    m_sprite.graphics.strokeColor(((m_alpha * 255).round() << 24) | color.color, m_lineThickness);
  }

  /**
	* Draw a solid circle.
	*/
  void DrawSolidCircle(b2Vec2 center, double radius, b2Vec2 axis, b2Color color) {
    m_sprite.graphics.circle(center.x * m_drawScale, center.y * m_drawScale, radius * m_drawScale);
    m_sprite.graphics.fillColor(((m_fillAlpha * 255).round() << 24) | color.color);

    m_sprite.graphics.moveTo(center.x * m_drawScale, center.y * m_drawScale);
    m_sprite.graphics.lineTo((center.x + axis.x * radius) * m_drawScale, (center.y + axis.y * radius) * m_drawScale);
    m_sprite.graphics.strokeColor(((m_alpha * 255).round() << 24) | color.color, m_lineThickness);
  }

  /**
	* Draw a line segment.
	*/
  void DrawSegment(b2Vec2 p1, b2Vec2 p2, b2Color color) {
    m_sprite.graphics.moveTo(p1.x * m_drawScale, p1.y * m_drawScale);
    m_sprite.graphics.lineTo(p2.x * m_drawScale, p2.y * m_drawScale);
    m_sprite.graphics.strokeColor(((m_alpha * 255).round() << 24) | color.color, m_lineThickness);
  }

  /**
	* Draw a transform. Choose your own length scale.
	* @param xf a transform.
	*/
  void DrawTransform(b2Transform xf) {
    m_sprite.graphics.moveTo(xf.position.x * m_drawScale, xf.position.y * m_drawScale);
    m_sprite.graphics.lineTo((xf.position.x + m_xformScale * xf.R.col1.x) * m_drawScale,
        (xf.position.y + m_xformScale * xf.R.col1.y) * m_drawScale);
    m_sprite.graphics.strokeColor(((m_alpha * 255).round() << 24) | 0xff0000, m_lineThickness);

    m_sprite.graphics.moveTo(xf.position.x * m_drawScale, xf.position.y * m_drawScale);
    m_sprite.graphics.lineTo((xf.position.x + m_xformScale * xf.R.col2.x) * m_drawScale,
        (xf.position.y + m_xformScale * xf.R.col2.y) * m_drawScale);
    m_sprite.graphics.strokeColor(((m_alpha * 255).round() << 24) | 0x00ff00, m_lineThickness);
  }

  int m_drawFlags = 0;
  Sprite m_sprite;
  double m_drawScale = 1.0;

  double m_lineThickness = 1.0;
  double m_alpha = 1.0;
  double m_fillAlpha = 1.0;
  double m_xformScale = 1.0;
}
