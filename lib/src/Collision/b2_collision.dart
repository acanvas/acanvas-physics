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
class b2Collision {
  // Null feature
  static const int b2_nullFeature = 0x000000ff; //UCHAR_MAX;

  // Sutherland-Hodgman clipping.
  static int ClipSegmentToLine(List<ClipVertex> vOut, List<ClipVertex> vIn, b2Vec2 normal, double offset) {
    ClipVertex cv;

    // Start with no output points
    int doubleOut = 0;

    cv = vIn[0];
    b2Vec2 vIn0 = cv.v;
    cv = vIn[1];
    b2Vec2 vIn1 = cv.v;

    // Calculate the distance of end points to the line
    double distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset;
    double distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset;

    // If the points are behind the plane
    if (distance0 <= 0.0) vOut[doubleOut++].Set(vIn[0]);
    if (distance1 <= 0.0) vOut[doubleOut++].Set(vIn[1]);

    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0.0) {
      // Find intersection point of edge and plane
      double interp = distance0 / (distance0 - distance1);
      // expanded for performance
      // vOut[doubleOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
      cv = vOut[doubleOut];
      b2Vec2 tVec = cv.v;
      tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
      tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
      cv = vOut[doubleOut];
      ClipVertex cv2;
      if (distance0 > 0.0) {
        cv2 = vIn[0];
        cv.id = cv2.id;
      } else {
        cv2 = vIn[1];
        cv.id = cv2.id;
      }
      ++doubleOut;
    }

    return doubleOut;
  }

  // Find the separation between poly1 and poly2 for a give edge normal on poly1.
  static double EdgeSeparation(
      b2PolygonShape poly1, b2Transform xf1, int edge1, b2PolygonShape poly2, b2Transform xf2) {
    int count1 = poly1.m_vertexCount;
    List<b2Vec2> vertices1 = poly1.m_vertices;
    List<b2Vec2> normals1 = poly1.m_normals;

    int count2 = poly2.m_vertexCount;
    List<b2Vec2> vertices2 = poly2.m_vertices;

    //b2Assert(0 <= edge1 && edge1 < count1);

    b2Mat22 tMat;
    b2Vec2 tVec;

    // Convert normal from poly1's frame into poly2's frame.
    //b2Vec2 normal1World = b2Mul(xf1.R, normals1[edge1]);
    tMat = xf1.R;
    tVec = normals1[edge1];
    double normal1WorldX = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    double normal1WorldY = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    //b2Vec2 normal1 = b2MulT(xf2.R, normal1World);
    tMat = xf2.R;
    double normal1X = (tMat.col1.x * normal1WorldX + tMat.col1.y * normal1WorldY);
    double normal1Y = (tMat.col2.x * normal1WorldX + tMat.col2.y * normal1WorldY);

    // Find support vertex on poly2 for -normal.
    int index = 0;
    double minDot = double.maxFinite;
    for (int i = 0; i < count2; ++i) {
      //float32 dot = b2Dot(poly2->m_vertices[i], normal1);
      tVec = vertices2[i];
      double dot = tVec.x * normal1X + tVec.y * normal1Y;
      if (dot < minDot) {
        minDot = dot;
        index = i;
      }
    }

    //b2Vec2 v1 = b2Mul(xf1, vertices1[edge1]);
    tVec = vertices1[edge1];
    tMat = xf1.R;
    double v1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    double v1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    //b2Vec2 v2 = b2Mul(xf2, vertices2[index]);
    tVec = vertices2[index];
    tMat = xf2.R;
    double v2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    double v2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

    //double separation = b2Math.b2Dot( b2Math.SubtractVV( v2, v1 ) , normal);
    v2X -= v1X;
    v2Y -= v1Y;
    //float32 separation = b2Dot(v2 - v1, normal1World);
    double separation = v2X * normal1WorldX + v2Y * normal1WorldY;
    return separation;
  }

  // Find the max separation between poly1 and poly2 using edge normals
  // from poly1.
  static double FindMaxSeparation(
      List<int> edgeIndex, b2PolygonShape poly1, b2Transform xf1, b2PolygonShape poly2, b2Transform xf2) {
    int count1 = poly1.m_vertexCount;
    List<b2Vec2> normals1 = poly1.m_normals;

    b2Vec2 tVec;
    b2Mat22 tMat;

    // Listpointing from the centroid of poly1 to the centroid of poly2.
    //b2Vec2 d = b2Mul(xf2, poly2->m_centroid) - b2Mul(xf1, poly1->m_centroid);
    tMat = xf2.R;
    tVec = poly2.m_centroid;
    double dX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    double dY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    tMat = xf1.R;
    tVec = poly1.m_centroid;
    dX -= xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    dY -= xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

    //b2Vec2 dLocal1 = b2MulT(xf1.R, d);
    double dLocal1X = (dX * xf1.R.col1.x + dY * xf1.R.col1.y);
    double dLocal1Y = (dX * xf1.R.col2.x + dY * xf1.R.col2.y);

    // Get support vertex as a hint for our search
    int edge = 0;
    double maxDot = -double.maxFinite;
    for (int i = 0; i < count1; ++i) {
      //double dot = b2Math.b2Dot(normals1[i], dLocal1);
      tVec = normals1[i];
      double dot = (tVec.x * dLocal1X + tVec.y * dLocal1Y);
      if (dot > maxDot) {
        maxDot = dot;
        edge = i;
      }
    }

    // Get the separation for the edge normal.
    double s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);

    // Check the separation for the previous edge normal.
    int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
    double sPrev = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);

    // Check the separation for the next edge normal.
    int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
    double sNext = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

    // Find the best edge and the search direction.
    int bestEdge = 0;
    double bestSeparation = 0.0;
    int increment = 0;
    if (sPrev > s && sPrev > sNext) {
      increment = -1;
      bestEdge = prevEdge;
      bestSeparation = sPrev;
    } else if (sNext > s) {
      increment = 1;
      bestEdge = nextEdge;
      bestSeparation = sNext;
    } else {
      // pointer out
      edgeIndex[0] = edge;
      return s;
    }

    // Perform a local search for the best edge normal.
    while (true) {
      if (increment == -1)
        edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
      else
        edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

      s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);

      if (s > bestSeparation) {
        bestEdge = edge;
        bestSeparation = s;
      } else {
        break;
      }
    }

    // pointer out
    edgeIndex[0] = bestEdge;
    return bestSeparation;
  }

  static void FindIncidentEdge(
      List<ClipVertex> c, b2PolygonShape poly1, b2Transform xf1, int edge1, b2PolygonShape poly2, b2Transform xf2) {
    int count1 = poly1.m_vertexCount;
    List<b2Vec2> normals1 = poly1.m_normals;

    int count2 = poly2.m_vertexCount;
    List<b2Vec2> vertices2 = poly2.m_vertices;
    List<b2Vec2> normals2 = poly2.m_normals;

    //b2Assert(0 <= edge1 && edge1 < count1);

    b2Mat22 tMat;
    b2Vec2 tVec;

    // Get the normal of the reference edge in poly2's frame.
    //b2Vec2 normal1 = b2MulT(xf2.R, b2Mul(xf1.R, normals1[edge1]));
    tMat = xf1.R;
    tVec = normals1[edge1];
    double normal1X = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    double normal1Y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    tMat = xf2.R;
    double tX = (tMat.col1.x * normal1X + tMat.col1.y * normal1Y);
    normal1Y = (tMat.col2.x * normal1X + tMat.col2.y * normal1Y);
    normal1X = tX;

    // Find the incident edge on poly2.
    int index = 0;
    double minDot = double.maxFinite;
    for (int i = 0; i < count2; ++i) {
      //double dot = b2Dot(normal1, normals2[i]);
      tVec = normals2[i];
      double dot = (normal1X * tVec.x + normal1Y * tVec.y);
      if (dot < minDot) {
        minDot = dot;
        index = i;
      }
    }

    ClipVertex tClip;
    // Build the clip vertices for the incident edge.
    int i1 = index;
    int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

    tClip = c[0];
    //c[0].v = b2Mul(xf2, vertices2[i1]);
    tVec = vertices2[i1];
    tMat = xf2.R;
    tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

    tClip.id.features.referenceEdge = edge1;
    tClip.id.features.incidentEdge = i1;
    tClip.id.features.incidentVertex = 0;

    tClip = c[1];
    //c[1].v = b2Mul(xf2, vertices2[i2]);
    tVec = vertices2[i2];
    tMat = xf2.R;
    tClip.v.x = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    tClip.v.y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

    tClip.id.features.referenceEdge = edge1;
    tClip.id.features.incidentEdge = i2;
    tClip.id.features.incidentVertex = 1;
  }

  static List<ClipVertex> MakeClipPointVector() {
    List<ClipVertex> r = new List<ClipVertex>(2);
    r[0] = new ClipVertex();
    r[1] = new ClipVertex();
    return r;
  }

  static List<ClipVertex> s_incidentEdge = MakeClipPointVector();
  static List<ClipVertex> s_clipPoints1 = MakeClipPointVector();
  static List<ClipVertex> s_clipPoints2 = MakeClipPointVector();
  static List<int> s_edgeAO = new List<int>(1);
  static List<int> s_edgeBO = new List<int>(1);
  static b2Vec2 s_localTangent = new b2Vec2();
  static b2Vec2 s_localNormal = new b2Vec2();
  static b2Vec2 s_planePoint = new b2Vec2();
  static b2Vec2 s_normal = new b2Vec2();
  static b2Vec2 s_tangent = new b2Vec2();
  static b2Vec2 s_tangent2 = new b2Vec2();
  static b2Vec2 s_v11 = new b2Vec2();
  static b2Vec2 s_v12 = new b2Vec2();
  // Find edge normal of max separation on A - return if separating axis is found
  // Find edge normal of max separation on B - return if separation axis is found
  // Choose reference edge as min(minA, minB)
  // Find incident edge
  // Clip
  static b2Vec2 b2CollidePolyTempVec = new b2Vec2();
  // The normal points from 1 to 2
  static void CollidePolygons(
      b2Manifold manifold, b2PolygonShape polyA, b2Transform xfA, b2PolygonShape polyB, b2Transform xfB) {
    ClipVertex cv;

    manifold.m_pointCount = 0;
    double totalRadius = polyA.m_radius + polyB.m_radius;

    int edgeA = 0;
    s_edgeAO[0] = edgeA;
    double separationA = FindMaxSeparation(s_edgeAO, polyA, xfA, polyB, xfB);
    edgeA = s_edgeAO[0];
    if (separationA > totalRadius) return;

    int edgeB = 0;
    s_edgeBO[0] = edgeB;
    double separationB = FindMaxSeparation(s_edgeBO, polyB, xfB, polyA, xfA);
    edgeB = s_edgeBO[0];
    if (separationB > totalRadius) return;

    b2PolygonShape poly1; // reference poly
    b2PolygonShape poly2; // incident poly
    b2Transform xf1;
    b2Transform xf2;
    int edge1 = 0; // reference edge
    int flip = 0;
    const double k_relativeTol = 0.98;
    const double k_absoluteTol = 0.001;
    b2Mat22 tMat;

    if (separationB > k_relativeTol * separationA + k_absoluteTol) {
      poly1 = polyB;
      poly2 = polyA;
      xf1 = xfB;
      xf2 = xfA;
      edge1 = edgeB;
      manifold.m_type = b2Manifold.e_faceB;
      flip = 1;
    } else {
      poly1 = polyA;
      poly2 = polyB;
      xf1 = xfA;
      xf2 = xfB;
      edge1 = edgeA;
      manifold.m_type = b2Manifold.e_faceA;
      flip = 0;
    }

    List<ClipVertex> incidentEdge = s_incidentEdge;
    FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

    int count1 = poly1.m_vertexCount;
    List<b2Vec2> vertices1 = poly1.m_vertices;

    b2Vec2 local_v11 = vertices1[edge1];
    b2Vec2 local_v12;
    if (edge1 + 1 < count1) {
      local_v12 = vertices1[(edge1 + 1).toInt()];
    } else {
      local_v12 = vertices1[0];
    }

    b2Vec2 localTangent = s_localTangent;
    localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
    localTangent.Normalize();

    b2Vec2 localNormal = s_localNormal;
    localNormal.x = localTangent.y;
    localNormal.y = -localTangent.x;

    b2Vec2 planePoint = s_planePoint;
    planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));

    b2Vec2 tangent = s_tangent;
    //tangent = b2Math.b2MulMV(xf1.R, localTangent);
    tMat = xf1.R;
    tangent.x = (tMat.col1.x * localTangent.x + tMat.col2.x * localTangent.y);
    tangent.y = (tMat.col1.y * localTangent.x + tMat.col2.y * localTangent.y);
    b2Vec2 tangent2 = s_tangent2;
    tangent2.x = -tangent.x;
    tangent2.y = -tangent.y;
    b2Vec2 normal = s_normal;
    normal.x = tangent.y;
    normal.y = -tangent.x;

    //v11 = b2Math.MulX(xf1, local_v11);
    //v12 = b2Math.MulX(xf1, local_v12);
    b2Vec2 v11 = s_v11;
    b2Vec2 v12 = s_v12;
    v11.x = xf1.position.x + (tMat.col1.x * local_v11.x + tMat.col2.x * local_v11.y);
    v11.y = xf1.position.y + (tMat.col1.y * local_v11.x + tMat.col2.y * local_v11.y);
    v12.x = xf1.position.x + (tMat.col1.x * local_v12.x + tMat.col2.x * local_v12.y);
    v12.y = xf1.position.y + (tMat.col1.y * local_v12.x + tMat.col2.y * local_v12.y);

    // Face offset
    double frontOffset = normal.x * v11.x + normal.y * v11.y;
    // Side offsets, extended by polytope skin thickness
    double sideOffset1 = -tangent.x * v11.x - tangent.y * v11.y + totalRadius;
    double sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius;

    // Clip incident edge against extruded edge1 side edges.
    List<ClipVertex> clipPoints1 = s_clipPoints1;
    List<ClipVertex> clipPoints2 = s_clipPoints2;
    int np = 0;

    // Clip to box side 1
    //np = ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1);
    np = ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1);

    if (np < 2) return;

    // Clip to negative box side 1
    np = ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2);

    if (np < 2) return;

    // Now clipPoints2 contains the clipped points.
    manifold.m_localPlaneNormal.SetV(localNormal);
    manifold.m_localPoint.SetV(planePoint);

    int pointCount = 0;
    for (int i = 0; i < b2Settings.b2_maxManifoldPoints; ++i) {
      cv = clipPoints2[i];
      double separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
      if (separation <= totalRadius) {
        b2ManifoldPoint cp = manifold.m_points[pointCount];
        //cp.m_localPoint = b2Math.b2MulXT(xf2, cv.v);
        tMat = xf2.R;
        double tX = cv.v.x - xf2.position.x;
        double tY = cv.v.y - xf2.position.y;
        cp.m_localPoint.x = (tX * tMat.col1.x + tY * tMat.col1.y);
        cp.m_localPoint.y = (tX * tMat.col2.x + tY * tMat.col2.y);
        cp.m_id.Set(cv.id);
        cp.m_id.features.flip = flip;
        ++pointCount;
      }
    }

    manifold.m_pointCount = pointCount;
  }

  static void CollideCircles(
      b2Manifold manifold, b2CircleShape circle1, b2Transform xf1, b2CircleShape circle2, b2Transform xf2) {
    manifold.m_pointCount = 0;

    b2Mat22 tMat;
    b2Vec2 tVec;

    //b2Vec2 p1 = b2Mul(xf1, circle1->m_p);
    tMat = xf1.R;
    tVec = circle1.m_p;
    double p1X = xf1.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    double p1Y = xf1.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    //b2Vec2 p2 = b2Mul(xf2, circle2->m_p);
    tMat = xf2.R;
    tVec = circle2.m_p;
    double p2X = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    double p2Y = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    //b2Vec2 d = p2 - p1;
    double dX = p2X - p1X;
    double dY = p2Y - p1Y;
    //double distSqr = b2Math.b2Dot(d, d);
    double distSqr = dX * dX + dY * dY;
    double radius = circle1.m_radius + circle2.m_radius;
    if (distSqr > radius * radius) {
      return;
    }
    manifold.m_type = b2Manifold.e_circles;
    manifold.m_localPoint.SetV(circle1.m_p);
    manifold.m_localPlaneNormal.SetZero();
    manifold.m_pointCount = 1;
    manifold.m_points[0].m_localPoint.SetV(circle2.m_p);
    manifold.m_points[0].m_id.key = 0;
  }

  static void CollidePolygonAndCircle(
      b2Manifold manifold, b2PolygonShape polygon, b2Transform xf1, b2CircleShape circle, b2Transform xf2) {
    manifold.m_pointCount = 0;
    b2ManifoldPoint tPoint;

    double dX = 0.0;
    double dY = 0.0;
    double positionX = 0.0;
    double positionY = 0.0;

    b2Vec2 tVec;
    b2Mat22 tMat;

    // Compute circle position in the frame of the polygon.
    //b2Vec2 c = b2Mul(xf2, circle->m_localPosition);
    tMat = xf2.R;
    tVec = circle.m_p;
    double cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    double cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

    //b2Vec2 cLocal = b2MulT(xf1, c);
    dX = cX - xf1.position.x;
    dY = cY - xf1.position.y;
    tMat = xf1.R;
    double cLocalX = (dX * tMat.col1.x + dY * tMat.col1.y);
    double cLocalY = (dX * tMat.col2.x + dY * tMat.col2.y);

    double dist = 0.0;

    // Find the min separating edge.
    int normalIndex = 0;
    double separation = -double.maxFinite;
    double radius = polygon.m_radius + circle.m_radius;
    int vertexCount = polygon.m_vertexCount;
    List<b2Vec2> vertices = polygon.m_vertices;
    List<b2Vec2> normals = polygon.m_normals;

    for (int i = 0; i < vertexCount; ++i) {
      //float32 s = b2Dot(normals[i], cLocal - vertices[i]);
      tVec = vertices[i];
      dX = cLocalX - tVec.x;
      dY = cLocalY - tVec.y;
      tVec = normals[i];
      double s = tVec.x * dX + tVec.y * dY;

      if (s > radius) {
        // Early out.
        return;
      }

      if (s > separation) {
        separation = s;
        normalIndex = i;
      }
    }
    // Vertices that subtend the incident face
    int vertIndex1 = normalIndex;
    int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
    b2Vec2 v1 = vertices[vertIndex1];
    b2Vec2 v2 = vertices[vertIndex2];

    // If the center is inside the polygon ...
    if (separation < double.minPositive) {
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.SetV(normals[normalIndex]);
      manifold.m_localPoint.x = 0.5 * (v1.x + v2.x);
      manifold.m_localPoint.y = 0.5 * (v1.y + v2.y);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0;
      return;
    }

    // Project the circle center onto the edge segment.
    double u1 = (cLocalX - v1.x) * (v2.x - v1.x) + (cLocalY - v1.y) * (v2.y - v1.y);
    double u2 = (cLocalX - v2.x) * (v1.x - v2.x) + (cLocalY - v2.y) * (v1.y - v2.y);
    if (u1 <= 0.0) {
      if ((cLocalX - v1.x) * (cLocalX - v1.x) + (cLocalY - v1.y) * (cLocalY - v1.y) > radius * radius) return;
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.x = cLocalX - v1.x;
      manifold.m_localPlaneNormal.y = cLocalY - v1.y;
      manifold.m_localPlaneNormal.Normalize();
      manifold.m_localPoint.SetV(v1);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0;
    } else if (u2 <= 0) {
      if ((cLocalX - v2.x) * (cLocalX - v2.x) + (cLocalY - v2.y) * (cLocalY - v2.y) > radius * radius) return;
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.x = cLocalX - v2.x;
      manifold.m_localPlaneNormal.y = cLocalY - v2.y;
      manifold.m_localPlaneNormal.Normalize();
      manifold.m_localPoint.SetV(v2);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0;
    } else {
      double faceCenterX = 0.5 * (v1.x + v2.x);
      double faceCenterY = 0.5 * (v1.y + v2.y);
      separation = (cLocalX - faceCenterX) * normals[vertIndex1].x + (cLocalY - faceCenterY) * normals[vertIndex1].y;
      if (separation > radius) return;
      manifold.m_pointCount = 1;
      manifold.m_type = b2Manifold.e_faceA;
      manifold.m_localPlaneNormal.x = normals[vertIndex1].x;
      manifold.m_localPlaneNormal.y = normals[vertIndex1].y;
      manifold.m_localPlaneNormal.Normalize();
      manifold.m_localPoint.Set(faceCenterX, faceCenterY);
      manifold.m_points[0].m_localPoint.SetV(circle.m_p);
      manifold.m_points[0].m_id.key = 0;
    }
  }

  static bool TestOverlap(b2AABB a, b2AABB b) {
    b2Vec2 t1 = b.lowerBound;
    b2Vec2 t2 = a.upperBound;
    //d1 = b2Math.SubtractVV(b.lowerBound, a.upperBound);
    double d1X = t1.x - t2.x;
    double d1Y = t1.y - t2.y;
    //d2 = b2Math.SubtractVV(a.lowerBound, b.upperBound);
    t1 = a.lowerBound;
    t2 = b.upperBound;
    double d2X = t1.x - t2.x;
    double d2Y = t1.y - t2.y;

    if (d1X > 0.0 || d1Y > 0.0) return false;

    if (d2X > 0.0 || d2Y > 0.0) return false;

    return true;
  }
}
