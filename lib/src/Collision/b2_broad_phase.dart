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

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
//   overlap query results.
// - where possible, we compare bound indices instead of values to reduce
//   cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for huge
//   worlds (use a multi-SAP instead), it is not great for large objects.

/**
* @
*/
class b2BroadPhase implements IBroadPhase {
//:
  b2BroadPhase(b2AABB worldAABB) {
    //b2Settings.b2Assert(worldAABB.IsValid());
    int i = 0;

    m_pairManager.Initialize(this);

    m_worldAABB = worldAABB;

    m_proxyCount = 0;

    // bounds array
    m_bounds = new List<List<b2Bound>>();
    for (i = 0; i < 2; i++) {
      m_bounds[i] = new List<b2Bound>();
    }

    //b2Vec2 d = worldAABB.upperBound - worldAABB.lowerBound;
    double dX = worldAABB.upperBound.x - worldAABB.lowerBound.x;
    ;
    double dY = worldAABB.upperBound.y - worldAABB.lowerBound.y;

    m_quantizationFactor.x = b2Settings.USHRT_MAX / dX;
    m_quantizationFactor.y = b2Settings.USHRT_MAX / dY;

    m_timeStamp = 1;
    m_queryResultCount = 0;
  }
  //~b2BroadPhase();

  // Use this to see if your proxy is in range. If it is not in range,
  // it should be destroyed. Otherwise you may get O(m^2) pairs, where m
  // is the doubleber of proxies that are out of range.
  bool InRange(b2AABB aabb) {
    //b2Vec2 d = b2Max(aabb.lowerBound - m_worldAABB.upperBound, m_worldAABB.lowerBound - aabb.upperBound);
    double dX = 0.0;
    double dY = 0.0;
    double d2X = 0.0;
    double d2Y = 0.0;

    dX = aabb.lowerBound.x;
    dY = aabb.lowerBound.y;
    dX -= m_worldAABB.upperBound.x;
    dY -= m_worldAABB.upperBound.y;

    d2X = m_worldAABB.lowerBound.x;
    d2Y = m_worldAABB.lowerBound.y;
    d2X -= aabb.upperBound.x;
    d2Y -= aabb.upperBound.y;

    dX = b2Math.Max(dX, d2X);
    dY = b2Math.Max(dY, d2Y);

    return b2Math.Max(dX, dY) < 0.0;
  }

  // Create and destroy proxies. These call Flush first.
  dynamic CreateProxy(b2AABB aabb, dynamic userData) {
    int index = 0;
    b2Proxy proxy;
    int i = 0;
    int j = 0;

    //b2Settings.b2Assert(m_proxyCount < b2_maxProxies);
    //b2Settings.b2Assert(m_freeProxy != b2Pair.b2_nullProxy);

    if (m_freeProxy == null) {
      // As all proxies are allocated, m_proxyCount == m_proxyPool.length
      m_freeProxy = m_proxyPool[m_proxyCount] = new b2Proxy();
      m_freeProxy.next = null;
      m_freeProxy.timeStamp = 0;
      m_freeProxy.overlapCount = b2_invalid;
      m_freeProxy.userData = null;

      for (i = 0; i < 2; i++) {
        j = m_proxyCount * 2;
        m_bounds[i][j++] = new b2Bound();
        m_bounds[i][j] = new b2Bound();
      }
    }

    proxy = m_freeProxy;
    m_freeProxy = proxy.next;

    proxy.overlapCount = 0;
    proxy.userData = userData;

    int boundCount = 2 * m_proxyCount;

    List<double> lowerValues = new List<double>();
    List<double> upperValues = new List<double>();
    ComputeBounds(lowerValues, upperValues, aabb);

    for (int axis = 0; axis < 2; ++axis) {
      List<b2Bound> bounds = m_bounds[axis];
      int lowerIndex = 0;
      int upperIndex = 0;
      List<int> lowerIndexOut = new List<int>();
      lowerIndexOut.add(lowerIndex);
      List<int> upperIndexOut = new List<int>();
      upperIndexOut.add(upperIndex);
      QueryAxis(
          lowerIndexOut, upperIndexOut, lowerValues[axis].toInt(), upperValues[axis].toInt(), bounds, boundCount, axis);
      lowerIndex = lowerIndexOut[0];
      upperIndex = upperIndexOut[0];

      bounds.insert(upperIndex, bounds[bounds.length - 1]);
      bounds.length--;
      bounds.insert(lowerIndex, bounds[bounds.length - 1]);
      bounds.length--;

      // The upper index has increased because of the lower bound insertion.
      ++upperIndex;

      // Copy in the new bounds.
      b2Bound tBound1 = bounds[lowerIndex];
      b2Bound tBound2 = bounds[upperIndex];
      tBound1.value = lowerValues[axis].toInt();
      tBound1.proxy = proxy;
      tBound2.value = upperValues[axis].toInt();
      tBound2.proxy = proxy;

      b2Bound tBoundAS3 = bounds[(lowerIndex - 1).toInt()];
      tBound1.stabbingCount = lowerIndex == 0 ? 0 : tBoundAS3.stabbingCount;
      tBoundAS3 = bounds[(upperIndex - 1).toInt()];
      tBound2.stabbingCount = tBoundAS3.stabbingCount;

      // Adjust the stabbing count between the new bounds.
      for (index = lowerIndex; index < upperIndex; ++index) {
        tBoundAS3 = bounds[index];
        tBoundAS3.stabbingCount++;
      }

      // Adjust the all the affected bound indices.
      for (index = lowerIndex; index < boundCount + 2; ++index) {
        tBound1 = bounds[index];
        b2Proxy proxy2 = tBound1.proxy;
        if (tBound1.IsLower()) {
          proxy2.lowerBounds[axis] = index;
        } else {
          proxy2.upperBounds[axis] = index;
        }
      }
    }

    ++m_proxyCount;

    //b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);

    for (i = 0; i < m_queryResultCount; ++i) {
      //b2Settings.b2Assert(m_queryResults[i] < b2_maxProxies);
      //b2Settings.b2Assert(m_proxyPool[m_queryResults[i]].IsValid());

      m_pairManager.AddBufferedPair(proxy, m_queryResults[i]);
    }

    // Prepare for next query.
    m_queryResultCount = 0;
    IncrementTimeStamp();

    return proxy;
  }

  void DestroyProxy(dynamic proxy_) {
    b2Proxy proxy = proxy_ as b2Proxy;
    b2Bound tBound1;
    b2Bound tBound2;

    //b2Settings.b2Assert(proxy.IsValid());

    int boundCount = 2 * m_proxyCount;

    for (int axis = 0; axis < 2; ++axis) {
      List<b2Bound> bounds = m_bounds[axis];

      int lowerIndex = proxy.lowerBounds[axis];
      int upperIndex = proxy.upperBounds[axis];
      tBound1 = bounds[lowerIndex];
      int lowerValue = tBound1.value;
      tBound2 = bounds[upperIndex];
      int upperValue = tBound2.value;

      bounds.removeAt(upperIndex);
      bounds.removeAt(lowerIndex);
      bounds.add(tBound1);
      bounds.add(tBound2);

      // Fix bound indices.
      int tEnd = boundCount - 2;
      for (int index = lowerIndex; index < tEnd; ++index) {
        tBound1 = bounds[index];
        b2Proxy proxy2 = tBound1.proxy;
        if (tBound1.IsLower()) {
          proxy2.lowerBounds[axis] = index;
        } else {
          proxy2.upperBounds[axis] = index;
        }
      }

      // Fix stabbing count.
      tEnd = upperIndex - 1;
      for (int index2 = lowerIndex; index2 < tEnd; ++index2) {
        tBound1 = bounds[index2];
        tBound1.stabbingCount--;
      }

      // Query for pairs to be removed. lowerIndex and upperIndex are not needed.
      // make lowerIndex and upper output using an array and do this for others if compiler doesn't pick them up
      List<int> ignore = new List<int>();
      QueryAxis(ignore, ignore, lowerValue, upperValue, bounds, boundCount - 2, axis);
    }

    //b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);

    for (int i = 0; i < m_queryResultCount; ++i) {
      //b2Settings.b2Assert(m_proxyPool[m_queryResults[i]].IsValid());

      m_pairManager.RemoveBufferedPair(proxy, m_queryResults[i]);
    }

    // Prepare for next query.
    m_queryResultCount = 0;
    IncrementTimeStamp();

    // Return the proxy to the pool.
    proxy.userData = null;
    proxy.overlapCount = b2_invalid;
    proxy.lowerBounds[0] = b2_invalid;
    proxy.lowerBounds[1] = b2_invalid;
    proxy.upperBounds[0] = b2_invalid;
    proxy.upperBounds[1] = b2_invalid;

    proxy.next = m_freeProxy;
    m_freeProxy = proxy;
    --m_proxyCount;
  }

  // Call MoveProxy as many times as you like, then when you are done
  // call Commit to finalized the proxy pairs (for your time step).
  void MoveProxy(dynamic proxy_, b2AABB aabb, b2Vec2 displacement) {
    b2Proxy proxy = proxy_ as b2Proxy;

    List<int> as3arr;
    int as3int = 0;

    int axis = 0;
    int index = 0;
    b2Bound bound;
    b2Bound prevBound;
    b2Bound nextBound;
    int nextProxyId = 0;
    b2Proxy nextProxy;

    if (proxy == null) {
      //b2Settings.b2Assert(false);
      return;
    }

    if (aabb.IsValid() == false) {
      //b2Settings.b2Assert(false);
      return;
    }

    int boundCount = 2 * m_proxyCount;

    // Get new bound values
    b2BoundValues newValues = new b2BoundValues();
    ComputeBounds(newValues.lowerValues, newValues.upperValues, aabb);

    // Get old bound values
    b2BoundValues oldValues = new b2BoundValues();
    for (axis = 0; axis < 2; ++axis) {
      bound = m_bounds[axis][proxy.lowerBounds[axis]];
      oldValues.lowerValues[axis] = bound.value.toDouble();
      bound = m_bounds[axis][proxy.upperBounds[axis]];
      oldValues.upperValues[axis] = bound.value.toDouble();
    }

    for (axis = 0; axis < 2; ++axis) {
      List<b2Bound> bounds = m_bounds[axis];

      int lowerIndex = proxy.lowerBounds[axis];
      int upperIndex = proxy.upperBounds[axis];

      int lowerValue = newValues.lowerValues[axis].toInt();
      int upperValue = newValues.upperValues[axis].toInt();

      bound = bounds[lowerIndex];
      int deltaLower = lowerValue - bound.value;
      bound.value = lowerValue;

      bound = bounds[upperIndex];
      int deltaUpper = upperValue - bound.value;
      bound.value = upperValue;

      //
      // Expanding adds overlaps
      //
      b2Proxy prevProxy;

      // Should we move the lower bound down?
      if (deltaLower < 0) {
        index = lowerIndex;
        while (index > 0 && lowerValue < (bounds[(index - 1)]).value) {
          bound = bounds[index];
          prevBound = bounds[(index - 1).toInt()];

          prevProxy = prevBound.proxy;

          prevBound.stabbingCount++;

          if (prevBound.IsUpper() == true) {
            if (TestOverlapBound(newValues, prevProxy)) {
              m_pairManager.AddBufferedPair(proxy, prevProxy);
            }

            //prevProxy.upperBounds[axis]++;
            as3arr = prevProxy.upperBounds;
            as3int = as3arr[axis];
            as3int++;
            as3arr[axis] = as3int;

            bound.stabbingCount++;
          } else {
            //prevProxy.lowerBounds[axis]++;
            as3arr = prevProxy.lowerBounds;
            as3int = as3arr[axis];
            as3int++;
            as3arr[axis] = as3int;

            bound.stabbingCount--;
          }

          //proxy.lowerBounds[axis]--;
          as3arr = proxy.lowerBounds;
          as3int = as3arr[axis];
          as3int--;
          as3arr[axis] = as3int;

          // swap
          //b2Bound temp = bound;
          //bound = prevEdge;
          //prevEdge = temp;
          bound.Swap(prevBound);
          //b2Math.Swap(bound, prevEdge);
          --index;
        }
      }

      // Should we move the upper bound up?
      if (deltaUpper > 0) {
        index = upperIndex;
        while (index < boundCount - 1 && (bounds[(index + 1)]).value <= upperValue) {
          bound = bounds[index];
          nextBound = bounds[(index + 1)];
          nextProxy = nextBound.proxy;

          nextBound.stabbingCount++;

          if (nextBound.IsLower() == true) {
            if (TestOverlapBound(newValues, nextProxy)) {
              m_pairManager.AddBufferedPair(proxy, nextProxy);
            }

            //nextProxy.lowerBounds[axis]--;
            as3arr = nextProxy.lowerBounds;
            as3int = as3arr[axis];
            as3int--;
            as3arr[axis] = as3int;

            bound.stabbingCount++;
          } else {
            //nextProxy.upperBounds[axis]--;
            as3arr = nextProxy.upperBounds;
            as3int = as3arr[axis];
            as3int--;
            as3arr[axis] = as3int;

            bound.stabbingCount--;
          }

          //proxy.upperBounds[axis]++;
          as3arr = proxy.upperBounds;
          as3int = as3arr[axis];
          as3int++;
          as3arr[axis] = as3int;

          // swap
          //b2Bound temp = bound;
          //bound = nextEdge;
          //nextEdge = temp;
          bound.Swap(nextBound);
          //b2Math.Swap(bound, nextEdge);
          index++;
        }
      }

      //
      // Shrinking removes overlaps
      //

      // Should we move the lower bound up?
      if (deltaLower > 0) {
        index = lowerIndex;
        while (index < boundCount - 1 && (bounds[(index + 1)]).value <= lowerValue) {
          bound = bounds[index];
          nextBound = bounds[(index + 1).toInt()];

          nextProxy = nextBound.proxy;

          nextBound.stabbingCount--;

          if (nextBound.IsUpper()) {
            if (TestOverlapBound(oldValues, nextProxy)) {
              m_pairManager.RemoveBufferedPair(proxy, nextProxy);
            }

            //nextProxy.upperBounds[axis]--;
            as3arr = nextProxy.upperBounds;
            as3int = as3arr[axis];
            as3int--;
            as3arr[axis] = as3int;

            bound.stabbingCount--;
          } else {
            //nextProxy.lowerBounds[axis]--;
            as3arr = nextProxy.lowerBounds;
            as3int = as3arr[axis];
            as3int--;
            as3arr[axis] = as3int;

            bound.stabbingCount++;
          }

          //proxy.lowerBounds[axis]++;
          as3arr = proxy.lowerBounds;
          as3int = as3arr[axis];
          as3int++;
          as3arr[axis] = as3int;

          // swap
          //b2Bound temp = bound;
          //bound = nextEdge;
          //nextEdge = temp;
          bound.Swap(nextBound);
          //b2Math.Swap(bound, nextEdge);
          index++;
        }
      }

      // Should we move the upper bound down?
      if (deltaUpper < 0) {
        index = upperIndex;
        while (index > 0 && upperValue < (bounds[(index - 1)]).value) {
          bound = bounds[index];
          prevBound = bounds[(index - 1).toInt()];

          prevProxy = prevBound.proxy;

          prevBound.stabbingCount--;

          if (prevBound.IsLower() == true) {
            if (TestOverlapBound(oldValues, prevProxy)) {
              m_pairManager.RemoveBufferedPair(proxy, prevProxy);
            }

            //prevProxy.lowerBounds[axis]++;
            as3arr = prevProxy.lowerBounds;
            as3int = as3arr[axis];
            as3int++;
            as3arr[axis] = as3int;

            bound.stabbingCount--;
          } else {
            //prevProxy.upperBounds[axis]++;
            as3arr = prevProxy.upperBounds;
            as3int = as3arr[axis];
            as3int++;
            as3arr[axis] = as3int;

            bound.stabbingCount++;
          }

          //proxy.upperBounds[axis]--;
          as3arr = proxy.upperBounds;
          as3int = as3arr[axis];
          as3int--;
          as3arr[axis] = as3int;

          // swap
          //b2Bound temp = bound;
          //bound = prevEdge;
          //prevEdge = temp;
          bound.Swap(prevBound);
          //b2Math.Swap(bound, prevEdge);
          index--;
        }
      }
    }
  }

  void UpdatePairs(Function callback) {
    m_pairManager.Commit(callback);
  }

  bool TestOverlap(dynamic proxyA, dynamic proxyB) {
    b2Proxy proxyA_ = proxyA as b2Proxy;
    b2Proxy proxyB_ = proxyB as b2Proxy;
    if (proxyA_.lowerBounds[0] > proxyB_.upperBounds[0]) return false;
    if (proxyB_.lowerBounds[0] > proxyA_.upperBounds[0]) return false;
    if (proxyA_.lowerBounds[1] > proxyB_.upperBounds[1]) return false;
    if (proxyB_.lowerBounds[1] > proxyA_.upperBounds[1]) return false;
    return true;
  }

  /**
	 * Get user data from a proxy. Returns null if the proxy is invalid.
	 */
  dynamic GetUserData(dynamic proxy) {
    return (proxy as b2Proxy).userData;
  }

  /**
	 * Get the AABB for a proxy.
	 */
  b2AABB GetFatAABB(dynamic proxy_) {
    b2AABB aabb = new b2AABB();
    b2Proxy proxy = proxy_ as b2Proxy;
    aabb.lowerBound.x = m_worldAABB.lowerBound.x + m_bounds[0][proxy.lowerBounds[0]].value / m_quantizationFactor.x;
    aabb.lowerBound.y = m_worldAABB.lowerBound.y + m_bounds[1][proxy.lowerBounds[1]].value / m_quantizationFactor.y;
    aabb.upperBound.x = m_worldAABB.lowerBound.x + m_bounds[0][proxy.upperBounds[0]].value / m_quantizationFactor.x;
    aabb.upperBound.y = m_worldAABB.lowerBound.y + m_bounds[1][proxy.upperBounds[1]].value / m_quantizationFactor.y;
    return aabb;
  }

  /**
	 * Get the doubleber of proxies.
	 */
  int GetProxyCount() {
    return m_proxyCount;
  }

  /**
	 * Query an AABB for overlapping proxies. The callback class
	 * is called for proxy that overlaps the supplied AABB.
	 */
  void Query(Function callback, b2AABB aabb) {
    List<double> lowerValues = new List<double>();
    List<double> upperValues = new List<double>();
    ComputeBounds(lowerValues, upperValues, aabb);

    int lowerIndex = 0;
    int upperIndex = 0;
    List<int> lowerIndexOut = new List<int>();
    lowerIndexOut.add(lowerIndex);
    List<int> upperIndexOut = new List<int>();
    upperIndexOut.add(upperIndex);
    QueryAxis(
        lowerIndexOut, upperIndexOut, lowerValues[0].toInt(), upperValues[0].toInt(), m_bounds[0], 2 * m_proxyCount, 0);
    QueryAxis(
        lowerIndexOut, upperIndexOut, lowerValues[1].toInt(), upperValues[1].toInt(), m_bounds[1], 2 * m_proxyCount, 1);

    //b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);

    // TODO: Don't be lazy, transform QueryAxis to directly call callback
    for (int i = 0; i < m_queryResultCount; ++i) {
      b2Proxy proxy = m_queryResults[i];
      //b2Settings.b2Assert(proxy.IsValid());
      if (!callback(proxy)) {
        break;
      }
    }

    // Prepare for next query.
    m_queryResultCount = 0;
    IncrementTimeStamp();
  }

  void Validate() {
    b2Pair pair;
    b2Proxy proxy1;
    b2Proxy proxy2;
    bool overlap;

    for (int axis = 0; axis < 2; ++axis) {
      List<b2Bound> bounds = m_bounds[axis];

      int boundCount = 2 * m_proxyCount;
      int stabbingCount = 0;

      for (int i = 0; i < boundCount; ++i) {
        b2Bound bound = bounds[i];
        //b2Settings.b2Assert(i == 0 || bounds[i-1].value <= bound->value);
        //b2Settings.b2Assert(bound->proxyId != b2_nullProxy);
        //b2Settings.b2Assert(m_proxyPool[bound->proxyId].IsValid());

        if (bound.IsLower() == true) {
          //b2Settings.b2Assert(m_proxyPool[bound.proxyId].lowerBounds[axis] == i);
          stabbingCount++;
        } else {
          //b2Settings.b2Assert(m_proxyPool[bound.proxyId].upperBounds[axis] == i);
          stabbingCount--;
        }

        //b2Settings.b2Assert(bound.stabbingCount == stabbingCount);
      }
    }
  }

  void Rebalance(int iterations) {
    // Do nothing
  }

  /**
	 * @inheritDoc
	 */
  void RayCast(Function callback, b2RayCastInput input) {
    b2RayCastInput subInput = new b2RayCastInput();
    subInput.p1.SetV(input.p1);
    subInput.p2.SetV(input.p2);
    subInput.maxFraction = input.maxFraction;

    double dx = (input.p2.x - input.p1.x) * m_quantizationFactor.x;
    double dy = (input.p2.y - input.p1.y) * m_quantizationFactor.y;

    int sx = dx < -double.MIN_POSITIVE ? -1 : (dx > double.MIN_POSITIVE ? 1 : 0);
    int sy = dy < -double.MIN_POSITIVE ? -1 : (dy > double.MIN_POSITIVE ? 1 : 0);

    //b2Settings.b2Assert(sx!=0||sy!=0);

    double p1x = m_quantizationFactor.x * (input.p1.x - m_worldAABB.lowerBound.x);
    double p1y = m_quantizationFactor.y * (input.p1.y - m_worldAABB.lowerBound.y);

    List startValues = new List();
    List startValues2 = new List();
    startValues[0] = (p1x).toInt() & (b2Settings.USHRT_MAX - 1);
    startValues[1] = (p1y).toInt() & (b2Settings.USHRT_MAX - 1);
    startValues2[0] = startValues[0] + 1;
    startValues2[1] = startValues[1] + 1;

    List startIndices = new List();

    int xIndex = 0;
    int yIndex = 0;

    b2Proxy proxy;

    //First deal with all the proxies that contain segment.p1
    int lowerIndex = 0;
    int upperIndex = 0;
    List<int> lowerIndexOut = new List<int>();
    lowerIndexOut.add(lowerIndex);
    List<int> upperIndexOut = new List<int>();
    upperIndexOut.add(upperIndex);
    QueryAxis(lowerIndexOut, upperIndexOut, startValues[0], startValues2[0], m_bounds[0], 2 * m_proxyCount, 0);
    if (sx >= 0)
      xIndex = upperIndexOut[0] - 1;
    else
      xIndex = lowerIndexOut[0];
    QueryAxis(lowerIndexOut, upperIndexOut, startValues[1], startValues2[1], m_bounds[1], 2 * m_proxyCount, 1);
    if (sy >= 0)
      yIndex = upperIndexOut[0] - 1;
    else
      yIndex = lowerIndexOut[0];

    // Callback for starting proxies:
    for (int i = 0; i < m_queryResultCount; i++) {
      subInput.maxFraction = callback(m_queryResults[i], subInput);
    }

    //Now work through the rest of the segment
    for (;;) {
      double xProgress = 0.0;
      double yProgress = 0.0;
      //Move on to next bound
      xIndex += sx >= 0 ? 1 : -1;
      if (xIndex < 0 || xIndex >= m_proxyCount * 2) break;
      if (sx != 0) {
        xProgress = (m_bounds[0][xIndex].value - p1x) / dx;
      }
      //Move on to next bound
      yIndex += sy >= 0 ? 1 : -1;
      if (yIndex < 0 || yIndex >= m_proxyCount * 2) break;
      if (sy != 0) {
        yProgress = (m_bounds[1][yIndex].value - p1y) / dy;
      }
      for (;;) {
        if (sy == 0 || (sx != 0 && xProgress < yProgress)) {
          if (xProgress > subInput.maxFraction) break;

          //Check that we are entering a proxy, not leaving
          if (sx > 0 ? m_bounds[0][xIndex].IsLower() : m_bounds[0][xIndex].IsUpper()) {
            //Check the other axis of the proxy
            proxy = m_bounds[0][xIndex].proxy;
            if (sy >= 0) {
              if (proxy.lowerBounds[1] <= yIndex - 1 && proxy.upperBounds[1] >= yIndex) {
                //Add the proxy
                subInput.maxFraction = callback(proxy, subInput);
              }
            } else {
              if (proxy.lowerBounds[1] <= yIndex && proxy.upperBounds[1] >= yIndex + 1) {
                //Add the proxy
                subInput.maxFraction = callback(proxy, subInput);
              }
            }
          }

          //Early out
          if (subInput.maxFraction == 0) break;

          //Move on to the next bound
          if (sx > 0) {
            xIndex++;
            if (xIndex == m_proxyCount * 2) break;
          } else {
            xIndex--;
            if (xIndex < 0) break;
          }
          xProgress = (m_bounds[0][xIndex].value - p1x) / dx;
        } else {
          if (yProgress > subInput.maxFraction) break;

          //Check that we are entering a proxy, not leaving
          if (sy > 0 ? m_bounds[1][yIndex].IsLower() : m_bounds[1][yIndex].IsUpper()) {
            //Check the other axis of the proxy
            proxy = m_bounds[1][yIndex].proxy;
            if (sx >= 0) {
              if (proxy.lowerBounds[0] <= xIndex - 1 && proxy.upperBounds[0] >= xIndex) {
                //Add the proxy
                subInput.maxFraction = callback(proxy, subInput);
              }
            } else {
              if (proxy.lowerBounds[0] <= xIndex && proxy.upperBounds[0] >= xIndex + 1) {
                //Add the proxy
                subInput.maxFraction = callback(proxy, subInput);
              }
            }
          }

          //Early out
          if (subInput.maxFraction == 0) break;

          //Move on to the next bound
          if (sy > 0) {
            yIndex++;
            if (yIndex == m_proxyCount * 2) break;
          } else {
            yIndex--;
            if (yIndex < 0) break;
          }
          yProgress = (m_bounds[1][yIndex].value - p1y) / dy;
        }
      }
      break;
    }

    // Prepare for next query.
    m_queryResultCount = 0;
    IncrementTimeStamp();

    return;
  }

//:
  void ComputeBounds(List<double> lowerValues, List<double> upperValues, b2AABB aabb) {
    //b2Settings.b2Assert(aabb.upperBound.x >= aabb.lowerBound.x);
    //b2Settings.b2Assert(aabb.upperBound.y >= aabb.lowerBound.y);

    //b2Vec2 minVertex = b2Math.ClampV(aabb.minVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
    double minVertexX = aabb.lowerBound.x;
    double minVertexY = aabb.lowerBound.y;
    minVertexX = b2Math.Min(minVertexX, m_worldAABB.upperBound.x);
    minVertexY = b2Math.Min(minVertexY, m_worldAABB.upperBound.y);
    minVertexX = b2Math.Max(minVertexX, m_worldAABB.lowerBound.x);
    minVertexY = b2Math.Max(minVertexY, m_worldAABB.lowerBound.y);

    //b2Vec2 maxVertex = b2Math.ClampV(aabb.maxVertex, m_worldAABB.minVertex, m_worldAABB.maxVertex);
    double maxVertexX = aabb.upperBound.x;
    double maxVertexY = aabb.upperBound.y;
    maxVertexX = b2Math.Min(maxVertexX, m_worldAABB.upperBound.x);
    maxVertexY = b2Math.Min(maxVertexY, m_worldAABB.upperBound.y);
    maxVertexX = b2Math.Max(maxVertexX, m_worldAABB.lowerBound.x);
    maxVertexY = b2Math.Max(maxVertexY, m_worldAABB.lowerBound.y);

    // Bump lower bounds downs and upper bounds up. This ensures correct sorting of
    // lower/upper bounds that would have equal values.
    // TODO_ERIN implement fast float to int16 conversion.
    lowerValues[0] =
        ((m_quantizationFactor.x * (minVertexX - m_worldAABB.lowerBound.x)).toInt() & (b2Settings.USHRT_MAX - 1))
            .toDouble();
    upperValues[0] =
        (((m_quantizationFactor.x * (maxVertexX - m_worldAABB.lowerBound.x)).toInt() & 0x0000ffff) | 1).toDouble();

    lowerValues[1] =
        ((m_quantizationFactor.y * (minVertexY - m_worldAABB.lowerBound.y)).toInt() & (b2Settings.USHRT_MAX - 1))
            .toDouble();
    upperValues[1] =
        (((m_quantizationFactor.y * (maxVertexY - m_worldAABB.lowerBound.y)).toInt() & 0x0000ffff) | 1).toDouble();
  }

  // This one is only used for validation.
  bool TestOverlapValidate(b2Proxy p1, b2Proxy p2) {
    for (int axis = 0; axis < 2; ++axis) {
      List<b2Bound> bounds = m_bounds[axis];

      //b2Settings.b2Assert(p1.lowerBounds[axis] < 2 * m_proxyCount);
      //b2Settings.b2Assert(p1.upperBounds[axis] < 2 * m_proxyCount);
      //b2Settings.b2Assert(p2.lowerBounds[axis] < 2 * m_proxyCount);
      //b2Settings.b2Assert(p2.upperBounds[axis] < 2 * m_proxyCount);

      b2Bound bound1 = bounds[p1.lowerBounds[axis]];
      b2Bound bound2 = bounds[p2.upperBounds[axis]];
      if (bound1.value > bound2.value) return false;

      bound1 = bounds[p1.upperBounds[axis]];
      bound2 = bounds[p2.lowerBounds[axis]];
      if (bound1.value < bound2.value) return false;
    }

    return true;
  }

  bool TestOverlapBound(b2BoundValues b, b2Proxy p) {
    for (int axis = 0; axis < 2; ++axis) {
      List<b2Bound> bounds = m_bounds[axis];

      //b2Settings.b2Assert(p.lowerBounds[axis] < 2 * m_proxyCount);
      //b2Settings.b2Assert(p.upperBounds[axis] < 2 * m_proxyCount);

      b2Bound bound = bounds[p.upperBounds[axis]];
      if (b.lowerValues[axis] > bound.value) return false;

      bound = bounds[p.lowerBounds[axis]];
      if (b.upperValues[axis] < bound.value) return false;
    }

    return true;
  }

  void QueryAxis(List<int> lowerQueryOut, List<int> upperQueryOut, int lowerValue, int upperValue, List<b2Bound> bounds,
      int boundCount, int axis) {
    int lowerQuery = BinarySearch(bounds, boundCount, lowerValue);
    int upperQuery = BinarySearch(bounds, boundCount, upperValue);
    b2Bound bound;

    // Easy case: lowerQuery <= lowerIndex(i) < upperQuery
    // Solution: search query range for min bounds.
    for (int j = lowerQuery; j < upperQuery; ++j) {
      bound = bounds[j];
      if (bound.IsLower()) {
        IncrementOverlapCount(bound.proxy);
      }
    }

    // Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
    // Solution: use the stabbing count to search down the bound array.
    if (lowerQuery > 0) {
      int i = lowerQuery - 1;
      bound = bounds[i];
      int s = bound.stabbingCount;

      // Find the s overlaps.
      while (s != 0) {
        //b2Settings.b2Assert(i >= 0);
        bound = bounds[i];
        if (bound.IsLower()) {
          b2Proxy proxy = bound.proxy;
          if (lowerQuery <= proxy.upperBounds[axis]) {
            IncrementOverlapCount(bound.proxy);
            --s;
          }
        }
        --i;
      }
    }

    lowerQueryOut[0] = lowerQuery;
    upperQueryOut[0] = upperQuery;
  }

  void IncrementOverlapCount(b2Proxy proxy) {
    if (proxy.timeStamp < m_timeStamp) {
      proxy.timeStamp = m_timeStamp;
      proxy.overlapCount = 1;
    } else {
      proxy.overlapCount = 2;
      //b2Settings.b2Assert(m_queryResultCount < b2Settings.b2_maxProxies);
      m_queryResults[m_queryResultCount] = proxy;
      ++m_queryResultCount;
    }
  }

  void IncrementTimeStamp() {
    if (m_timeStamp == b2Settings.USHRT_MAX) {
      for (int i = 0; i < m_proxyPool.length; ++i) {
        (m_proxyPool[i] as b2Proxy).timeStamp = 0;
      }
      m_timeStamp = 1;
    } else {
      ++m_timeStamp;
    }
  }

  b2PairManager m_pairManager = new b2PairManager();

  List m_proxyPool = new List();
  b2Proxy m_freeProxy;

  List<List<b2Bound>> m_bounds;

  List m_querySortKeys = new List();
  List m_queryResults = new List();
  int m_queryResultCount = 0;

  b2AABB m_worldAABB;
  b2Vec2 m_quantizationFactor = new b2Vec2();
  int m_proxyCount = 0;
  int m_timeStamp = 0;

  static bool s_validate = false;

  static const int b2_invalid = b2Settings.USHRT_MAX;
  static const int b2_nullEdge = b2Settings.USHRT_MAX;

  static int BinarySearch(List<b2Bound> bounds, int count, int value) {
    double low = 0.0;
    double high = count - 1.0;
    while (low <= high) {
      double mid = ((low + high) / 2);
      b2Bound bound = bounds[mid.toInt()];
      if (bound.value > value) {
        high = mid - 1;
      } else if (bound.value < value) {
        low = mid + 1;
      } else {
        return (mid).toInt();
      }
    }

    return (low).toInt();
  }
}
