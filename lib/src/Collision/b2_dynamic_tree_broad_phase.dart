part of rockdot_physics;

/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
 * It is up to the client to consume the new pairs and to track subsequent overlap.
 */
class b2DynamicTreeBroadPhase implements IBroadPhase {
  b2DynamicTree m_tree = new b2DynamicTree();
  int m_proxyCount = 0;
  List<b2DynamicTreeNode> m_moveBuffer = new List<b2DynamicTreeNode>();

  List<b2DynamicTreePair> m_pairBuffer = new List<b2DynamicTreePair>();
  int m_pairCount = 0;

  /**
	 * Create a proxy with an initial AABB. Pairs are not reported until
	 * UpdatePairs is called.
	 */
  dynamic CreateProxy(b2AABB aabb, dynamic userData) {
    b2DynamicTreeNode proxy = m_tree.CreateProxy(aabb, userData);
    ++m_proxyCount;
    BufferMove(proxy);
    return proxy;
  }

  /**
	 * Destroy a proxy. It is up to the client to remove any pairs.
	 */
  void DestroyProxy(dynamic proxy) {
    UnBufferMove(proxy);
    --m_proxyCount;
    m_tree.DestroyProxy(proxy);
  }

  /**
	 * Call MoveProxy as many times as you like, then when you are done
	 * call UpdatePairs to finalized the proxy pairs (for your time step).
	 */
  void MoveProxy(dynamic proxy, b2AABB aabb, b2Vec2 displacement) {
    bool buffer = m_tree.MoveProxy(proxy, aabb, displacement);
    if (buffer) {
      BufferMove(proxy);
    }
  }

  bool TestOverlap(dynamic proxyA, dynamic proxyB) {
    b2AABB aabbA = m_tree.GetFatAABB(proxyA);
    b2AABB aabbB = m_tree.GetFatAABB(proxyB);
    return aabbA.TestOverlap(aabbB);
  }

  /**
	 * Get user data from a proxy. Returns null if the proxy is invalid.
	 */
  dynamic GetUserData(dynamic proxy) {
    return m_tree.GetUserData(proxy);
  }

  /**
	 * Get the AABB for a proxy.
	 */
  b2AABB GetFatAABB(dynamic proxy) {
    return m_tree.GetFatAABB(proxy);
  }

  /**
	 * Get the doubleber of proxies.
	 */
  int GetProxyCount() {
    return m_proxyCount;
  }

  bool QueryCallback(b2DynamicTreeNode proxy) {
    // A proxy cannot form a pair with itself.
    if (proxy == queryProxy) return true;

    // Grow the pair buffer as needed
    if (m_pairCount == m_pairBuffer.length) {
      m_pairBuffer.add(new b2DynamicTreePair());
    }

    m_pairBuffer[m_pairCount].proxyA = (proxy.key < queryProxy.key) ? proxy : queryProxy;
    m_pairBuffer[m_pairCount].proxyB = (proxy.key >= queryProxy.key) ? proxy : queryProxy;
    ++m_pairCount;

    return true;
  }

  b2DynamicTreeNode queryProxy;

  /**
	 * Update the pairs. This results in pair callbacks. This can only add pairs.
	 */
  void UpdatePairs(Function callback) {
    m_pairCount = 0;
    // Perform tree queries for all moving queries
    for (int i = 0; i < m_moveBuffer.length; ++i) {
      queryProxy = m_moveBuffer[i];
      if (queryProxy == null) continue;
      // We have to query the tree with the fat AABB so that
      // we don't fail to create a pair that may touch later.
      b2AABB fatAABB = m_tree.GetFatAABB(queryProxy);
      m_tree.Query(QueryCallback, fatAABB);
    }

    // Reset move buffer
    m_moveBuffer.length = 0;

    // Sort the pair buffer to expose duplicates.
    // TODO: Something more sensible
    //m_pairBuffer.sort(ComparePairs);

    // Send the pair buffer
    for (int i = 0; i < m_pairCount;) {
      b2DynamicTreePair primaryPair = m_pairBuffer[i];
      dynamic userDataA = m_tree.GetUserData(primaryPair.proxyA);
      dynamic userDataB = m_tree.GetUserData(primaryPair.proxyB);
      callback(userDataA, userDataB);
      ++i;

      // Skip any duplicate pairs
      while (i < m_pairCount) {
        b2DynamicTreePair pair = m_pairBuffer[i];
        if (pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB) {
          break;
        }
        ++i;
      }
    }
  }

  /**
	 * @inheritDoc
	 */
  void Query(Function callback, b2AABB aabb) {
    m_tree.Query(callback, aabb);
  }

  /**
	 * @inheritDoc
	 */
  void RayCast(Function callback, b2RayCastInput input) {
    m_tree.RayCast(callback, input);
  }

  void Validate() {
    //TODO_BORIS
  }

  void Rebalance(int iterations) {
    m_tree.Rebalance(iterations);
  }

  // Private ///////////////

  void BufferMove(b2DynamicTreeNode proxy) {
    m_moveBuffer.add(proxy);
  }

  void UnBufferMove(b2DynamicTreeNode proxy) {
    int i = m_moveBuffer.indexOf(proxy);
    if (i != -1) m_moveBuffer.removeAt(i);
  }

  int ComparePairs(b2DynamicTreePair pair1, b2DynamicTreePair pair2) {
    //TODO_BORIS:
    // We cannot consistently sort objects easily in AS3
    // The caller of this needs replacing with a different method.
    return 0;
  }
}
