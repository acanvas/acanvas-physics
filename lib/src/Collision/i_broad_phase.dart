 part of rockdot_physics;
	/**
	 * Interface for objects tracking overlap of many AABBs.
	 */
	 abstract class IBroadPhase 
	{
		/**
		 * Create a proxy with an initial AABB. Pairs are not reported until
		 * UpdatePairs is called.
		 */
		 dynamic CreateProxy(b2AABB aabb,dynamic userData);
		
		/**
		 * Destroy a proxy. It is up to the client to remove any pairs.
		 */
		 void DestroyProxy(dynamic proxy);
		
		/**
		 * Call MoveProxy as many times as you like, then when you are done
		 * call UpdatePairs to finalized the proxy pairs (for your time step).
		 */
		 void MoveProxy(dynamic proxy,b2AABB aabb,b2Vec2 displacement);
		
		 bool TestOverlap(dynamic proxyA,dynamic proxyB);
		
		/**
		 * Get user data from a proxy. Returns null if the proxy is invalid.
		 */
		 dynamic GetUserData(dynamic proxy);
		
		/**
		 * Get the fat AABB for a proxy.
		 */
		 b2AABB GetFatAABB(dynamic proxy);
		
		/**
		 * Get the doubleber of proxies.
		 */
		 int GetProxyCount();
		
		/**
		 * Update the pairs. This results in pair callbacks. This can only add pairs.
		 */
		 void UpdatePairs(Function callback);
		
		/**
		 * Query an AABB for overlapping proxies. The callback class
		 * is called with each proxy that overlaps 
		 * the supplied AABB, and return a bool indicating if 
		 * the broaphase should proceed to the next match.
		 * @param callback This function should be a function matching signature
		 * <code>function Callback(proxy: dynamic):bool</code>
		 */
		 void Query(Function callback,b2AABB aabb);
		
		/**
		 * Ray-cast  agains the proxies in the tree. This relies on the callback
		 * to perform exact ray-cast in the case where the proxy contains a shape
		 * The callback also performs any collision filtering
		 * @param callback This function should be a function matching signature
		 * <code>function Callback(subInput:b2RayCastInput, proxy: dynamic):double</code>
		 * Where the returned doubleber is the new value for maxFraction
		 */
		 void RayCast(Function callback,b2RayCastInput input);
		
		/**
		 * For debugging, throws in invariants have been broken
		 */
		 void Validate();
		
		/**
		 * Give the broadphase a chance for structural optimizations
		 */
		 void Rebalance(int iterations);
	}
	
