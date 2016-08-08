/*
* Copyright (2009 as c) Adam Newgas http://www.boristhebrave.com
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

	 class Benchmark extends MovieClip {
		
		// Parameters of the test ///////////////
		 String broadphase = 
			"DynamicTree";
			//"SAP";
			
		 IBenchmark test = 
			//new PyramidBenchmark();
			//new RagdollBenchmark();
			new LotteryBenchmark();
			//new NullBenchmark();
			
		 bool doSleep = false;
		 int steps = 300;
		 int frequency = 60;
		 int velocityIterations = 10;
		 int positionIterations = 10;
		 bool warmStarting = true;
		 bool continuousPhysics = false;
		 bool preview = false;
		
		
		// Private variables //////////////////
		 b2World world;
		 double timeStep = 0.0;
		 int totalRuns = 10;
		 int runCount = 0;
		 List<double> data = new List<double>();
	 Benchmark()
		{
			 timeStep = 1.0 / frequency;
			// Gather the specifics of the test
				/*
			bool isDebug = Capabilities.isDebugger;
			String revision = r'$Rev: 92 $ $Date: 2009-12-26 17:08:24 +0000 (Sat, 26 Dec 2009) $';
			String version = b2Settings.VERSION;
			String playerVersion = Capabilities.version;
			String os = Capabilities.os;
			result = 
				<benchmark>
					<setup>
						<isDebug>{isDebug}</isDebug>
						<revision>{revision}</revision>
						<version>{version}</version>
						<playerVersion>{playerVersion}</playerVersion>
						<os>{os}</os>
					</setup>
					<parameters>
						<test>{test.Name()}</test>
						<broadphase>{broadphase}</broadphase>
						<doSleep>{doSleep}</doSleep>
						<steps>{steps}</steps>
						<frequency>{frequency}</frequency>
						<velocityIterations>{velocityIterations}</velocityIterations>
						<positionIterations>{positionIterations}</positionIterations>
						<warmStarting>{warmStarting}</warmStarting>
						<continuousPhysics>{continuousPhysics}</continuousPhysics>
					</parameters>
					{test.Details()}
				</benchmark>;
			*/
			this.addEventListener(Event.ENTER_FRAME, RunTest);
		
			if( preview )
			{
				totalRuns = steps;
				steps = 1;
				InitWorld();
			}
		}
		
		  void InitWorld()
		{
			world = new b2World(new b2Vec2(), doSleep);
			world.SetWarmStarting(warmStarting);
			world.SetContinuousPhysics(continuousPhysics);
			DynamicTreeBroadphase();
			test.Init(world);
		}
		
		  void RunTest([Event event=null])
		{
			if (runCount < totalRuns)
			{
				if(preview == false)
					InitWorld();
				data.add(DoRun().toDouble());
				runCount++;
				if (preview)DisplayScene();
			}else {
				removeEventListener(Event.ENTER_FRAME, RunTest);
				SummarizeResults();
				ReportResults();
			}
		}
		
		  void SummarizeResults()
		{
			int n = data.length;
			double sum = 0.0;
			double sum2 = 0.0;
			double t = 0.0;
			for(t in data)
			{
				sum += t;
				sum2 += t * t;
			}
			double average = sum / n;
			double sd = sqrt(sum2 / n - average * average);
			/*
			XML results = <results>
				<average>{average}</average>
				<sd>{sd}</sd>
				<totalMemory>{System.totalMemory}</totalMemory>
			</results>;
			for(t in data)
			{
				results.appendChild(<run>{t}</run>);
			}
			result.appendChild(results);
			 */
		}
		
		  int DoRun()
		{
			int start = /*getTimer()*/ (stage.juggler.elapsedTime*1000);
			for (int n = 0; n < steps; n++)
			{
				world.Step(timeStep, velocityIterations, positionIterations);
				world.ClearForces();
			}
			int end = /*getTimer()*/ (stage.juggler.elapsedTime*1000);
			return end - start;
		}
		
		  void DisplayScene()
		{
			// Show a snapshot of the world in the background.
			b2DebugDraw debugDraw = new b2DebugDraw();
			debugDraw.SetSprite(this);
			debugDraw.SetDrawScale(30.0);
			debugDraw.SetFillAlpha(0.3);
			debugDraw.SetLineThickness(1.0);
			debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
			world.SetDebugDraw(debugDraw);
			this.graphics.clear();
			world.DrawDebugData();
		}
		
		  void ReportResults()
		{
			DisplayScene();
			
			// Show the text
			TextField tf = new TextField();
			addChild(tf);
			tf.width = stage.stageWidth;
			tf.height = stage.stageHeight;
			//tf.text = result;
			
			// Copy to clipboard
			//System.setClipboard(result);
		}
		
		  void DynamicTreeBroadphase()
		{
			world.SetBroadPhase(new b2DynamicTreeBroadPhase());
		}
		
		  void SAPBroadphase()
		{
			b2AABB aabb  = new b2AABB();
			aabb.lowerBound.x = -100000.0;
			aabb.lowerBound.y = -100000.0;
			aabb.upperBound.x = 100000.0;
			aabb.upperBound.y = 100000.0;
			world.SetBroadPhase(new b2BroadPhase(aabb));
		}
		
	}
