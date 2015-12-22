/*
* Copyright (2006 as c)-2007 Adam Newgas
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
 * Applies an acceleration every frame, like gravity
 */
 class b2ConstantAccelController extends b2Controller
{	
	/**
	 * The acceleration to apply
	 */
	 b2Vec2 A = new b2Vec2(0.0,0.0);
	
	 @override 
		 void Step(b2TimeStep step){
		b2Vec2 smallA = new b2Vec2(A.x*step.dt,A.y*step.dt);
		for(b2ControllerEdge i=m_bodyList;i;i=i.nextBody){
			b2Body body = i.body;
			if(!body.IsAwake()) continue;
			//Am being lazy here
			body.SetLinearVelocity(new b2Vec2(
				body.GetLinearVelocity().x +smallA.x,
				body.GetLinearVelocity().y +smallA.y
				));
		}
	}
}

