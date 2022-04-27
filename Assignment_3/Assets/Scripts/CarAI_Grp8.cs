using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Purchasing.Extension;
using UnityEngine.Rendering;
using UnityEngine.SocialPlatforms;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI_Grp8 : MonoBehaviour
    {
	    // Lower level car stuff
        private CarController car;
        private Rigidbody rigidBody;

		// Terrain
		public GameObject terrain_manager_game_object;
		TerrainManager terrainManager;

		// Visibility graph
		public GameObject graphObject;
        private VisibilityGraph visGraph;
        // Planned Path
        private VisibilityGraph.Path path;
        
        // PD variables
        // Position gain
        public float kP = 1.5f;
        // Velocity gain
        public float kD = 1f;
        // The target speed of the car
        public float desiredSpeed = 20f;
	    bool reducedSpeedState = false;
        public float currentTopSpeed;
        private Vector3 oldTargetPosition;
        public int carState = 0;
        private Vector3 actionPosition = new Vector3();
        
        // Verboseness 
        public bool lasers = true;
        public bool verboseMode = true;
        public bool showCurvature = true;
        
        // Whether to enable path tracking
        public bool tracking;

        // Reynolds tracking
        // The radius of the path. When the target is within this distance of the path, we only steer forwards.
        public float pathRadius = 8f;
        // The coefficient for the vector to the next node when computing the desired velocity.
        public float nextNodeIncentive = 0.2f;
        // The coefficient of the velocity in computing the look ahead point.
        public float lookAheadVel = 0.5f;
        // The coefficient of the forward vector when computing the look ahead point.
        public float lookAheadForward = 1f;
        // Controls the momentum for the acceleration
        [Range(0, 1)] public float beta = 0.2f;
        private Vector3 oldDirection = Vector3.zero;
        public float lookAheadDistance = .5f;
        
        public float VelocityLookAhead = 20f;
        public float frictionCoefficient = 0.2f;
	    public float earlyBraking = 1f;
	    public float decelDerate = .5f;

        private float[] optimalVelocities;
        
        // ABS
        private Vector3 oldVelocity = Vector3.zero;
        private float oldForwardAcceleration = 0f;

        private void Start()
        {
            // get the car controller
            car = GetComponent<CarController>();
            rigidBody = GetComponent<Rigidbody>();
            terrainManager = terrain_manager_game_object.GetComponent<TerrainManager>();
			visGraph = graphObject.GetComponent<VisibilityGraph>();
            path = visGraph.path;

            optimalVelocities = CalcOptimalVel();
            currentTopSpeed = desiredSpeed;
        }

        private void FixedUpdate()
        {
	        // State Machine
	        // Start(0)
	        // Reorient(1)
	        //		Turning reverse (1)
	        //		turning forward (11)
	        // Drive(2) - currently forward and reverse are treated the same which can be a prob
	        //		Forward
	        //		Reverse
	        // Fault(3)
	        //		Simple reverse
	        // Goal(4)
	        //		Handbrake!

	        // Informaition to determine current state
	        var laser_spread = Mathf.PI / 6;
	        var rangeResults = DoDistanceSensing(laser_spread);
	        var collisionResults = DetectCollision(rangeResults, laser_spread);
	        var distToGoal = (terrainManager.myInfo.goal_pos - car.transform.position).magnitude;
	        var headingError = CalcHeadingError();
	        var currentPos = car.transform.position + 2 * car.transform.forward;
	        
	        // Parameters for different states
	        //(1) Reorient
	        float engageReorient = Mathf.PI / 2;
	        float disengageReorient = Mathf.PI / 6;
	        //(3) fault
	        float fautDistThresh = 3f;
	        //(4) goal
	        float distToGoalThreshold = 8f;
			
	        // Determine state
	        // Completed course
	        if (distToGoal <= distToGoalThreshold)
	        {
		        carState = 4;
	        }
	        
	        if (carState == 0)
	        {
		        // Initial position requires reorientation (0) -> (1)
		        if (Mathf.Abs(headingError) >= engageReorient)
		        {
			        carState = 1;
			        Debug.Log("0 -> 1");
		        }
		        // Initial position requires reorientation (0) -> (2)
		        else if (Mathf.Abs(headingError) < engageReorient)
		        {
			        carState = 2;
			        Debug.Log("0 -> 2");
		        }
	        }
	        else if (carState == 1)
	        {
		        Debug.Log(collisionResults[1]);
		        // Move from Reorientation to drive (1) -> (2)
		        if (Mathf.Abs(headingError) <= disengageReorient)
		        {
			        carState = 2;
			        Debug.Log("1 -> 2");
		        }
		        // move from reverse reorient to forward reorient (1) -> (11)
		        else if (collisionResults[1] == true)
		        {
			        carState = 11;
			        Debug.Log("1 -> 11");
		        }
	        }
	        else if (carState == 11)
	        {
		        // Move from Reorientation to drive (11) -> (2)
		        if (Mathf.Abs(headingError) <= disengageReorient)
		        {
			        carState = 2;
			        Debug.Log("11 -> 2");
		        }
		        else if (collisionResults[0] == true)
		        {
			        carState = 3;
			        Debug.Log("11 -> 3");
		        }
	        }
	        else if (carState == 2)
	        {
		        // Move from 2 to 11 if we are far from the path and in the wrong orientation
		        var (closestPath, projectedCar, nodeIndices) = path.FindClosestLineSegment(currentPos, true);
		        if (Vector3.Distance(currentPos, projectedCar) > 4 && Mathf.Abs(headingError) >= disengageReorient)
		        {
			        carState = 11;
			        Debug.Log("2 -> 11");
		        }
		        // Move from driving to fault (2) -> (3)
		        // Ignore rear collisions
		        if (carState == 2 & (collisionResults[0] || collisionResults[2] || collisionResults[3]))
		        {
			        // Only front and side collision should be possible from the drive state
			        carState = 3;
			        // Save current position
			        actionPosition = rigidBody.position;
			        Debug.Log("2 -> 3");
		        }
	        }
	        else if (carState == 3)
	        {
		        // Move from Reorientation to drive (3) -> (2)
		        // todo improve
		        var distFromFault = (actionPosition - rigidBody.position).magnitude;
		        if ((Mathf.Abs(headingError) <= disengageReorient && distFromFault >= fautDistThresh) || collisionResults[1])
		        {
			        Debug.Log("3 -> 11");
			        carState = 11;
		        
			        if (verboseMode)
			        {
				        Debug.Log("3 -> 2");
				        Debug.Log("front Collision: " + collisionResults[0] + "  Rear Collision: " + collisionResults[1]);
				        Debug.Log("heading Error: " + headingError);
				        Debug.Log("Dist From Fault: " + distFromFault);
			        }
		        }
	        }

	        // Act according to state
			switch (carState)
			{
				case 1: // Reorient reverse
					//Debug.Log("front Collision: " + collisionResults[0] + "  Rear Collision: " + collisionResults[1]);
					//Debug.Log("front distance: " + rangeResults[0] + "  Rear distance: " + rangeResults[1]);
					car.Move(headingError, 0, -0.5f, 0f);
					break;
				
				case 11: // Reorient forward
					//Debug.Log("front Collision: " + collisionResults[0] + "  Rear Collision: " + collisionResults[1]);
					//Debug.Log("front distance: " + rangeResults[0] + "  Rear distance: " + rangeResults[1]);
					car.Move(-1 * headingError, 0.5f, 0f, 0f);
					break;

				case 2: // Normal driving
					UsePdTracker(currentPos);
					break;

				case 3: // Fault
					car.Move(headingError, 0, -0.5f, 0f);
					break;
				
				case 4: // Goal reached
					car.Move(0f, 0f, 0f, 1f);
					break;

				default: // In case I forgot something
					UsePdTracker(currentPos);
					// UsePurePursuit(transform.position);
					break;
			}

			if (verboseMode)
			{
				Debug.Log("Collision:" + collisionResults);
				Debug.Log("headingError: " + headingError);
			}
        }

        private void UsePdTracker(Vector3 currentPos)
        {
            // Get the steering direction
            var (closestPath, projectedCar, nodeIndices) = path.FindClosestLineSegment(currentPos, true);
            var (closestPathDiscrete, _, _) = path.FindClosestLineSegment(currentPos, false);
            // var targetVector = currentPos + lookAheadForward * transform.forward + lookAheadVel * my_rigidbody.velocity;
            // var (closestPathTarget, projectedTarget) = FindClosestLineSegment(targetVector, path);

            var forwardDirection = ComputeLookAhead(nodeIndices[1], projectedCar);
            //Debug.DrawLine(currentPos, currentPos + forwardDirection*5, Color.cyan);

            
            // Find target node
            var targetNode = closestPathDiscrete[1];
            PdController(currentPos, projectedCar, transform.forward, targetNode, nodeIndices[1]);
        }
        
        private void PdController(Vector3 currentPosition, Vector3 targetPosition, Vector3 forwardDirection, Vector3 targetVisibilityNode, int nextNodeIndex)
        {
            // keep track of target velocity
            Vector3 targetVelocity = (targetPosition - oldTargetPosition) / Time.fixedDeltaTime;
            oldTargetPosition = targetPosition;

            var requestedSpeed = HandleAccelerationByCurvature(nextNodeIndex);
	        //Debug.LogFormat("requested speed = {0}", requestedSpeed);
            
            // Steering
            var positionError = targetPosition - currentPosition;
            var velocityError = targetVelocity - rigidBody.velocity;
            var desiredDirection = kP * positionError + kD * velocityError;
            var steering = Vector3.Dot(desiredDirection, transform.right);
            //Debug.LogFormat("speed forward:{0}", forwardSpeed);
            //Debug.LogFormat("speed error:{0}", velocityError);

            // Acceleration
            // requestedSpeed is determined above
            var forwardSpeed = requestedSpeed * forwardDirection;
            var forwardSpeedError = forwardSpeed - rigidBody.velocity;
            var rawAcceleration = Vector3.Dot(forwardSpeedError, transform.forward);

            var acceleration = Tcs(rawAcceleration, steering);
            // var deceleration = Abs(rawAcceleration);
            
            //Debug.Log("Target_node_id: " + target_node_id);
            //Debug.Log("Target_dist: " + target_dist + "  Brake_dist: " + brake_dist);
            //Debug.Log("current_speed: " + current_speed + "  turn_speed: " + turn_speed + "  turn_speed_old: " + turn_speed_old);
            //Debug.Log("turn_angle: " + turn_angle + " acceleration: " + acceleration);

            //Debug.DrawLine(targetPosition, targetPosition + targetVelocity, Color.red);
            //Debug.DrawLine(currentPosition, currentPosition + rigidBody.velocity, Color.blue);
            Debug.DrawLine(currentPosition, currentPosition + positionError, Color.black);
            Debug.DrawLine(currentPosition, currentPosition + Vector3.ClampMagnitude(forwardSpeedError, 4f), Color.blue);

            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            if (tracking)
            {
                car.Move(steering, acceleration, acceleration, 0f);
            }
            oldForwardAcceleration = rawAcceleration;
        }
        
        private float HandleAccelerationLegacy(Vector3 currentPosition, int nextNodeIndex, Vector3 targetVisibilityNode)
        {
	        /// Turn angle used to define a turn speed
            
            // Computed quantities
            float target_dist;
            float turn_angle;
            float turn_speed;
            float turn_speed_old = 0;
	        float current_speed;
	        float brake_dist;
            float requestedSpeed;
            
            // Parameters
            float reducedSpeedResetDist = 20;
            float turn_ang_threshold = Mathf.PI / 5; // turns under threshold will have no limiting applied
            float speed_0 = 35f; // speed for 0 rad turn NOTE: not actually applied to such turns due to above threshold
            float speed_pi_2 = 12.5f; // speed for pi/2 turn //10
            float min_speed = 10; // min speed
            
            // Estimates for kinematic equations
            // Mass: 1160, wheel radius = 0.335
            // engine torque: 2500, brake torque:20000
            // Kinematic Eq: v_f^2 = v_i^2 + 2a(x_f - x_i)
            // (v_f^2 - v_i^2) / 2(x_f - x_i) = a
            // float accel_derate = 0.1f;
            // float max_accel = accel_derate * (2500f / 0.335f) / 1160f;
            float decel_derate = .25f;
            float max_decel = decel_derate * (20000f / 0.335f) / 1160f;
            
            // START OF NEW - this should be moved only needs to happen at start
            // Instead of only taking into account the next node we will take into account all nodes
            // requested speed at given node will allow for requested speed at new node to be achievable
            List<int> turn_node_list = new List<int>();
            List<float> turn_angle_list = new List<float>();
            List<float> dist_2_prev_list = new List<float>();
            List<float> ideal_speed_list = new List<float>();	// ideal speeds
            List<float> better_speed_list = new List<float>();	// list of speeds with now approach
            
            var node_list = path.path_ids;
            var start_index = node_list[0];
            
            // turning node list: node ids along path starting from the end of the path
            for (int i = 1; i < node_list.Count - 1; i++) turn_node_list.Add(node_list[i]);
            turn_node_list.Reverse();
	        
            // turn angle list: list of angles at nodes along the path
            foreach (var index in turn_node_list)
            {
	            turn_angle = Mathf.Deg2Rad * Mathf.DeltaAngle(Mathf.Rad2Deg * visGraph.nodes[index].orientation, 
																Mathf.Rad2Deg * visGraph.nodes[index].child_orientation);
	            turn_angle_list.Add(Mathf.Abs(turn_angle));
            }
			
            // Distance to the previous node
            for (int i = 0; i < turn_node_list.Count - 1; i++)
            {
	            var index_a = turn_node_list[i];
	            var index_b = turn_node_list[i+1];
	            
	            dist_2_prev_list.Add(visGraph.path.node_dist(visGraph.nodes[index_a], visGraph.nodes[index_b]));
            }
            
            // Add the first turn to start dist
            // var index_a = turn_node_list[turn_node_list.Count - 1];
            // dist_2_prev_list.Add(visGraph.path.node_dist(visGraph.nodes[index_a], visGraph.nodes[start_index]));

            // Ideal turning speed list: this is just based of the arb function that mad angle to speed
			foreach (var angle in turn_angle_list)
			{
				turn_speed = (float)(angle * (speed_pi_2 - speed_0) / (0.5 * Mathf.PI) + speed_0);
				ideal_speed_list.Add(turn_speed);
			}
			
			// Calculate the better turn speeds that will take into acount the following turns
			// The last turn speed is the same as in the ideal case
			better_speed_list.Add(ideal_speed_list[0]);

			for (int i = 1; i < ideal_speed_list.Count; i++)
			{
				// Calculate max speed at give node to allow for the next node target to be reachable
				var max_speed = Mathf.Sqrt(Mathf.Pow(better_speed_list[i - 1], 2) +
				                       (2 * max_decel * dist_2_prev_list[i - 1]));
				
				better_speed_list.Add(Mathf.Min(max_speed, ideal_speed_list[i]));
			}
			
			/// START OF OLD
            // Find distance to target node
            target_dist = Vector3.Distance(currentPosition,targetVisibilityNode);
            
            // Test for reset of currentTopSpeed
            if (target_dist >= reducedSpeedResetDist)
            {
	            reducedSpeedState = false;
	            currentTopSpeed = desiredSpeed;
            }
            
            // Find turn angle at next node
            var target_node_id = path.node_id_from_coord(targetVisibilityNode);
            turn_angle = Mathf.Deg2Rad * Mathf.DeltaAngle(Mathf.Rad2Deg * visGraph.nodes[target_node_id].orientation, 
														   Mathf.Rad2Deg * visGraph.nodes[target_node_id].child_orientation);
	        turn_angle = Mathf.Abs(turn_angle);
	        
	        
	        // current speed
	        current_speed = Vector3.Dot(rigidBody.velocity, transform.forward);
	        
	        if (turn_angle >= turn_ang_threshold)
	        {
		        // OLDER WAY
		        //float turn_speed = desiredSpeed - desiredSpeed * Mathf.Clamp(2 * turn_angle / Mathf.PI, 0, 0.5f);
		        turn_speed_old = (float)(turn_angle * (speed_pi_2 - speed_0) / (0.5 * Mathf.PI) + speed_0);
		        //turn_speed = Mathf.Clamp(turn_speed, min_speed, desiredSpeed);
		        
		        // NEWER WAY
		        var turn_id = turn_node_list.IndexOf(target_node_id);
		        if (turn_id > 0)
		        {
			        turn_speed = better_speed_list[turn_id];
			        //Debug.Log("Using Better method! turn ID: " + turn_id);
			        //Debug.Log("Ideal: " + ideal_speed_list[turn_id] + "  Better: " + better_speed_list[turn_id]);
		        }
		        else
		        {
			        // rever to old way
			        turn_speed = (float)(turn_angle * (speed_pi_2 - speed_0) / (0.5 * Mathf.PI) + speed_0);
		        }
		        
		        // Calculate required braking distance
		        brake_dist = (Mathf.Pow(turn_speed, 2) - Mathf.Pow(current_speed, 2)) / (-2 * max_decel);

		        if (target_dist <= brake_dist)
		        {
			        if (target_dist < 1f)
			        {
				        // Entering a braking maneuver
				        reducedSpeedState = true;
				        currentTopSpeed = 7f;
			        }
			        
			        requestedSpeed = Mathf.Min(turn_speed,currentTopSpeed);
			        Debug.DrawLine(currentPosition,targetVisibilityNode, Color.yellow,100f);
		        }
		        else
		        {
			        requestedSpeed = currentTopSpeed;
			        Debug.DrawLine(currentPosition,targetVisibilityNode, Color.magenta,0.05f);
		        }
	        }
	        else
	        {
		        brake_dist = -1f;
		        turn_speed = currentTopSpeed;
		        requestedSpeed = currentTopSpeed;
		        //Debug.DrawLine(currentPosition,targetVisibilityNode, Color.cyan,0.05f);
	        }
	        
	        ///

	        requestedSpeed = optimalVelocities[nextNodeIndex];
	        return requestedSpeed;
        }

        private float HandleAccelerationByCurvature(int nextNodeIndex)
        {
	        // Estimates for kinematic equations
	        // Mass: 1160, wheel radius = 0.335
	        // engine torque: 2500, brake torque:20000
	        // Kinematic Eq: v_f^2 = v_i^2 + 2a(x_f - x_i)
	        // (v_f^2 - v_i^2) / 2(x_f - x_i) = a
	        // float accelDerate = 0.1f;
	        // float maxAccel = accelDerate * (2500f / 0.335f) / 1160f;
	        float maxDecel = decelDerate * (20000f / 0.335f) / 1160f;
	        
	        var requestedSpeed = 0f;
	        
	        var maxLookAhead = rigidBody.velocity.magnitude;
	        var maxLookAheadVertices = Convert.ToInt32(maxLookAhead / visGraph.vertexSpacing);
	        
	        var lookAheadInterval = 1;
	        var verticesLookAheadInterval = Convert.ToInt32(lookAheadInterval / visGraph.vertexSpacing);

	        float velAtTargetPosition = car.MaxSpeed;
	        var brake = false;
	        // Look at intervals forward
	        for (int i = 0; i < maxLookAheadVertices; i += verticesLookAheadInterval)
	        {
		        var offset = Math.Min(nextNodeIndex + i, optimalVelocities.Length-1);
		        var count = Math.Min(i+verticesLookAheadInterval, (optimalVelocities.Length - 1) - offset);
		        if (offset+count > optimalVelocities.Length -1) break;
		        var velSegment = new ArraySegment<float>(optimalVelocities, offset, count);
		        // Find the minimum optimal velocity in this interval
		        var (argmin, min) = MinVelInSegment(velSegment.ToList());
		        // Calculate required braking distance
		        var brakeDist = (Mathf.Pow(min, 2) - Mathf.Pow(rigidBody.velocity.magnitude, 2)) / (-2 * maxDecel);
		        if (brakeDist > argmin * visGraph.vertexSpacing)
		        {
			        velAtTargetPosition = min;
			        brake = true;
			        //Debug.LogFormat("Braking at interval {0}", (i/verticesLookAheadInterval)+1);
			        break;
		        }
	        }

	        //var velAtTargetPosition = optimalVelocities[Math.Min(nextNodeIndex + maxLookAheadVertices, optimalVelocities.Length-1)];
	        
	        // Debugging
	        if (showCurvature)
	        {
		        Debug.DrawLine(transform.position,
			        path.VectorizedBezierPath[
				        Math.Min(optimalVelocities.Length - 1, nextNodeIndex + maxLookAheadVertices)], Color.black);
		        for (var i = 0; i < path.VectorizedBezierPath.Count; i++)
		        {
			        Debug.DrawLine(
				        path.VectorizedBezierPath[Math.Min(i + maxLookAheadVertices, optimalVelocities.Length - 1)],
				        path.VectorizedBezierPath[Math.Min(i + maxLookAheadVertices, optimalVelocities.Length - 1)] +
				        Vector3.up *
				        optimalVelocities[Math.Min(i + maxLookAheadVertices, optimalVelocities.Length - 1)],
				        Color.cyan);
		        }
	        }




	        if (brake)
	        {
		        requestedSpeed = velAtTargetPosition;
	        }
	        else
	        {
		        requestedSpeed = car.MaxSpeed;
	        }

	        return requestedSpeed;
        }

        private (int, float) MinVelInSegment(IList<float> segment)
        {
	        var min = Mathf.Infinity;
	        var argmin = 0;
	        for (int i = 0; i < segment.Count; i++)
	        {
		        if (segment[i] < min)
		        {
			        min = segment[i];
			        argmin = i;
		        }
	        }

	        return (argmin, min);
        }
        
        private void UsePurePursuit(Vector3 currentPosition)
        {
            var (direction, b) = SteerPath();
            PurePursuit(currentPosition, b, direction);
        }
        
        private void PurePursuit(Vector3 currentPosition, Vector3 nextNodePosition, Vector3 steerDirection)
        {
            // Momentum
            var newDirection = beta * oldDirection + (1 - beta) * steerDirection;
            oldDirection = newDirection;
            // Compute the desired velocity by scaling the steering steerDirection by the target velocity
            // Add vector from position to next node,  incentivizing traveling towards it.
            Vector3 desiredVelocity = newDirection.normalized * desiredSpeed + nextNodeIncentive * transform.forward;//(nextNodePosition - currentPosition);
            Vector3 velocityError = desiredVelocity - rigidBody.velocity;
            Vector3 desiredAcceleration = kD * velocityError;
	        
            float steering = Vector3.Dot(newDirection, transform.right);
            float acceleration = Vector3.Dot(desiredAcceleration, transform.forward);
	        
            if (tracking)
            {
                car.Move(steering, acceleration, acceleration, 0f);
            }

            Debug.DrawLine(currentPosition, currentPosition + desiredAcceleration, Color.black);
            Debug.DrawLine(currentPosition, currentPosition + steerDirection, Color.white);
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
        }
        
        private (Vector3, Vector3) SteerPath()
        {
            var currentPos = transform.position;

            var (closestPathCar, projectedCar, _) = path.FindClosestLineSegment(currentPos, true);
            var bCarDist = (closestPathCar[1] - currentPos).magnitude;
            var carPathDist = (projectedCar - currentPos).magnitude;
            var targetVector = currentPos + lookAheadForward * transform.forward + lookAheadVel * rigidBody.velocity;
            if (carPathDist != 0)
            {
                var targetVectorScaling = 1.0f;
                if (bCarDist > 20 && carPathDist < 2) ;
                {
                    carPathDist = Mathf.Clamp(carPathDist, 0.0f, 1.0f);
                    targetVectorScaling = 3.0f; // / carPathDist; // 0 - 5
                    //targetVectorScaling *= bCarDist; // suppress when < 20
                }

                // targetVector += transform.forward * targetVectorScaling;
            }
            Debug.LogFormat("CAR PATH DIST: {0}", carPathDist);
            Debug.LogFormat("CAR b DIST: {0}", bCarDist);
            var (closestPathSegment, projectedTarget, _) = path.FindClosestLineSegment(targetVector, true);
            var a = closestPathSegment[0];
            var b = closestPathSegment[1];
            // Target position relative to the first node of the path segment
            var relativeTarget = targetVector - a;
            // Compute the distance vector between the target and the projected point on the path.
            // Might be used later for path radius check
            var targetToPath = relativeTarget - projectedTarget;
            var diff = targetVector - projectedTarget;
            Debug.DrawLine(projectedTarget, projectedTarget + diff.normalized * pathRadius, Color.black);
            if (targetToPath.magnitude < pathRadius)
            {
                return (Vector3.forward, b);
            }
            var steeringDirection = projectedTarget - currentPos;
            return (steeringDirection, b);
        }
        
        // Returns the normalized look ahead vector which is the direction of node that is in the "forward" direction
        // along the bezier path, relative to the current position.
        Vector3 ComputeLookAhead(int nextNodeIndex, Vector3 projectedPoint)
        {
	        var maxLookAheadIndex = Convert.ToInt32(Math.Ceiling(lookAheadDistance / visGraph.vertexSpacing));
	        var pathLength = path.VectorizedBezierPath.Count;

	        var lookAheadNode = path.VectorizedBezierPath[Math.Min(pathLength -1, nextNodeIndex + maxLookAheadIndex)];
	        var direction = lookAheadNode - projectedPoint;
	        return direction;
	        
	        var visitedNode = path.VectorizedBezierPath[nextNodeIndex];
	        for (var i = 0; i <= maxLookAheadIndex; i++)
	        {
		        visitedNode = path.VectorizedBezierPath[Math.Min(pathLength-1, nextNodeIndex + i)];
		        var dist = (visitedNode - projectedPoint).magnitude;
		        if (dist >= lookAheadDistance) break;
	        }

	        var lookAheadDirection = visitedNode - projectedPoint;
	        
	        return lookAheadDirection.normalized;
        }

        private float[] CalcOptimalVel()
        {
	        float[] optimalVel = new float[path.Curvatures.Length];
	        
	        int lookAheadVertices = Convert.ToInt32(earlyBraking / visGraph.vertexSpacing);
	        for (int i = 0; i < optimalVel.Length; i++)
	        {
		        // Radius of circle with same curvature at the desired point
		        var rho = 1 / path.Curvatures[Mathf.Min(optimalVel.Length - 1,i + lookAheadVertices)];
		        optimalVel[i] = Mathf.Clamp(Mathf.Sqrt(Physics.gravity.magnitude * frictionCoefficient * rho), - car.MaxSpeed, car.MaxSpeed);
		        continue;
		        // Todo are vertices this evenly spaced?
		        var end = i + lookAheadVertices;
		        end = Math.Min(end, optimalVel.Length - 1);
		        var segment = new ArraySegment<float>(path.Curvatures, i, end-i);
		        var cumulativeCurvature = segment.Sum() / VelocityLookAhead;
		        optimalVel[i] = 3 / cumulativeCurvature; 
	        }

	        return optimalVel;
        }

        private List<float> DoDistanceSensing(float angular_spread)
        {
	        // Currently using 8 range finders: forward, backward, Right and Left
	        // Angular spread defines the angle in radian from the forward/backward axis
	        // The transform.(forward and right) directions can sometimes contain an unwanted y component
	        var trueForward = new Vector3(transform.forward.x, 0, transform.forward.z);
	        var trueRight = new Vector3(transform.right.x, 0, transform.right.z);
	        var newDirection1 = Vector3.RotateTowards(trueForward, trueRight, angular_spread, 0f);
	        var newDirection2 = Vector3.RotateTowards(trueForward, trueRight, -angular_spread, 0f);

	        Vector3[] lookDirections = new Vector3[]
	        {
		        trueForward, -1 * trueForward,
		        trueRight, -1 * trueRight,
		        newDirection1, -1 * newDirection1,
		        newDirection2, -1 * newDirection2
	        };

	        List<float> rangeResults = new List<float>();
	        
	        foreach (var laser_vect in lookDirections)
	        {
		        RaycastHit hit;
		        float maxRange = 50f;
		        if (Physics.Raycast(transform.position, laser_vect, out hit, maxRange))
		        {
			        Vector3 closestObstacleInFront = laser_vect * (hit.distance - 1f);
			        if (lasers)
			        {
				        Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);    
			        }

			        // Add range reading to the list of results
			        rangeResults.Add(hit.distance);
		        }
		        else
		        {
			        // Add negative (out of range) reading to results
			        rangeResults.Add(-1f);
		        }
	        }

	        if (verboseMode)
	        {
		        Debug.Log("Forward: " + rangeResults[0] + " Left: " + rangeResults[3] + " Angled: " + rangeResults[4]);
	        }

	        return rangeResults;
        }

        private List<bool> DetectCollision(List<float> rangeResults, float laser_spread)
        {
	        // Return Collision states of car, true indicates collision
	        // Front, Rear, Right, Left
	        // Is there a reason to include the angles range readings?
	        float front_dim = 2.25f; // Actual ~ 2.15
	        float side_dim = 1.25f;
	        float rear_dim = 2.60f; // Actual ~ 2.47
	        List<bool> collisionState = new List<bool>() {false, false, false, false};

	        // Front
	        if (rangeResults[0] != -1 && rangeResults[0] <= front_dim) collisionState[0] = true;
	        if (rangeResults[4] != -1 && rangeResults[4] * Mathf.Cos(laser_spread) <= front_dim) collisionState[0] = true;
	        if (rangeResults[6] != -1 && rangeResults[6] * Mathf.Cos(laser_spread) <= front_dim) collisionState[0] = true;

	        if (verboseMode)
	        {
		        Debug.Log("collision test: " + rangeResults[4] * Mathf.Cos(laser_spread) + "  font collision: " + collisionState[0]);    
	        }
	        
	        // Rear
	        if (rangeResults[1] != -1 && rangeResults[1] <= rear_dim) collisionState[1] = true;
	        if (rangeResults[5] != -1 && rangeResults[5] * Mathf.Cos(laser_spread) <= rear_dim) collisionState[1] = true;
	        if (rangeResults[7] != -1 && rangeResults[7] * Mathf.Cos(laser_spread) <= rear_dim) collisionState[1] = true;
	        
	        // Right
	        if (rangeResults[2] != -1 && rangeResults[2] <= side_dim) collisionState[2] = true;
	        
	        // Left
	        if (rangeResults[3] != -1 && rangeResults[3] <= side_dim) collisionState[3] = true;

	        return collisionState;
        }

        private float CalcHeadingError()
        {
	        // current Heading w/ y component removed
	        // Also calculate the angle of the heading on the x-z plane
	        var currentHeading = transform.forward;
	        currentHeading.y = 0f;
	        var currentHeadingAngle = Mathf.Atan2(currentHeading.z, currentHeading.x);
	        
	        // tangential direction at look ahead distanceon the path
	        // Also calculate the angle of the path dforward direction on the x-z plane
	        var (closestPath, projectedCar, nodeIndices) = path.FindClosestLineSegment(transform.position, true);
	        var forwardDirection = ComputeLookAhead(nodeIndices[1], projectedCar);
	        var forwardDirectionAngle = Mathf.Atan2(forwardDirection.z, forwardDirection.x);
	        
	        // Calculate the 
	        float error = Mathf.Deg2Rad * Mathf.DeltaAngle(
		        Mathf.Rad2Deg * currentHeadingAngle, 
		        Mathf.Rad2Deg * forwardDirectionAngle);

	        return error;
        }

        private float Tcs(float acceleration, float steering)
        {
	        if (Mathf.Abs(steering) > .5)
	        {
		        return acceleration;
	        };
	        var clampedAcceleration = Mathf.Clamp(acceleration, -1, 1);
	        var approxForwardAcceleration = Vector3.Dot(rigidBody.velocity - oldVelocity, transform.forward);
	        var diff = oldForwardAcceleration - approxForwardAcceleration;
	        var forwardSpeed = Vector3.Dot(rigidBody.velocity, transform.forward);
	        var accelerationCoeff = Mathf.InverseLerp(0f, 5f, rigidBody.velocity.magnitude);
	        if (Math.Abs(diff) > 0 && forwardSpeed > 0) return clampedAcceleration * accelerationCoeff;
	        return acceleration;
        }

        private float Abs(float acceleration)
        {
	        if (acceleration > 0) return acceleration;
	        var clampedAcceleration = Mathf.Clamp(acceleration, -1, 0);
	        var forwardSpeed = Vector3.Dot(rigidBody.velocity, transform.forward);
	        var interpolatedSpeed = Mathf.InverseLerp(0f, 10f, forwardSpeed);
	        var deceleration = clampedAcceleration * (1-interpolatedSpeed);
	        Debug.Log(deceleration);
	        return deceleration;
        }
    }
}

