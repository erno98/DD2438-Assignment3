using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using UnityEngine.Purchasing.Extension;
using UnityEngine.Rendering;
using UnityEngine.SocialPlatforms;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI_merged : MonoBehaviour
    {
        // Variables for Car
        private CarController car;
        private Rigidbody rigidBody;

        // Variables for Terrain
        public GameObject terrain_manager_game_object;
        TerrainManager terrainManager;
        private CarConfigSpace ObstacleSpace;

        // Variables for Path Planning on Roads
        public GameObject my_goal_object;
        private GraphEmbedding road_map;
        private PathPlanner planner = new PathPlanner();
        public GameObject[] friends;
        private int CarNumber, wayPoint, path_index;
        private float RoadOffset = 4f;
        private List<Vector3> my_path;


        // Variables for path & driving
        private float acceleration, max_speed;
        private bool MazeComplete;
        private float mazeTimer;

        // Variables for Collision detection
        private float angle, spacing, checkRadius;
        private int checkDensity;

        // Variables for Handshakes
        enum handshake { follow, wait_to_follow, cross, wait_to_cross, none }
        private handshake my_handshake = handshake.none;
        private int following, wait_follow, let_cross, wait_cross;
        private List<GameObject> CloseByFriends;

        private bool ReducedCars = false;

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

        private float vertexSpacing = 0.1f;
        private float[] Curvatures;

        private void Start()
        {
            // Initialize Variables
            Time.timeScale = 1;
            mazeTimer = 0f;
            MazeComplete = false;
            path_index = 1;
            max_speed = 40;

            // get the car controller
            car = GetComponent<CarController>();
            rigidBody = GetComponent<Rigidbody>();
            terrainManager = terrain_manager_game_object.GetComponent<TerrainManager>();


            // Get CarNumber
            friends = GameObject.FindGameObjectsWithTag("Car");
            for (int i = 0; i < friends.Length; i++)
            {
                if (friends[i].transform.position == car.transform.position)
                {
                    CarNumber = i;
                    break;
                }
            }

            // Debugger
            if (true)
            {
                ReducedCars = true;
            }

            my_path = new List<Vector3>();
            wayPoint = 0;
            path_index = 0;
            if (ReducedCars)
            {
                // Get RoadMap
                road_map = new GraphEmbedding(terrainManager.terrain_filename, 1, 0.5f);
                // Get Path
                List<Vector3> initial_path = planner.plan_path(transform.position, my_goal_object.transform.position, road_map, terrainManager.myInfo);
                // Get Road Path
                my_path.Add(transform.position);
                for (int i = 1; i < initial_path.Count - 1; i++)
                {
                    Orientation currOrientation = getPathDirection(initial_path[i], initial_path[i + 1]);
                    Orientation prevOrientation = getPathDirection(initial_path[i - 1], initial_path[i]);
                    my_path.Add(AddRoadLayout(initial_path[i], currOrientation, prevOrientation, RoadOffset));
                    //if(CarNumber==30) Debug.Log("S: " + initial_path[i] + " D: " + initial_path[i+1] + " " + currOrientation + " new:" + AddRoadLayout(initial_path[i], currOrientation, prevOrientation, RoadOffset));
                }
                my_path.Add(my_goal_object.transform.position);
                my_path = pathUpsampling(my_path, 8);
            }

            // Draw Blue road grid
            if (CarNumber == 0) road_map.draw_embedding(terrainManager.myInfo);

            // Grp 8 code
            //var curvatures = CalculateCurvatures();
            //Curvatures = CalculateAverageCurvatures(curvatures);
            //optimalVelocities = CalcOptimalVel();
            currentTopSpeed = desiredSpeed;

        }

        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;

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
                var (closestPath, projectedCar, nodeIndices) = FindClosestLineSegment(currentPos);
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
                    //UsePdTracker(currentPos);
                    UsePurePursuit(currentPos);
                    break;

                case 3: // Fault
                    car.Move(headingError, 0, -0.5f, 0f);
                    break;

                case 4: // Goal reached
                    car.Move(0f, 0f, 0f, 1f);
                    break;

                default: // In case I forgot something
                    //UsePdTracker(currentPos);
                    UsePurePursuit(currentPos);
                    break;
            }

            if (verboseMode)
            {
                Debug.Log("Collision:" + collisionResults);
                Debug.Log("headingError: " + headingError);
            }


            if (Vector3.Distance(car.transform.position, my_goal_object.transform.position) < 4f)
            {
                Debug.Log("Goal reached for car: " + CarNumber + " at time " + mazeTimer);

                //UnityEditor.EditorApplication.isPlaying = false;
            }

        }


        /// FUNCTIONS

        // FUNC: Get direction of path
        private Orientation getPathDirection(Vector3 a, Vector3 b)
        {
            // a - source , b - destination
            if (a.x == b.x) // going up or down
            {
                return a.z > b.z ? Orientation.Down : Orientation.Up;
            }
            if (a.z == b.z) // going left or  right
            {
                return a.x > b.x ? Orientation.Left : Orientation.Right;
            }
            if (a.x > b.x) // going left
            {
                return a.z > b.z ? Orientation.DownLeft : Orientation.UpLeft;
            }
            if (a.x < b.x) // going right
            {
                return a.z > b.z ? Orientation.DownRight : Orientation.UpRight;
            }
            return Orientation.Up;

        }

        // FUNC: Add Road offset based on direction of car path
        private Vector3 AddRoadLayout(Vector3 a, Orientation currOrient, Orientation prevOrient, float RoadOffset)
        {
            Vector3 new_point = new Vector3();
            switch (currOrient)
            {
                case Orientation.Left:
                    switch (prevOrient)
                    {
                        case Orientation.Up:
                            new_point = new Vector3(a.x + RoadOffset, a.y, a.z + RoadOffset);
                            break;
                        case Orientation.Down:
                            new_point = new Vector3(a.x - RoadOffset, a.y, a.z + RoadOffset);
                            break;
                        default:
                            new_point = new Vector3(a.x, a.y, a.z + RoadOffset);
                            break;
                    }
                    break;
                case Orientation.Right:
                    switch (prevOrient)
                    {
                        case Orientation.Up:
                            new_point = new Vector3(a.x + RoadOffset, a.y, a.z - RoadOffset);
                            break;
                        case Orientation.Down:
                            new_point = new Vector3(a.x - RoadOffset, a.y, a.z - RoadOffset);
                            break;
                        default:
                            new_point = new Vector3(a.x, a.y, a.z - RoadOffset);
                            break;
                    }
                    break;
                case Orientation.Up:
                    switch (prevOrient)
                    {
                        case Orientation.Left:
                            new_point = new Vector3(a.x + RoadOffset, a.y, a.z + RoadOffset);
                            break;
                        case Orientation.Right:
                            new_point = new Vector3(a.x + RoadOffset, a.y, a.z - RoadOffset);
                            break;
                        default:
                            new_point = new Vector3(a.x + RoadOffset, a.y, a.z);
                            break;
                    }
                    break;
                case Orientation.Down:
                    switch (prevOrient)
                    {
                        case Orientation.Left:
                            new_point = new Vector3(a.x - RoadOffset, a.y, a.z + RoadOffset);
                            break;
                        case Orientation.Right:
                            new_point = new Vector3(a.x - RoadOffset, a.y, a.z - RoadOffset);
                            break;
                        default:
                            new_point = new Vector3(a.x - RoadOffset, a.y, a.z);
                            break;
                    }
                    break;
                case Orientation.UpLeft:
                    new_point = new Vector3(a.x + RoadOffset / 2, a.y, a.z + RoadOffset / 2);
                    break;
                case Orientation.DownLeft:
                    new_point = new Vector3(a.x - RoadOffset / 2, a.y, a.z + RoadOffset / 2);
                    break;
                case Orientation.UpRight:
                    new_point = new Vector3(a.x + RoadOffset / 2, a.y, a.z - RoadOffset / 2);
                    break;
                case Orientation.DownRight:
                    new_point = new Vector3(a.x - RoadOffset / 2, a.y, a.z - RoadOffset / 2);
                    break;
                default:
                    new_point = a;
                    break;
            }
            return new_point;
        }

        // FUNC: Path UpSampling
        public List<Vector3> pathUpsampling(List<Vector3> original_path, int mul_fact)
        {
            if (original_path == null || original_path.Count == 0)
                return null;
            List<Vector3> upsampled_path = new List<Vector3>();
            for (int i = 0; i < original_path.Count - 1; i++)
            {
                Vector3 curr = original_path[i];
                Vector3 next = original_path[i + 1];
                for (int j = 1; j <= mul_fact; j++)
                {
                    float new_x_pos = (next.x - curr.x) / mul_fact * j + curr.x;
                    float new_z_pos = (next.z - curr.z) / mul_fact * j + curr.z;
                    Vector3 new_point = new Vector3(new_x_pos, 0, new_z_pos);
                    upsampled_path.Add(new_point);
                }
            }

            return upsampled_path;
        }

        /// <summary>
        /// GROUP 8 CODE RE_USED 
        /// </summary>
        /// 
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

            var (closestPathCar, projectedCar, _) = FindClosestLineSegment(currentPos);
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
            var (closestPathSegment, projectedTarget, _) = FindClosestLineSegment(targetVector);
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
        private void UsePdTracker(Vector3 currentPos)
        {
            // Get the steering direction
            var (closestPath, projectedCar, nodeIndices) = FindClosestLineSegment(currentPos);
            var (closestPathDiscrete, _, _) = FindClosestLineSegment(currentPos);
            // var targetVector = currentPos + lookAheadForward * transform.forward + lookAheadVel * my_rigidbody.velocity;
            // var (closestPathTarget, projectedTarget) = FindClosestLineSegment(targetVector, path);

            var forwardDirection = ComputeLookAhead(nodeIndices[1], projectedCar);
            //Debug.DrawLine(currentPos, currentPos + forwardDirection*5, Color.cyan);


            // Find target node
            var targetNode = closestPathDiscrete[1];
            PdController(currentPos, projectedCar, transform.forward, nodeIndices[1]);
        }

        private void PdController(Vector3 currentPosition, Vector3 targetPosition, Vector3 forwardDirection, int nextNodeIndex)
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
            var maxLookAheadVertices = Convert.ToInt32(maxLookAhead / vertexSpacing);

            var lookAheadInterval = 1;
            var verticesLookAheadInterval = Convert.ToInt32(lookAheadInterval / vertexSpacing);

            float velAtTargetPosition = car.MaxSpeed;
            var brake = false;
            // Look at intervals forward
            for (int i = 0; i < maxLookAheadVertices; i += verticesLookAheadInterval)
            {
                var offset = Math.Min(nextNodeIndex + i, optimalVelocities.Length - 1);
                var count = Math.Min(i + verticesLookAheadInterval, (optimalVelocities.Length - 1) - offset);
                if (offset + count > optimalVelocities.Length - 1) break;
                var velSegment = new ArraySegment<float>(optimalVelocities, offset, count);
                // Find the minimum optimal velocity in this interval
                var (argmin, min) = MinVelInSegment(velSegment.ToList());
                // Calculate required braking distance
                var brakeDist = (Mathf.Pow(min, 2) - Mathf.Pow(rigidBody.velocity.magnitude, 2)) / (-2 * maxDecel);
                if (brakeDist > argmin * vertexSpacing)
                {
                    velAtTargetPosition = min;
                    brake = true;
                    //Debug.LogFormat("Braking at interval {0}", (i/verticesLookAheadInterval)+1);
                    break;
                }
            }

            //var velAtTargetPosition = optimalVelocities[Math.Min(nextNodeIndex + maxLookAheadVertices, optimalVelocities.Length-1)];

            // Debugging
            /*
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
            */




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

        // Returns the normalized look ahead vector which is the direction of node that is in the "forward" direction
        // along the bezier path, relative to the current position.
        Vector3 ComputeLookAhead(int nextNodeIndex, Vector3 projectedPoint)
        {
            var maxLookAheadIndex = Convert.ToInt32(Math.Ceiling(lookAheadDistance / vertexSpacing));
            var pathLength = my_path.Count;

            var lookAheadNode = my_path[Math.Min(pathLength - 1, nextNodeIndex + maxLookAheadIndex)];
            var direction = lookAheadNode - projectedPoint;
            return direction;

            var visitedNode = my_path[nextNodeIndex];
            for (var i = 0; i <= maxLookAheadIndex; i++)
            {
                visitedNode = my_path[Math.Min(pathLength - 1, nextNodeIndex + i)];
                var dist = (visitedNode - projectedPoint).magnitude;
                if (dist >= lookAheadDistance) break;
            }

            var lookAheadDirection = visitedNode - projectedPoint;

            return lookAheadDirection.normalized;
        }

        private float[] CalcOptimalVel()
        {
            float[] optimalVel = new float[Curvatures.Length];

            int lookAheadVertices = Convert.ToInt32(earlyBraking / vertexSpacing);
            for (int i = 0; i < optimalVel.Length; i++)
            {
                // Radius of circle with same curvature at the desired point
                var rho = 1 / Curvatures[Mathf.Min(optimalVel.Length - 1, i + lookAheadVertices)];
                optimalVel[i] = Mathf.Clamp(Mathf.Sqrt(Physics.gravity.magnitude * frictionCoefficient * rho), -car.MaxSpeed, car.MaxSpeed);
                continue;
                // Todo are vertices this evenly spaced?
                var end = i + lookAheadVertices;
                end = Math.Min(end, optimalVel.Length - 1);
                var segment = new ArraySegment<float>(Curvatures, i, end - i);
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
            List<bool> collisionState = new List<bool>() { false, false, false, false };

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
            var (closestPath, projectedCar, nodeIndices) = FindClosestLineSegment(transform.position);
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
        public (Vector3[], Vector3, int[]) FindClosestLineSegment(Vector3 pos)
        {

            var minDistance = float.PositiveInfinity;
            var closestNodePair = new Vector3[2];
            var closestNodePairIndices = new int[2];
            // Todo remove
            var smallestProjection = new Vector3();
            List<Vector3> usedPath = my_path;
            // Skip last node to avoid index error
            for (var i = 0; i < usedPath.Count - 1; i++)
            {
                var a = usedPath[i];
                var b = usedPath[i + 1];
                var relativePos = pos - a;
                var projectedPos = FindProjection(relativePos, a, b);
                var distToSegment = (projectedPos - pos).magnitude;

                if (distToSegment > minDistance) continue;
                minDistance = distToSegment;
                closestNodePair[0] = a;
                closestNodePair[1] = b;
                closestNodePairIndices[0] = i;
                closestNodePairIndices[1] = i + 1;
                // Todo remove
                smallestProjection = projectedPos;
            }

            // Todo remove
            var diff = closestNodePair[1] - closestNodePair[0];
            var mid = closestNodePair[0] + diff / 2;
            // Debug.DrawLine(pos, smallestProjection, Color.cyan);


            return (closestNodePair, smallestProjection, closestNodePairIndices);
        }

        // Projects a position onto a path segment.
        public static Vector3 FindProjection(Vector3 p, Vector3 a, Vector3 b)
        {
            var pathSegment = b - a;
            var projectionScale = Vector3.Dot(p, pathSegment) / pathSegment.magnitude;
            var projectedPos = a + pathSegment.normalized * projectionScale;
            // If the normal point is not within the path segment, set the projected point to be the closest
            // node on the segment.
            if (projectedPos.x < Math.Min(a.x, b.x) ||
                projectedPos.x > Math.Max(a.x, b.x) ||
                projectedPos.z < Math.Min(a.z, b.z) ||
                projectedPos.z > Math.Max(a.z, b.z))
            {
                var distA = p.magnitude;
                var distB = (b - p).magnitude;
                projectedPos = distA < distB ? a : b;
            }

            return projectedPos;
        }

        private float[] CalculateAverageCurvatures(float[] actualCurvatures)
        {
            float[] avgCurvature = new float[actualCurvatures.Length];
            for (int i = 0; i < avgCurvature.Length; i++)
            {
                var start = Math.Max(0, i - 2);
                var end = Math.Min(actualCurvatures.Length - 1, i + 2);
                // Must be non-zero
                var n = end - start;
                var segment = new ArraySegment<float>(actualCurvatures, start, n);
                avgCurvature[i] = segment.Sum() / n;
            }

            return avgCurvature;
        }

        private float[] CalculateCurvatures()
        {
            float[] curvatures = new float[my_path.Count];
            var previousTangent = my_path[1] - my_path[0];
            curvatures[0] = 0;
            curvatures[1] = 0;
            for (var i = 2; i < my_path.Count - 1; i++)
            {
                var tangent = my_path[i] - my_path[i - 1];
                var approxSecondDerivative = tangent - previousTangent;
                curvatures[i] = CalculateCurvature(tangent, approxSecondDerivative);
                previousTangent = tangent;
            }

            return curvatures;
        }
        // For one point
        private float CalculateCurvature(Vector3 firstDer, Vector3 secondDer)
        {
            var numerator = Mathf.Abs(firstDer.x * secondDer.z - firstDer.z * secondDer.x);
            var denominator = Mathf.Pow(Mathf.Pow(firstDer.x, 2) + Mathf.Pow(firstDer.z, 2), 1.5f);
            return numerator / denominator;
        }
        /// <summary>
        /// END of GRP 8 CODE ^^^
        /// </summary>


        void OnDrawGizmos()
        {
            if (my_path != null && ReducedCars)
            {
                //Color[] colors = { Color.cyan, Color.white, Color.yellow, Color.green, Color.magenta, Color.gray, Color.red, Color.black, Color.cyan, Color.yellow.gamma, Color.green};
                //Gizmos.color = colors[CarNumber%5];
                Gizmos.color = Color.cyan;
                for (int i = 0; i < my_path.Count - 1; i++)
                {
                    Gizmos.DrawLine(my_path[i], my_path[i + 1]);
                }
            }
        }
    }
}
