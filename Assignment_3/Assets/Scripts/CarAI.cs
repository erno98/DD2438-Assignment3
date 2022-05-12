using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        // Variables for Car
        private CarController m_Car;
        private Rigidbody m_Car_Rigidbody;

        // Variables for Terrain
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        private CarConfigSpace ObstacleSpace;

        // Variables for Path Planning on Roads
        public GameObject my_goal_object;
        private GraphEmbedding road_map;
        private PathPlanner planner = new PathPlanner();
        public GameObject[] friends;
        private int CarNumber, wayPoint, path_index;
        private float RoadOffset = 3f;
        private List<Vector3> my_path, simple_path, intersection;
        private Vector3 L, R, U, D, C;

        // PD Variables
        private Vector3 targetSpeed, desiredSpeed, targetPos_K1;
        private float k_p = 2f, k_d = 0.5f;

        // Variables for path & driving
        private float acceleration, max_speed;
        private bool PlayerCrashed, MazeComplete, WaitOver;
        private float mazeTimer, driveTimer, stuckTimer, recoveryTime, waitTime;
        private float recoverySteer;
        enum car_state { drive, front_crash, back_crash };
        private car_state car_status;
        private bool backward_crash = false;
        private bool frontal_crash = false;
        private float accelerationAmount = 0;
        private float footbrake = 0;
        private float steeringAmount = 0;
        private float handbrake = 0;
        private Vector3 targetPosition;
        private Vector3 start_pos, previous_pos;

        // Variables for Collision detection
        private float angle, spacing, checkRadius;
        private int checkDensity;

        // Variables for Handshakes
        enum handshake { follow, wait_to_follow, cross, wait_to_cross, none }
        private handshake my_handshake = handshake.none;
        private int following, wait_follow, let_cross, wait_cross;
        private List<GameObject> CloseByFriends;

        private bool ReducedCars = false;

        private void Start()
        {
            // Initialize Variables
            Time.timeScale = 10;
            driveTimer = 0f;
            MazeComplete = false;
            max_speed = 30;
            acceleration = 1f;
            path_index = 1;
            car_status = car_state.drive;
            stuckTimer = 0f;
            PlayerCrashed = false;
            recoveryTime = 0.7f;
            recoverySteer = 0.45f;//45 degrees gives best result
            angle = 90;
            spacing = 40;
            checkDensity = 2;
            checkRadius = 1f;
            following = -1;
            let_cross = -1;
            wait_cross = -1;
            wait_follow = -1;
            WaitOver = false;

            // get the car controller
            m_Car = GetComponent<CarController>();
            m_Car_Rigidbody = GetComponent<Rigidbody>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            // HardCoded Intersection
            C = new Vector3(177.5f, 0f, 192.5f);
            L = new Vector3(C.x - 60f, C.y, C.z);
            R = new Vector3(C.x + 60f, C.y, C.z);
            U = new Vector3(C.x, C.y, C.z + 60f);
            D = new Vector3(C.x, C.y, C.z - 60f);
            intersection = new List<Vector3>();
            intersection.Add(L);
            intersection.Add(R);
            intersection.Add(U);
            intersection.Add(D);


            // Get CarNumber
            friends = GameObject.FindGameObjectsWithTag("Car");
            for (int i = 0; i < friends.Length; i++)
            {
                if (friends[i].transform.position == m_Car.transform.position)
                {
                    CarNumber = i;
                    break;
                }
            }

            //Set delay timer
            waitTime = IdentifyQuardants(transform.position) * 20f;


            // Debugger
            if (CarNumber % 10 == 0)
            {
                ReducedCars = true;
            }

            my_path = new List<Vector3>();
            wayPoint = 0;
            path_index = 0;
            if (ReducedCars)
            {
                // Initialize ConfigSpace
                CreateObstacleSpace();
                // Get RoadMap
                road_map = new GraphEmbedding(terrain_manager.terrain_filename, 1, 1.5f);
                // Get Path
                List<Vector3> initial_path = planner.plan_path(transform.position, my_goal_object.transform.position, road_map, terrain_manager.myInfo);
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
            if (CarNumber == 0) road_map.draw_embedding(terrain_manager.myInfo);

        }

        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;

            if (!WaitOver && mazeTimer >= waitTime)
            {
                //Debug.Log("Waiting done for car: " + CarNumber + " at time " + mazeTimer);
                Debug.Log("Car: " + CarNumber + " at Quadrant" + IdentifyQuardants(transform.position));
                WaitOver = true;
            }

            if (WaitOver && !MazeComplete && ReducedCars)
            {
                //path_index = DriveCar(my_path, m_Car, path_index);

                if (wayPoint < my_path.Count)
                {
                    DriveCarPD(my_path[wayPoint]);
                    if (Vector3.Distance(m_Car.transform.position, my_path[wayPoint]) <= 0.5f)
                    {
                        wayPoint = Mathf.Min(wayPoint + 1, my_path.Count - 1);
                    }
                }

                if (Vector3.Distance(m_Car.transform.position, my_goal_object.transform.position) < 4f)
                {
                    Debug.Log("Goal reached for car: " + CarNumber + " at time " + mazeTimer);
                    MazeComplete = true;
                    //UnityEditor.EditorApplication.isPlaying = false;
                }
            }

        }

        /// FUNCTIONS
        // FUNC: Identify Quadrants
        private int IdentifyQuardants(Vector3 pos)
        {
            float l = 10f;
            int q;
            List<Vector3> corners = new List<Vector3>();
            corners.Add(new Vector3(terrain_manager.myInfo.x_low, 0, terrain_manager.myInfo.z_low));
            corners.Add(new Vector3(terrain_manager.myInfo.x_high, 0, terrain_manager.myInfo.z_low));
            corners.Add(new Vector3(terrain_manager.myInfo.x_high, 0, terrain_manager.myInfo.z_high));
            corners.Add(new Vector3(terrain_manager.myInfo.x_low, 0, terrain_manager.myInfo.z_high));

            if ((pos.x <= (C.x - l)) && (pos.z >= (C.z - l)))
            {
                q = 1;
                corners[1] = new Vector3((C.x - l), 0, (C.z - l));
            }
            else if((pos.x <= (C.x - l)) && (pos.z >= (C.z + l)))
            {
                q = 2;
                corners[0] = new Vector3((C.x - l), 0, (C.z + l));
            }
            else if ((pos.x <= (C.x + l)) && (pos.z >= (C.z + l)))
            {
                q = 3;
                corners[3] = new Vector3((C.x + l), 0, (C.z + l));
            }
            else 
            {
                q = 4;
                corners[2] = new Vector3((C.x + l), 0, (C.z - l));
            }
            for(int i=0; i<corners.Count-1;i++)
            {
                Debug.DrawLine(corners[i], corners[i + 1], Color.magenta);
            }

            return q;
        }



        // FUNC: PD Tracker for Car
        private void DriveCarPD(Vector3 targetPos)
        {
            // store previous
            Vector3 currentPos = m_Car.transform.position;
            targetSpeed = (targetPos - targetPos_K1) / Time.fixedDeltaTime;
            targetPos_K1 = targetPos;

            // Steering & Acceleration
            var posError = targetPos - currentPos;
            var speedError = targetSpeed - m_Car_Rigidbody.velocity;
            var desiredDir = k_p * posError + k_d * speedError;
            var steering = Vector3.Dot(desiredDir, transform.right);
            var acceleration = Vector3.Dot(desiredDir, transform.forward);

            Debug.DrawLine(targetPos, targetPos + targetSpeed, Color.red);
            Debug.DrawLine(currentPos, currentPos + m_Car_Rigidbody.velocity, Color.blue);
            Debug.DrawLine(currentPos, currentPos + desiredDir, Color.black);

            m_Car.Move(steering, acceleration, acceleration, 0f);
        }

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

        // FUNC: Car Drive
        public int DriveCar(List<Vector3> player_path, CarController player_Car, int player_pathIndex)
        {
            Vector3 player_waypoint = player_path[player_pathIndex];
            float steeringAmount, accelerationAmount, car_steer_next;

            //find steering needed to get to next point
            steeringAmount = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            accelerationAmount = Accelerate(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            car_steer_next = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_path[(player_pathIndex + 1) % player_path.Count]);
            // Do opposite turn if next next steer is in opposte
            if (Math.Abs(car_steer_next) > 0.3f && Math.Abs(steeringAmount) < 0.3f)
            {
                steeringAmount = -car_steer_next;
            }
            if (Math.Abs(steeringAmount) < 0.2f)
            {
                ;
                //steeringAmount = 0f;
            }
            else if (Math.Abs(steeringAmount) > 0.9f)
            {
                if (steeringAmount > 0)
                {
                    steeringAmount = 0.8f;

                }
                else
                {
                    steeringAmount = -0.8f;
                }

            }

            // DRIVE BLOCK
            if (!PlayerCrashed)
            {
                driveTimer += Time.deltaTime;

                //Check if car stuck and not at starting pointing
                if (driveTimer >= stuckTimer && Vector3.Distance(start_pos, player_Car.transform.position) > 0 && path_index > 10)
                {
                    driveTimer = 0;
                    //if (CarNumber == 1) Debug.Log("Crash pos: " + Vector3.Distance(previous_pos, player_Car.transform.position));
                    //check: car not moved very much from previous position
                    if (Vector3.Distance(previous_pos, player_Car.transform.position) < 0.05f)
                    {
                        PlayerCrashed = true;
                        car_status = car_state.front_crash;
                        // check if obstacle in front of car

                        if (player_Car.CurrentSpeed < 0.5f && Physics.BoxCast(player_Car.transform.position,
                            new Vector3(ObstacleSpace.BoxSize.x / 2, ObstacleSpace.BoxSize.y / 2, 0.5f),
                            player_Car.transform.forward,
                            Quaternion.LookRotation(player_Car.transform.forward),
                            ObstacleSpace.BoxSize.z / 1.7f))
                        {
                            car_status = car_state.front_crash;
                        }
                        else if (player_Car.CurrentSpeed < 0.5f)
                        {
                            car_status = car_state.back_crash;
                        }

                        else if (Physics.BoxCast(player_Car.transform.position,
                            new Vector3(ObstacleSpace.BoxSize.x / 2, ObstacleSpace.BoxSize.y / 2, 0.5f),
                            Quaternion.AngleAxis(0, Vector3.down) * player_Car.transform.forward,
                            Quaternion.LookRotation(Quaternion.AngleAxis(0, Vector3.down) * player_Car.transform.forward),
                            ObstacleSpace.BoxSize.z / 1.7f))
                        {
                            car_status = car_state.back_crash;
                        }
                        else
                        {
                            PlayerCrashed = false;
                            car_status = car_state.drive;
                        }
                    }
                    else
                    {
                        // update previous car position
                        previous_pos = player_Car.transform.position;
                    }
                }
                float old_steering = steeringAmount;
                avoid_obstacles();
                if (old_steering != steeringAmount)
                {
                    if (CarNumber == 1) Debug.Log("Crashed, old steering: " + old_steering + " new steering: " + steeringAmount);
                }
                // if current steering angle is too high, then don't accelerate
                if (Math.Abs(steeringAmount) > 0.8f && player_Car.CurrentSpeed > max_speed / 10)
                {
                    ;//accelerationAmount = 0;
                }
                // if current speed is max, then don't accelerate
                if (player_Car.CurrentSpeed >= max_speed)
                {
                    accelerationAmount = 0;
                }
                // if acceleration is reverse, apply backwards turns
                if (accelerationAmount < 0)
                {
                    player_Car.Move(-steeringAmount, footbrake, accelerationAmount * acceleration, 0);
                }
                else
                {
                    player_Car.Move(steeringAmount, accelerationAmount * acceleration, 0, 0);
                }

            }
            // CRASH RECOVERY BLOCK
            else
            {
                stuckTimer += Time.deltaTime;

                // immediately drive away from crash position
                if (stuckTimer <= recoveryTime && Math.Abs(steeringAmount) > recoverySteer)
                {
                    //reverse car if crashed in front
                    if (car_status == car_state.front_crash)
                    {
                        //if (CarNumber == 1) Debug.Log("Front Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                        player_Car.Move(-steeringAmount, 0, -1.5f, 0);
                        // switch up crash direction if recovery delayed
                        if (stuckTimer > recoveryTime / 2 && PlayerCrashed)
                        {
                            car_status = car_state.back_crash;
                            //if (CarNumber == 1) Debug.Log("Back Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                            player_Car.Move(steeringAmount, 1.5f, 0, 0);
                        }
                    }
                    //drive forward if crashed while reverse drive
                    else
                    {
                        //if (CarNumber == 1) Debug.Log("Back Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                        player_Car.Move(steeringAmount, 1.5f, 0, 0);
                    }
                }
                else
                {
                    // reset to drive mode from recovery
                    stuckTimer = 0;
                    car_status = car_state.drive;
                    PlayerCrashed = false;
                }
            }

            //update to new path index
            if (Vector3.Distance(player_Car.transform.position, player_waypoint) <= 4f)
            {
                player_pathIndex = Mathf.Min(player_pathIndex + 1, player_path.Count - 1);
            }

            return player_pathIndex;
        }

        // Sub-Func: To find steering angle for car
        private float Steer(Vector3 position, float theta, Vector3 target)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToTarget = target - position;
            float angle = Vector3.Angle(direction, directionToTarget) * Mathf.Sign(-direction.x * directionToTarget.z + direction.z * directionToTarget.x);
            return Mathf.Clamp(angle, -m_Car.m_MaximumSteerAngle, m_Car.m_MaximumSteerAngle) / m_Car.m_MaximumSteerAngle;
        }

        //Sub-Func: To find accleration for car
        private float Accelerate(Vector3 position, float theta, Vector3 target)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToTarget = target - position;
            return Mathf.Clamp(direction.x * directionToTarget.x + direction.z * directionToTarget.z, -1, 1);
        }

        // Sub-Func: To avoid obstacles while driving
        private void avoid_obstacles(bool curve_approaching = false, float range = 5f)
        {
            RaycastHit hit;
            Vector3 maxRange = new Vector3(range, 0, range / 1.5f);
            bool had_hit = false;
            bool had_hit_frontally = false;

            if (frontal_crash && Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange.z) && hit.collider.gameObject.name == "Cube")
            {
                this.accelerationAmount = 0;
                this.footbrake = -1;
                this.steeringAmount *= -1;

                return;
            }
            else frontal_crash = false; //stays in the frontal crash state driving backward until there is no obstacle in front of the car

            if (backward_crash && m_Car.CurrentSpeed < 5f)
            {
                this.accelerationAmount = 1;
                this.footbrake = 0;
                return;
            }
            else backward_crash = false;

            if (m_Car.CurrentSpeed < 5f)
            {
                this.accelerationAmount += 0.5f;
            }

            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange.z) && hit.collider.gameObject.name == "Cube")
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.5f;
                this.footbrake = this.footbrake < 0.2f ? 0.5f : this.footbrake * 2;
                if (CarNumber == 0) Debug.Log("Frontal collision, distance: " + hit.distance);
                had_hit = true;
                had_hit_frontally = true;

                if (hit.distance < 5 && m_Car.CurrentSpeed < 1f) //recovery from frontal hit
                {
                    frontal_crash = true;
                    //if (CarNumber == 0) Debug.Log("Collision STOP");
                    this.accelerationAmount = 0;
                    this.footbrake = -1;
                    this.steeringAmount *= -1;
                    this.handbrake = 0;
                }
            }

            /*if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back), out hit, maxRange.z))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount = 1;
                this.footbrake = 0;
                if (CarNumber == 1) Debug.Log("Back collision");
                had_hit = true;
            }*/

            if (Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.right), out hit, maxRange.x) && hit.collider.gameObject.name == "Cube")
            //Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.up + Vector3.right), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.y * maxRange.y)))
            {

                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += -0.5f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("Right collision");
                had_hit = true;


            }

            if (Physics.Raycast(transform.position + transform.right + transform.up, transform.TransformDirection(new Vector3(1, 0, 1)), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.z * maxRange.z) / 3f) && hit.collider.gameObject.name == "Cube")
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(new Vector3(1, 0, 1)) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.red);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += -0.5f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("Right-up collision " + hit.collider.gameObject.name);
                had_hit = true;
                had_hit_frontally = true;



            }

            if (Physics.Raycast(transform.position + transform.right + transform.up, transform.TransformDirection(new Vector3(-1, 0, 1)), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.z * maxRange.z) / 3f) && hit.collider.gameObject.name == "Cube")
            {


                Vector3 closestObstacleInFront = transform.TransformDirection(new Vector3(-1, 0, 1)) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.red);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += 0.5f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("left-up collision " + hit.collider.gameObject.name);
                had_hit = true;
                had_hit_frontally = true;



            }

            if (Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.left), out hit, maxRange.x) && hit.collider.gameObject.name == "Cube")
            //Physics.Raycast(transform.position + transform.right, transform.TransformDirection((Vector3.up + Vector3.left).normalized), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.y* maxRange.y)))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += 0.5f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("Left collision");

                had_hit = true;
            }

            if (!had_hit && !curve_approaching)
            {
                this.accelerationAmount *= 1.25f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("Not hit speed");
            }

            if (!had_hit && m_Car.CurrentSpeed < 1f)
            {
                if (CarNumber == 1) Debug.Log("Had hit backward");
                backward_crash = true;
                this.accelerationAmount = 1;
                this.footbrake = 0;
                this.handbrake = 0;
                this.steeringAmount *= 1;
            }

            if (had_hit && m_Car.CurrentSpeed < 1f)
            {
                frontal_crash = true;
                this.accelerationAmount = 0;
                this.footbrake = -1;
                this.handbrake = 0;
                this.steeringAmount *= -1;

            }
        }

        //FUNC: Check if any obstacle in ConfigSpace
        private bool Obstacle(float x, float z)
        {
            // check traversablity of the position
            if (terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z)] > 0.5f)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // FUNC: To create obstacle space
        private void CreateObstacleSpace()
        {
            //Obstacle Sapce
            Quaternion carRotation = m_Car.transform.rotation;
            m_Car.transform.rotation = Quaternion.identity;
            ObstacleSpace = new CarConfigSpace();
            BoxCollider carCollider = GameObject.Find("ColliderBottom").GetComponent<BoxCollider>();
            ObstacleSpace.BoxSize = carCollider.transform.TransformVector(carCollider.size);
            m_Car.transform.rotation = carRotation;
        }

        // FUNC: Detect friends close to you
        private void DetectFriends(GameObject[] friends)
        {
            for (int i = 0; i < friends.Length; i++)
            {
                if (i != CarNumber)
                {
                    if (Vector3.Distance(friends[CarNumber].transform.position, friends[i].transform.position) < 5f)
                    {
                        CloseByFriends.Add(friends[i]);
                    }
                }
            }
        }


        // FUNC: Collision Detection
        /*
        private var CollisionDetection(CarController player_Car)
        {

            float totalSpacing = spacing, currentSpacing, leftSpacing, rightSpacing;
            int levels = 10;
            bool left, right;
            List<bool> levelIntensity = new List<bool>(new bool[levels]);
            float scale;

            // Check if path in front of leader is narrow or not and assign spacing
            left = false;
            right = false;
            for (int i = -checkDensity; i <= checkDensity; ++i)
            {
                for (int j = -checkDensity; j <= checkDensity; ++j)
                {
                    for (float refAngle = -180; refAngle <= 180; refAngle = refAngle + 10)
                    {
                        float ellipse = .75f;
                        if (refAngle >= -35 && refAngle <= 35)
                        {
                            ellipse = 2.5f;
                        }
                        Vector3 refline = Quaternion.AngleAxis(refAngle, player_Car.transform.up) * player_Car.transform.forward * ellipse * totalSpacing;
                        for (float k = 0.0f; k <= checkRadius; k = k + 0.1f)
                        {
                            float x = i + (player_Car.transform.position + k * refline).x;
                            float z = j + (player_Car.transform.position + k * refline).z;
                            if (Obstacle(x, z))
                            {
                                levelIntensity[(int)(k * levels)] = true;
                                if (refAngle >= -180 && refAngle < 0)
                                {
                                    left = true;

                                }
                                else
                                {
                                    right = true;
                                }
                            }
                        }
                    }
                }
            }

            //Debug.Log("Narrow Path dectected  = " + narrowpath);
            // Do sparse spacing
            currentSpacing = totalSpacing;
            leftSpacing = currentSpacing;
            rightSpacing = currentSpacing;
            scale = 2f;
            // Adjust position based on collision status
            if (narrowpath)
            {
                //Debug.Log("L1: " + L1 + " L2: " + L2 + " L3: " + L3 + " L4: " + L4);
                // Do tight spacing
                for (int i = levels - 1; i >= 0; i--)
                {
                    if (levelIntensity[i] == true)
                    {
                        currentSpacing = totalSpacing / ((levels - i + 1) * 3);
                        // Debug.Log("Level: " + i + " spacing " + (levels - i + 1) * 2);
                        scale = speed / ((levels - i + 1) * 4);
                        //rightSpacing = totalSpacing / 2;
                        //leftSpacing = totalSpacing / 2;
                        if (left)
                        {
                            leftSpacing = currentSpacing;
                        }
                        if (right)
                        {
                            rightSpacing = currentSpacing;
                        }

                    }
                }
                //Debug.Log("Current: " + currentSpacing + " scale " + scale);
            }
        */


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
