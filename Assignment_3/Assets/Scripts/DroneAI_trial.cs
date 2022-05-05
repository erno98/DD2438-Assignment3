
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Newtonsoft.Json;



[RequireComponent(typeof(DroneController))]
public class DroneAI_trial : MonoBehaviour
{

    private DroneController m_Drone; // the car controller we want to use


    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    public drone_PP_controller control3, control_backup;
    public List<RayController> rays;
    public int state;
    public bool forward, right;
    public Vector3 direction;
    public float avg_speed, tau_speed, avg_speed_threshold, timer, timer_tau;
    public float stuck_radius, padding;

    // copied from PD script

    Rigidbody my_rigidbody;

    // Variables for Path Planning on Roads
    public GameObject my_goal_object;
    private GraphEmbedding road_map;
    private PathPlanner planner = new PathPlanner();
    public GameObject[] friends;
    private int DroneNumber, wayPoint, path_index;
    private float RoadOffset = 2f;
    private List<Vector3> my_path;

    // Variables for path & driving
    private bool MazeComplete, WaitOver;
    private float mazeTimer, waitTime, stuckTimer;
    private Vector3 oldTargetPosition, olddronePosition, target_velocity;


    private bool ReducedDrones = false;


    private void Start()
    {

        // set hyperparameters

        state = 0;
        avg_speed_threshold = 3F;
        avg_speed = 5F;
        tau_speed = 2.5F;
        timer_tau = 2.5F;
        timer = 100F;
        stuck_radius = 4.5F;
        forward = true;
        right = true;
        direction = new Vector3();
        padding = 2.5F;

        // Initialize Variables
        Time.timeScale = 1;
        MazeComplete = false;
        mazeTimer = 0f;
        WaitOver = false;
        stuckTimer = 0f;

        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        my_rigidbody = GetComponent<Rigidbody>();

        // Get DroneNumber
        friends = GameObject.FindGameObjectsWithTag("Drone");
        for (int i = 0; i < friends.Length; i++)
        {
            if (friends[i].transform.position == m_Drone.transform.position)
            {
                DroneNumber = i;
                break;
            }
        }

        //Set delay timer
        waitTime = (DroneNumber % 4) * 0.25f;

        // Debugger
        if (true)
        {
            ReducedDrones = true;
        }

        my_path = new List<Vector3>();
        wayPoint = 0;
        path_index = 0;
        if (ReducedDrones)
        {
            // Get RoadMap
            road_map = new GraphEmbedding(terrain_manager.terrain_filename, 2, padding);
            // Get Path
            List<Vector3> initial_path = planner.plan_path(transform.position, my_goal_object.transform.position, road_map, terrain_manager.myInfo);
            // Get Road Path
            my_path.Add(transform.position);
            for (int i = 1; i < initial_path.Count - 1; i++)
            {
                Orientation currOrientation = getPathDirection(initial_path[i], initial_path[i + 1]);
                Orientation prevOrientation = getPathDirection(initial_path[i - 1], initial_path[i]);
                my_path.Add(AddRoadLayout(initial_path[i], currOrientation, prevOrientation, RoadOffset));
                //if(DroneNumber==30) Debug.Log("S: " + initial_path[i] + " D: " + initial_path[i+1] + " " + currOrientation + " new:" + AddRoadLayout(initial_path[i], currOrientation, prevOrientation, RoadOffset));
            }
            my_path.Add(my_goal_object.transform.position);
            my_path = pathUpsampling(my_path, 8);
        }

        // Draw Blue road grid
        if (DroneNumber == 0) road_map.draw_embedding(terrain_manager.myInfo);


        List<Vector2> waypoints = new List<Vector2>();
        foreach (var waypoint in my_path)
        {
            //Debug.Log("waypoint added");
            //Debug.Log(waypoint);
            waypoints.Add(new Vector2(waypoint[0], waypoint[2]));
        }

        // entering path into the controller, postprocessing, stuff

        polygon_path p1 = new polygon_path(new waypoint_list(waypoints), padding, terrain_manager.terrain_filename);
        polygon_path p2 = new polygon_path(p1);


        // **********************

        // CONTROLLER PARAMS HERE

        // **********************

        // polygon_path _path, float _lookahead, float coarseness, float max_deviation, float k_p, float k_d, float v , float mu          

        drone_PP_controller pp_ctr3 = new drone_PP_controller(p1, 18F, padding, 1F, 10F, 0.5F, 0.1F, 15F);
        drone_PP_controller pp_ctr_backup = new drone_PP_controller(p2, 4F, padding, 0.5F, 10F, 0.5F, 0.1F, 15F);
        /*
        Vector3 _old_wp = pp_ctr_backup.path.wp.waypoints[0];
        foreach (var wp in pp_ctr3.path.wp.waypoints)
        {
            Debug.DrawLine(_old_wp, wp, Color.red, 100f);
            //Debug.Log("segment drawn");
            _old_wp = wp;
        }

        _old_wp = pp_ctr_backup.path.wp.waypoints[0];
        foreach (var wp in pp_ctr_backup.path.wp.waypoints)
        {
            Debug.DrawLine(_old_wp, wp, Color.blue, 100f);
            //Debug.Log("segment drawn");
            _old_wp = wp;
        }

        //Debug.Log("speed limit");
        Debug.Log(pp_ctr3.path.wp.waypoints.Count);
        foreach (float lim in pp_ctr3.path.speed_limit)
        {
            Debug.Log(lim);
        }

        Debug.Log("------------------------");
        */

        this.control3 = pp_ctr3;
        this.control_backup = pp_ctr_backup;

        this.direction = this.control3.path.desired_driving_direction_3d(pp_ctr3.path.wp.waypoints[0]);

        // *****************

        // RAY PARAMS HERE


        // *****************

        List<RayController> rays = new List<RayController>();
        float k_r = 8F;
        float threshold_distance = 5F;
        float current_angle;

        for (int _ray = 0; _ray < 8; _ray++)
        {

            current_angle = (float)_ray * Mathf.PI * 2F / 8F;
            //Debug.Log("angle " + current_angle);
            rays.Add(new RayController(new Vector3(Mathf.Cos(current_angle), 0, Mathf.Sin(current_angle)), threshold_distance, k_r));

        }

        this.rays = rays;

        /*
        Vector3 new_wp;
        Vector3 old_wp = new Vector3(pp_ctr3.path.wp.waypoints[0].x, 0, pp_ctr3.path.wp.waypoints[0].y);
        foreach (var wp in pp_ctr3.path.wp.waypoints)
        {
            new_wp = new Vector3(wp.x, 0, wp.y);
            Debug.DrawLine(old_wp, new_wp, Color.red, 100f);
            old_wp = new_wp;
        }
        Vector3 _old_wp_ = transform.position;
        foreach (var wp in my_path)
        {
            Debug.DrawLine(_old_wp_, wp, Color.green, 100f);
            //Debug.Log("segment drawn");
            _old_wp_ = wp;
        }
        */



    }




    private void FixedUpdate()
    {
        mazeTimer += Time.deltaTime;

        if (!WaitOver && mazeTimer > waitTime)
        {
            //Debug.Log("Waiting done for drone: " + DroneNumber + " at time " + mazeTimer);
            WaitOver = true;
        }

        if (WaitOver && ReducedDrones)
        {
            // Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));
            /*
            Debug.DrawLine(transform.position, new Vector3(this.control3.path.wp.waypoints[this.control3.path.last_visited].x, 0f,
                                                            this.control3.path.wp.waypoints[this.control3.path.last_visited].y));
            Debug.DrawLine(transform.position, new Vector3(this.control_backup.path.wp.waypoints[this.control_backup.path.last_visited].x, 0f,
                                                                this.control_backup.path.wp.waypoints[this.control_backup.path.last_visited].y));
            */
            // *************************************************

            // 0: normal, 1: repel, 2: unstuck

            List<float> command = new List<float>();

            switch (this.state)
            {
                case 0: // Normal
                    command = normal_driving();
                    m_Drone.Move(command[0] / 15F, command[1] / 15F);
                    this.avg_speed = Time.fixedDeltaTime / this.tau_speed * m_Drone.velocity.magnitude + (1F - Time.fixedDeltaTime / this.tau_speed) * this.avg_speed;
                    if (this.avg_speed < this.avg_speed_threshold)
                    {
                        this.state = 2;
                        this.timer = 100;
                        this.direction = this.control3.path.desired_driving_direction_3d(transform.position);
                    }
                    if (CloseToFriend() >= 0)
                    {
                        this.state = 1;
                        this.direction = this.control3.path.desired_driving_direction_3d(transform.position);
                    }
                    break;

                case 1: // repel
                    this.state = 0;
                    this.avg_speed = 8F; // prevent 1-2 to fro swithcing
                    this.right = (Vector3.SignedAngle(transform.forward, this.control3.path.desired_driving_direction_3d(transform.position), new Vector3(0, 1, 0)) > 0F);
                    this.forward = true;
                    Vector3 repel_to = repel();
                    m_Drone.Move_vect(repel_to);
                    //Debug.Log("Drone: " + DroneNumber + "REPEL");
                    stuckTimer += Time.deltaTime;
                    if (stuckTimer > 1f)
                    {
                        this.state = 2;
                    }
                    break;

                case 2: // untrap repel
                    this.state = 0;
                    this.avg_speed = 8F; // prevent 1-2 to fro swithcing
                    this.right = (Vector3.SignedAngle(transform.forward, this.control3.path.desired_driving_direction_3d(transform.position), new Vector3(0, 1, 0)) > 0F);
                    this.forward = true;
                    Vector3 go_to = repel_driving();
                    m_Drone.Move_vect(go_to);
                    //command = untrap_driving();
                    //m_Drone.Move(command[0] / 15F, command[1] / 15F);
                    //Debug.Log("Drone: " + DroneNumber + "Untrap driving");
                    break;

                default:
                    command = normal_driving();
                    m_Drone.Move(command[0] / 15F, command[1] / 15F);
                    break;
            }


            if (!MazeComplete && Vector3.Distance(m_Drone.transform.position, my_goal_object.transform.position) < 6f)
            {
                Debug.Log("Goal reached for drone: " + DroneNumber + " at time " + mazeTimer);
                MazeComplete = true;
                //UnityEditor.EditorApplication.isPlaying = false;
            }
        }

    }

    private int CloseToFriend()
    {
        for (int i = 0; i < friends.Length; i++)
        {
            if (i != DroneNumber && Vector3.Distance(friends[i].transform.position, m_Drone.transform.position) < 5f)
            {
                return i;
            }
        }
        return -1;
    }


    List<float> normal_driving() // return driving command
    {
        Vector3 pp_acc, ray_acc;
        pp_acc = PPcontroller_acceleration();
        ray_acc = Ray_acceleration(true);
        //Debug.Log("accelerations");
        //Debug.Log(acc-Ray_acceleration(true));
        //Debug.Log(Ray_acceleration(true));
        //Debug.Log("-------------");
        List<float> command;

        Vector3 n_velocity = new Vector3(1, 0, 0);

        if (m_Drone.velocity.magnitude > 0.0001F)
        {
            n_velocity = m_Drone.velocity / m_Drone.velocity.magnitude;
        }

        Vector3 acc = pp_acc + ray_acc;

        Vector3 acceleration = Vector3.Dot(acc, n_velocity) * n_velocity;
        Vector3 steering = acc - acceleration;

        // brute hard-coding

        float gamma = 0.9F;

        acceleration = acceleration.normalized * Mathf.Max(15F * gamma, acceleration.magnitude);
        steering = steering.normalized * Mathf.Max(15F * Mathf.Sqrt(1F - gamma * gamma), steering.magnitude);

        acc = acceleration + steering;


        //Debug.Log(acc);

        command = this.control3.acceleration_to_command_drone(acc, Vector3.right, Vector3.forward);

        //Debug.Log(command[0]);
        //Debug.Log(command[1]);

        return command;
    }

    Vector3 repel() // return driving command
    {
        Vector3 relVect = friends[CloseToFriend()].transform.position - m_Drone.transform.position;
        //Vector3 relVect = my_goal_object.transform.position - m_Drone.transform.position;
        //relVect = relVect + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-50.0f, 50.0f));
        return new Vector3(relVect.z, 0, -relVect.x);
    }

    Vector3 repel_driving() // push away from friends  and then reorient itself
    {
        //Vector3 relVect = friends[CloseToFriend()].transform.position - m_Drone.transform.position;
        Vector3 relVect = my_goal_object.transform.position - m_Drone.transform.position;
        //relVect = relVect + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-50.0f, 50.0f));
        return relVect;
    }

    List<float> untrap_driving() // push away from walls and then reorient itself
    {
        Vector3 acc;
        acc = Ray_acceleration(false) + 10f * (new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-50.0f, 50.0f)));
        //acc = -Ray_acceleration(false);
        acc = acc * 10f;
        List<float> command;
        command = this.control3.acceleration_to_command_drone(acc, transform.right, transform.forward);

        return command;
    }

    Vector3 PPcontroller_acceleration()
    {
        Vector3 sum_acceleration = new Vector3(0, 0, 0);
        Vector3 _tmp;
        //float bp_ratio=10F;

        _tmp = this.control3.desired_acceleration(transform.position, m_Drone.velocity, transform.right, transform.forward, m_Drone.max_acceleration);
        if (!(this.control3.is_lookahead_blocked(transform.position)))
        {
            sum_acceleration += _tmp;
        }

        //Debug.Log("c3 looakhead: "+this.control3.is_lookahead_blocked(transform.position));

        _tmp = this.control_backup.desired_acceleration(transform.position, m_Drone.velocity, transform.right, transform.forward, m_Drone.max_acceleration);

        //if(this.control.is_lookahead_blocked(transform.position) && this.control2.is_lookahead_blocked(transform.position) && this.control3.is_lookahead_blocked(transform.position)){
        if (this.control3.is_lookahead_blocked(transform.position))
        {
            sum_acceleration = _tmp;
        }

        return sum_acceleration;
    }

    Vector3 Ray_acceleration(bool normal)
    {
        Vector3 sum_acceleration = new Vector3(0, 0, 0);
        Vector3 _direction;
        Vector3 current_acc;
        float _dist, kr;


        if (normal)
        {

            foreach (RayController ray in this.rays)
            {

                kr = ray.k_r;

                // only consider rays that point to the same direction as the car is riding

                /*     _direction=transform.TransformDirection(ray.relative_direction);
                    _direction.y=0F;
                    _direction=_direction/_direction.magnitude; */

                if (Vector3.Dot(ray.relative_direction, m_Drone.velocity.normalized) < 0.25F)
                { // non driving direction

                    //ray.k_r=ray.k_r/4F;

                    continue;
                }

                _dist = ray.max_distance;
                ray.max_distance = Mathf.Max((m_Drone.velocity.magnitude) * (m_Drone.velocity.magnitude) / (m_Drone.max_acceleration / 2F), _dist);
                current_acc = ray.desired_acceleration_drone(transform.position, ray.relative_direction);
                // Debug.Log(current_acc);
                //Debug.Log(ray.max_distance);
                // Debug.Log(ray.k_r);
                ray.max_distance = _dist;
                ray.k_r = kr;

                sum_acceleration = sum_acceleration + current_acc;
            }
            return sum_acceleration;
        }
        else
        {
            foreach (RayController ray in this.rays)
            {

                // only consider rays that point to the same direction as the car is riding

                /* _direction=transform.TransformDirection(ray.relative_direction); */

                if (Math.Abs(Vector3.Dot(Vector3.forward, ray.relative_direction)) < -0.99F)
                { // never triggred
                    current_acc = ray.desired_acceleration(transform.position, ray.relative_direction);
                    current_acc = transform.TransformDirection(Vector3.right) * Vector3.Dot(transform.TransformDirection(Vector3.right), current_acc);
                }
                else
                {
                    //_dist=ray.max_distance;
                    // ray.max_distance=(my_rigidbody.velocity.magnitude)*(my_rigidbody.velocity.magnitude)/(9.81F);
                    current_acc = ray.desired_acceleration(transform.position, ray.relative_direction);
                    //ray.max_distance=_dist;
                }
                sum_acceleration = sum_acceleration + current_acc;
            }
            return sum_acceleration;
        }
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

    // FUNC: Add Road offset based on direction of drone path
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

    void OnDrawGizmos()
    {
        if (my_path != null && ReducedDrones)
        {
            //Color[] colors = { Color.cyan, Color.white, Color.yellow, Color.green, Color.magenta, Color.gray, Color.red, Color.black, Color.cyan, Color.yellow.gamma, Color.green};
            //Gizmos.color = colors[DroneNumber%5];
            Gizmos.color = Color.cyan;
            for (int i = 0; i < my_path.Count - 1; i++)
            {
                Gizmos.DrawLine(my_path[i], my_path[i + 1]);
            }
        }
    }
}


