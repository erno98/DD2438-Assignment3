using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI_working : MonoBehaviour
{
    // Variables for Drone
    private DroneController m_Drone;

    // Variables for Terrain
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    // Variables for Path Planning on Roads
    public GameObject my_goal_object;
    private GraphEmbedding road_map;
    private PathPlanner planner = new PathPlanner();
    public GameObject[] friends;
    private int DroneNumber, path_index, saved_index;
    private float RoadOffset = 2f;
    private List<Vector3> my_path;

    // Variables for path & driving
    private bool MazeComplete, WaitOver;
    private float mazeTimer, waitTime, stuckTimer;
    private Vector3 oldTargetPosition, olddronePosition, target_velocity;
    public int state;
    public float avg_speed, tau_speed, avg_speed_threshold, timer_tau;


    private bool ReducedDrones = false;

    private void Start()
    {
        // Initialize Variables
        Time.timeScale = 1;
        MazeComplete = false;
        mazeTimer = 0f;
        WaitOver = false;
        stuckTimer = 0f;
        state = 0;
        avg_speed_threshold = 3F;
        avg_speed = 5F;
        tau_speed = 2.5F;
        timer_tau = 2.5F;
        saved_index = 0;

        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

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

        // Debugger
        if (DroneNumber % 5 == 0)
        {
            ReducedDrones = true;
        }

        //Set delay timer
        waitTime = (DroneNumber % 4) * 0.25f;

        my_path = new List<Vector3>();
        path_index = 0;
        if (ReducedDrones)
        {
            // Get RoadMap
            road_map = new GraphEmbedding(terrain_manager.terrain_filename, 2, 0.5f);
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
    }


    private void FixedUpdate()
    {

        mazeTimer += Time.deltaTime;
        /*
        if (!MazeComplete && ReducedDrones)
        {
            path_index = PdDriveDrone(path_index);
            
            Vector3 relVect = 0.5f * ((transform.position - my_path[path_index]) / (transform.position - my_path[path_index]).sqrMagnitude);
            m_Drone.Move_vect(relVect);
            //update to new path index
            if (Vector3.Distance(transform.position, my_path[path_index]) <= 4f)
            {
                path_index= Mathf.Min(path_index + 1, my_path.Count - 1);
            }
            
            if (Vector3.Distance(m_Drone.transform.position, my_goal_object.transform.position) < 6f)
            {
                Debug.Log("Goal reached for drone: " + DroneNumber + " at time " + mazeTimer);
                MazeComplete = true;
                //UnityEditor.EditorApplication.isPlaying = false;
            }
        }
        */

        if (!WaitOver && mazeTimer > waitTime)
        {
            //Debug.Log("Waiting done for drone: " + DroneNumber + " at time " + mazeTimer);
            WaitOver = true;
        }

        if (WaitOver && ReducedDrones)
        {

            // 0: normal, 1: repel, 2: unstuck

            List<float> command = new List<float>();

            switch (state)
            {
                case 0: // Normal
                    path_index = PdDriveDrone(path_index);
                    this.avg_speed = Time.fixedDeltaTime / this.tau_speed * m_Drone.velocity.magnitude + (1F - Time.fixedDeltaTime / this.tau_speed) * this.avg_speed;
                    if (this.avg_speed < this.avg_speed_threshold)
                    {
                        this.state = 2;
                        saved_index = path_index;
                    }
                    if (CloseToFriend() >= 0)
                    {
                        this.state = 2;
                        saved_index = path_index;
                    }
                    break;

                case 1: // repel
                    this.state = 0;
                    this.avg_speed = 8F; // prevent 1-2 to fro swithcing
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
                    Vector3 go_to = repel_driving();
                    m_Drone.Move_vect(go_to);
                    //Debug.Log("Drone: " + DroneNumber + "Untrap driving");
                    break;

                default:
                    path_index = PdDriveDrone(path_index);
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

    // FUNC: PD Controller for Drone
    private int PdDriveDrone(int path_index)
    {

        RaycastHit hit;
        Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, 50f);

        float k_p = 1f;
        float k_d = 1f;
        // keep track of target position and velocity
        Vector3 target_position = my_path[path_index];
        Vector3 drone_position = m_Drone.transform.position;
        if (path_index + 1 < my_path.Count)
        {
            target_velocity = Vector3.Normalize(my_path[path_index + 1] - oldTargetPosition) * 8;
        }
        else
        {
            target_velocity = (target_position - oldTargetPosition) / Time.fixedDeltaTime;
        }
        Vector3 drone_velocity = (drone_position - olddronePosition) / Time.fixedDeltaTime;
        oldTargetPosition = target_position;
        olddronePosition = drone_position;
        // a PD-controller to get desired velocity
        Vector3 position_error = target_position - transform.position;
        Vector3 velocity_error = target_velocity - drone_velocity;
        Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;
        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);
        m_Drone.Move(desired_acceleration[0], desired_acceleration[2]);

        //Debug.DrawLine(target_position, steering, Color.white);
        Debug.DrawLine(transform.position, transform.position + drone_velocity, Color.blue);
        Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

        // Update to new index
        if (Vector3.Distance(drone_position, target_position) < 2f)
        {
            path_index = Mathf.Min(path_index + 1, my_path.Count - 1);
        }
        return path_index;
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

    // Update is called once per frame
    void Update()
    {

    }
}
