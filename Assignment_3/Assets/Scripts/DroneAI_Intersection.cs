using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI_Intersection : MonoBehaviour
{
    private DroneController m_Drone; // the car controller we want to use
    public GameObject my_goal_object; // stores a Sphere. it's "transform" contains the Vec3 coordinates of the goal
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    private GraphEmbedding road_map;
    private PathPlanner planner = new PathPlanner();
    int UpdateCntr = 0;
    public Vector3 Target;
    public GameObject[] friends;
    private bool MazeComplete, WaitOver, ReducedDrones, SimplePath = false;
    private int DroneNumber, path_index, saved_index;
    private float RoadOffset = 3.5f;
    private List<Vector3> my_path, simple_path, intersection;
    private float mazeTimer, waitTime, velocityThreshold = 10f, distanceThreshold = 5f;
    private Vector3 L, R, U, D, C;

    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        // HardCoded Intersection
        C = new Vector3(177.5f, 0f, 192.5f);
        L = new Vector3(C.x - 60f, C.y, C.z);
        R = new Vector3(C.x + 60f, C.y, C.z);
        U = new Vector3(C.x, C.y, C.z + 60f);
        D = new Vector3(C.x, C.y, C.z - 60f);
        intersection = new List<Vector3>();
        intersection.Add(L);
        intersection.Add(U);
        intersection.Add(R);
        intersection.Add(D);

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
        if (true)
        {
            ReducedDrones = true;
        }

        //Set delay timer

        if (((DroneNumber >= 2) && (DroneNumber <= 10)) || ((DroneNumber >= 23) && (DroneNumber <= 36)))
        {
            waitTime = 30f;
        }
        else
        {
            waitTime = 0f;
        }
        Vector3 center = new Vector3(terrain_manager.myInfo.x_high / 2, 0, terrain_manager.myInfo.z_high / 2);
        Target = center - transform.position;
        my_path = new List<Vector3>();
        path_index = 0;

        if (ReducedDrones)
        {
            if (SimplePath)
            {
                simple_path = new List<Vector3>();
                simple_path.Add(intersection[IdentifyQuardant(transform.position)-1]); // start point
                // find closest intersection to start
                float close = float.MaxValue;
                Vector3 close_intersection = C;
                foreach (var wp in intersection)
                {
                    if (Vector3.Distance(wp, transform.position) < close)
                    {
                        close = Vector3.Distance(wp, transform.position);
                        close_intersection = wp;
                    }
                }
                simple_path.Add(close_intersection);
                simple_path.Add(C); // Add center
                // find closest intersection to goal
                close = float.MaxValue;
                close_intersection = C;
                foreach (var wp in intersection)
                {
                    if (Vector3.Distance(wp, my_goal_object.transform.position) < close)
                    {
                        close = Vector3.Distance(wp, transform.position);
                        close_intersection = wp;
                    }
                }
                simple_path.Add(close_intersection);
                simple_path.Add(my_goal_object.transform.position);
                my_path = simple_path;
            }
            else
            {
                // Get RoadMap
                road_map = new GraphEmbedding(terrain_manager.terrain_filename, 1, 1f);
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
                //my_path = pathUpsampling(my_path, 2);
            }
            Target = my_path[0] - transform.position;

        }

        // Draw Blue road grid
        if (DroneNumber == 0) road_map.draw_embedding(terrain_manager.myInfo);
    }


    private void FixedUpdate()
    {
        if (!WaitOver && mazeTimer > waitTime)
        {
            //Debug.Log("Waiting done for drone: " + DroneNumber + " at time " + mazeTimer);
            WaitOver = true;
        }

        if (WaitOver && ReducedDrones)
        {
            UpdateCntr++;
            if (UpdateCntr > 10)
            {
                Vector3 GoalDir = my_path[path_index] - transform.position; // Goal Direction
                Target = AvoidCollision(GoalDir);
                UpdateCntr = 0;
            }


            // Update to new index
            if (Vector3.Distance(transform.position, my_path[path_index]) < 6f)
            {
                path_index = Mathf.Min(path_index + 1, my_path.Count - 1);
            }


            // Regulate Velocity and move
            Vector3 velocity = GetComponent<Rigidbody>().velocity;
            if (velocity.magnitude > velocityThreshold)
            {
                m_Drone.Move_vect(-velocity); // Reduce velocity

            }
            else
            {
                m_Drone.Move_vect(Target); // Go towards
            }

            Debug.DrawLine(transform.position, transform.position + Target);
        }



        Debug.DrawLine(C, L, Color.red);
        Debug.DrawLine(C, R, Color.red);
        Debug.DrawLine(C, U, Color.red);
        Debug.DrawLine(C, D, Color.red);


        //return Final time
        mazeTimer += Time.deltaTime;
        if (!MazeComplete && Vector3.Distance(m_Drone.transform.position, my_goal_object.transform.position) < 6f)
        {
            Debug.Log("Goal reached for drone: " + DroneNumber + " at time " + mazeTimer);
            MazeComplete = true;
        }
    }

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
        else if ((pos.x <= (C.x - l)) && (pos.z >= (C.z + l)))
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
        for (int i = 0; i < corners.Count - 1; i++)
        {
            Debug.DrawLine(corners[i], corners[i + 1], Color.magenta);
        }

        return q;
    }
    private Vector3 AvoidCollision(Vector3 relVect)
    {
        float closest = float.MaxValue, dist;
        // Find distance to closet friend
        foreach (GameObject frnd in friends)
        {
            if (gameObject != frnd)
            {
                dist = Vector3.Distance(gameObject.transform.position, frnd.transform.position);
                if (dist < closest)
                {
                    closest = dist;
                }
            }
        }

        // Deflect if too close to closest friend
        if (closest < distanceThreshold)
        {
            return new Vector3(-relVect.z, 0.0f, relVect.x); // deflect

        }
        else
        {
            return relVect;
        }

    }
    // FUNC: Identify Quardant
    private int IdentifyQuardant(Vector3 pos)
    {
        Vector3 center = new Vector3((terrain_manager.myInfo.x_high / 2) * terrain_manager.myInfo.x_N, 0, (terrain_manager.myInfo.z_high / 2) * terrain_manager.myInfo.z_N);
        return -1;
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


