using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI_NoRoads : MonoBehaviour
{
    private DroneController m_Drone; // the car controller we want to use
    public GameObject my_goal_object; // stores a Sphere. it's "transform" contains the Vec3 coordinates of the goal
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    int UpdateCntr = 0;
    public Vector3 Target;
    public GameObject[] friends;
    private bool MazeComplete;
    private int DroneNumber;
    private float mazeTimer, velocityThreshold = 12f, distanceThreshold = 6f;

    private void Start()
    {
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
        Target = my_goal_object.transform.position - transform.position;
    }


    private void FixedUpdate()
    {
        // update only once in 10 calls
        UpdateCntr++;
        if (UpdateCntr > 10)
        {
            Vector3 GoalDir = my_goal_object.transform.position - transform.position; // Goal Direction
            Target = AvoidCollision(GoalDir);
            UpdateCntr = 0;
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

        //return Final time
        mazeTimer += Time.deltaTime;
        if (!MazeComplete && Vector3.Distance(m_Drone.transform.position, my_goal_object.transform.position) < 6f)
        {
            Debug.Log("Goal reached for drone: " + DroneNumber + " at time " + mazeTimer);
            MazeComplete = true;
        }

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
}



