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

        // Variables for path & driving
        public GameObject my_goal_object;
        public GameObject[] friends;
        private int CarNumber;
        private bool MazeComplete, WaitOver;
        private float mazeTimer, waitTime, RoadOffset = 4f, distanceThreshold = 3f;
        public Vector3 Target;

        // PD Variables
        private Vector3 targetSpeed, targetPos_K1, currentPos_K1;
        private float k_p = 2f, k_d = 0.5f;

        private void Start()
        {
            // Initialize Variables
            Time.timeScale = 5;
            MazeComplete = false;
            WaitOver = false;

            // get the car controller
            m_Car = GetComponent<CarController>();
            m_Car_Rigidbody = GetComponent<Rigidbody>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

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
            waitTime = ((int)(CarNumber % 25)) * 2.5f;
        }

        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;

            if (!WaitOver && mazeTimer >= waitTime)
            {
                //Debug.Log("Waiting done for car: " + CarNumber + " at time " + mazeTimer);
                WaitOver = true;
            }
            if (WaitOver && !MazeComplete)
            {

                if (Vector3.Distance(m_Car.transform.position, my_goal_object.transform.position) > 10f)
                {
                    Vector3 GoalDir = my_goal_object.transform.position - transform.position;
                    GoalDir = Quaternion.Euler(0, -90, 0) * GoalDir;
                    Target = my_goal_object.transform.position + GoalDir / 24.5f;
                }
                else
                {
                    Target = my_goal_object.transform.position;
                }

                /*
                bool is_in_front = Vector3.Dot(transform.forward, GoalDir) > 0f;
                bool is_to_right = Vector3.Dot(transform.right, GoalDir) > 0f;

                if (is_in_front && is_to_right)
                    m_Car.Move(1f, 1f, 0f, 0f);
                if (is_in_front && !is_to_right)
                    m_Car.Move(-1f, 1f, 0f, 0f);
                if (!is_in_front && is_to_right)
                    m_Car.Move(-1f, -1f, -1f, 0f);
                if (!is_in_front && !is_to_right)
                    m_Car.Move(1f, -1f, -1f, 0f);
                */
                DriveCarPD(Target);

                Debug.DrawLine(transform.position, Target);
            }
            //return Final time
            if (!MazeComplete && Vector3.Distance(m_Car.transform.position, my_goal_object.transform.position) < 5f)
            {
                Debug.Log("Goal reached for car: " + CarNumber + " at time " + mazeTimer);
                MazeComplete = true;
            }

        }

        /// FUNCTIONS
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
            if (m_Car.CurrentSpeed > 25f)
            {
                acceleration = acceleration / 10f;
            }
            m_Car.Move(steering, acceleration, acceleration, 0f);
        }
    }
}