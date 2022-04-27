using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
//using Panda;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private Rigidbody m_Car_Rigidbody;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends; // use these to avoid collisions

        public GameObject my_goal_object;

        private GraphEmbedding road_map;
        private int CarNumber, wayPoint;
        private PathPlanner planner = new PathPlanner();
        private List<Vector3> my_path;

        // PD Variables
        private Vector3 targetSpeed, desiredSpeed, targetPos_K1;
        private float k_p = 2f, k_d = 0.5f;

        // Panda Behaviour Tree
        //PandaBehaviour SoccerBT;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            m_Car_Rigidbody = GetComponent<Rigidbody>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            road_map = new GraphEmbedding(terrain_manager.terrain_filename, 1, 1);
            List<Vector3> initial_path = planner.plan_path(transform.position, terrain_manager.myInfo.start_pos, road_map, terrain_manager.myInfo);



            friends = GameObject.FindGameObjectsWithTag("Car");
            for (int i = 0; i < friends.Length; i++)
            { 
                if(friends[i].transform.position == m_Car.transform.position)
                {
                    CarNumber = i;
                    break;
                }
            }

            my_path = new List<Vector3>();
            my_path = initial_path;
            wayPoint = 1;

            /*
            Vector3 start_pos = transform.position; // terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;
            my_path.Add(start_pos);

            for (int i = 0; i < 3; i++)
            {
                Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                my_path.Add(waypoint);
            }
            my_path.Add(goal_pos);


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                //Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
            */


        }


        private void FixedUpdate()
        {

            //road_map.draw_embedding(terrain_manager.myInfo);
            if(wayPoint < my_path.Count && CarNumber % 5 == 0)
            {
                DriveCar(my_path[wayPoint]);
                if(Vector3.Distance(m_Car.transform.position,my_path[wayPoint]) <= 0.5f + m_Car.CurrentSpeed / 40)
                {
                    wayPoint = Mathf.Min(wayPoint + 1, my_path.Count - 1);
                }
            }

            if(Vector3.Distance(m_Car.transform.position, my_path[wayPoint]) < 0f)
            {
                Debug.Log("Goal reached for car: " + CarNumber);
                //UnityEditor.EditorApplication.isPlaying = false;
            }


            /*
            Vector3 relVect = my_goal_object.transform.position - transform.position;
            bool is_in_front = Vector3.Dot(transform.forward, relVect) > 0f;
            bool is_to_right = Vector3.Dot(transform.right, relVect) > 0f;

            if(is_in_front && is_to_right)
                m_Car.Move(1f, 1f, 0f, 0f);
            if(is_in_front && !is_to_right)
                m_Car.Move(-1f, 1f, 0f, 0f);
            if(!is_in_front && is_to_right)
                m_Car.Move(-1f, -1f, -1f, 0f);
            if(!is_in_front && !is_to_right)
                m_Car.Move(1f, -1f, -1f, 0f);
            */



        }

        /// FUNCTIONS
        // PD Tracker for Car
        private void DriveCar(Vector3 targetPos)
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

        void OnDrawGizmos()
        {
            if(my_path!=null && CarNumber % 5 == 0)
            {
                Gizmos.color = Color.yellow;
                for(int i =0; i < my_path.Count; i++)
                {
                    Gizmos.DrawLine(my_path[i], my_path[i + 1]);
                }
            }
        }
    }
}
