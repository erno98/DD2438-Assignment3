using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Panda;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAISoccer_gr1 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private Rigidbody m_Car_Rigidbody;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public string friend_tag;
        public GameObject[] enemies;
        public string enemy_tag;

        public GameObject own_goal;
        public GameObject other_goal;
        public GameObject ball;

        public float dist;
        public float maxKickSpeed = 40f;
        public float lastKickTime = 0f;

        // PD Variables
        private Vector3 targetSpeed, desiredSpeed, targetPos_K1;
        private float k_p=2f, k_d=0.5f;

        // Panda Behaviour Tree
        PandaBehaviour SoccerBT;

        private void Start()
        {
            // get Panda Behaviour Tree
            SoccerBT = GetComponent<PandaBehaviour>();

            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            m_Car_Rigidbody = GetComponent<Rigidbody>();



            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friend_tag = gameObject.tag;
            if (friend_tag == "Blue")
                enemy_tag = "Red";
            else
                enemy_tag = "Blue";

            friends = GameObject.FindGameObjectsWithTag(friend_tag);
            enemies = GameObject.FindGameObjectsWithTag(enemy_tag);

            ball = GameObject.FindGameObjectWithTag("Ball");

        }
        private void Update()
        {
            // Behaviour Tree Reset/Tick
            SoccerBT.Reset();
            SoccerBT.Tick();
        }

        private void FixedUpdate()
        {

            Vector3 avg_pos = Vector3.zero;

            foreach (GameObject friend in friends)
            {
                avg_pos += friend.transform.position;
            }
            avg_pos = avg_pos / friends.Length;
            //Vector3 direction = (avg_pos - transform.position).normalized;
            Vector3 direction = (ball.transform.position - transform.position).normalized;
            direction = ball.transform.position; //temp!!!!!

            DriveCar(direction);
            
            // ALL DEBUG LINES
            Debug.DrawLine(transform.position, ball.transform.position, Color.black);
            Debug.DrawLine(transform.position, own_goal.transform.position, Color.green);
            Debug.DrawLine(transform.position, other_goal.transform.position, Color.yellow);
            Debug.DrawLine(transform.position, friends[0].transform.position, Color.cyan);
            Debug.DrawLine(transform.position, enemies[0].transform.position, Color.magenta);
            if (CanKick())
            {
                Debug.DrawLine(transform.position, ball.transform.position, Color.red);
                //KickBall(maxKickSpeed * Vector3.forward);
            }


            // this is how you kick the ball (if close enough)
            // Note that the kick speed is added to the current speed of the ball (which might be non-zero)
            Vector3 kickDirection = (other_goal.transform.position - transform.position).normalized;

            // replace the human input below with some AI stuff
            if (Input.GetKeyDown("space"))
            {
                KickBall(maxKickSpeed * kickDirection);
            }
        }

        private bool CanKick()
        {
            dist = (transform.position - ball.transform.position).magnitude;
            return dist < 7f && (Time.time - lastKickTime) > 0.5f;
        }

        private void KickBall(Vector3 velocity)
        {
            // impulse to ball object in direction away from agent
            if (CanKick())
            {
                velocity.y = 0f;
                if (Vector3.Dot(velocity, ball.transform.position - transform.position) < 0f)
                {
                    print("Must kick away from agent");
                    velocity = Vector3.zero;
                }
                Rigidbody rb = ball.GetComponent<Rigidbody>();
                rb.AddForce(velocity, ForceMode.VelocityChange);
                lastKickTime = Time.time;
                print("ball was kicked ");

            }

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

        //// Behaviour Tree Tasks /////
        ///    IsGoalie, Defend 0.3, IsChaser, IsBallCloserThan 10.0, Dribble, InterceptBall
        ///    
        [Task]
        bool IsGoalie()
        {
            Debug.Log("Is Goalie");
            return true;
        }

        [Task]
        bool IsChaser()
        {
            Debug.Log("Not chaser");
            return false;
        }

        [Task]
        bool IsBallCloserThan(float distance)
        {
            bool close = Vector3.Distance(ball.transform.position, m_Car.transform.position) > distance;
            Debug.Log("Ball is closer: " + close);
            return close;
        }

        [Task]
        void Defend(float distance)
        {
            Debug.Log("Defending");
        }

        [Task]
        void InterceptBall()
        {
            Debug.Log("Intercept");
        }

        [Task]
        void Dribble()
        {
            Debug.Log("Dribbling");
        }
    }
}
