using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.Barracuda;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAISoccer_gr2 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

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

        public static Model model = new Model();

        // model takes:
        // xy of 1st enemy (float, 0-1) -> 2 variables
        // xy of 2nd enemy (float, 0-1) -> 2 variables
        // xy of 3rd enemy (float, 0-1) -> 2 variables
        // xy of the ball (float, 0-1) -> 2 variables
        // in total tensor of 8

        // outputs:
        // handbrake, steer, throttle (float 0-1) for each car
        // in total tensor of 9

        public static IWorker engine;
        public static Dictionary<int, List<float>> moves = new Dictionary<int, List<float>>();
        public int car_idx;
        public static int leader_car_idx = 0;
        public static int car_count;
        public static bool can_start = false;

        private void Start()
        {

            car_idx = car_count;
            if (car_idx == 2)
            {
                can_start = true;
            }
            car_count++;
            moves.Add(car_idx, new List<float>() { 0.0f, 0.0f, 0.0f });

            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


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


            if (car_idx == leader_car_idx)
            {
                model = ModelLoader.Load((NNModel)Resources.Load("model"));
                engine = WorkerFactory.CreateWorker(model, WorkerFactory.Device.GPU);
            }
            // Plan your path here
            // ...
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

        private float standardize_pos(float pos, float min, float max)
        {
            return (pos - min) / (max - min);
        }

        private void FixedUpdate()
        {

            if (!can_start)
            {
                return;
            }

            if (car_idx == leader_car_idx)
            {
                var input = new Tensor(1, 1, 1, 8);
                var tensor8D = new Tensor(new TensorShape(1, 1, 1, 1, 1, 1, 8, 1));
                var max_x = other_goal.transform.position[0];
                var min_x = own_goal.transform.position[0];
                var max_y = other_goal.transform.position[2] + 50.0f;
                var min_y = own_goal.transform.position[2] - 50.0f; ;

                tensor8D[0] = standardize_pos(enemies[0].transform.position[0], min_x, max_x); // enemy1 x
                tensor8D[1] = standardize_pos(enemies[0].transform.position[2], min_x, max_x); // enemy1 y
                tensor8D[2] = standardize_pos(enemies[1].transform.position[0], min_x, max_x);  // enemy2 x
                tensor8D[3] = standardize_pos(enemies[1].transform.position[2], min_x, max_x);  // enemy2 y
                tensor8D[4] = standardize_pos(enemies[2].transform.position[0], min_x, max_x);  // enemy3 x
                tensor8D[5] = standardize_pos(enemies[2].transform.position[2], min_x, max_x);  // enemy3 y
                tensor8D[6] = standardize_pos(ball.transform.position[0], min_x, max_x);  // ball x
                tensor8D[7] = standardize_pos(ball.transform.position[2], min_x, max_x);  // ball y

                var output = engine.Execute(tensor8D).PeekOutput();

                Debug.Log("Predicted for 1st car: " + output[0] + ", " + output[1] + ", " + output[2]);
                moves[0] = new List<float>() { output[0], output[1], output[2] };
                moves[1] = new List<float>() { output[3], output[4], output[5] };
                moves[2] = new List<float>() { output[6], output[7], output[8] };

            }

            // this is how you access information about the terrain
            // int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            // int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            // float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            // float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            var this_moves = moves[car_idx];

            m_Car.Move(this_moves[0], this_moves[1], this_moves[2], 0f);


        }
    }
}