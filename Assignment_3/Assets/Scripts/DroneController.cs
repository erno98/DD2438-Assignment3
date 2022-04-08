using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;


public class DroneController : MonoBehaviour
{

    public Vector3 velocity;

    public float max_speed = 15f;
    public float max_acceleration = 15f;
    public Vector3 desired_force;

    private float v = 0f; //desired acceleration first component
    private float h = 0f; //desired acceleration second component

    private Rigidbody rb;

    public void Move(float h_in, float v_in)
    { 
        h = h_in * 1000f * max_acceleration;
        v = v_in * 1000f * max_acceleration;
        desired_force = new Vector3(h, 0f, v);
        if(desired_force.magnitude > 1000f * max_acceleration)
        {
            desired_force = desired_force.normalized * 1000f * max_acceleration;
        }
    }

    public void Move_vect(Vector3 desired_relative_force)
    {
        if(desired_relative_force.magnitude > 1f)
        {
            desired_force = desired_relative_force.normalized * 1000f * max_acceleration;
        }
        else
        {
            desired_force = desired_relative_force * 1000f * max_acceleration;
        }
        desired_relative_force.y = 0f; //removing vertical component
            
    }

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        velocity = rb.velocity; // for debugging purposes
        rb.AddForce(desired_force);

        //Debug.Log(velocity.magnitude);
    }

    

    
}
