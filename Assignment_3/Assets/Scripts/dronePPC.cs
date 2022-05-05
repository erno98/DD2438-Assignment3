using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class drone_PP_controller
{
    public polygon_path path;
    public float lookahead, max_deviation,k_p,k_d,v,padding;

    public drone_PP_controller(polygon_path _path, float _lookahead, float padding, float coarseness, float max_deviation, float k_p, float k_d, float v) // costructor that also does the preprocessing on the path
    
    // 1) linearly smooth path
    // 2) determine to what extent to increase resoultion - the max distance will roughly be the coarseness*lookahead
    // 3) after init, the update member function gives driing commands upon entering current position


    {
        this.lookahead=_lookahead;
        this.max_deviation=max_deviation;
        this.path=_path;
        this.k_p=k_p;
        this.v=v;
        this.k_d=k_d;
        this.padding=padding;
       

        // linear smoothing

        this.path=this.path.linear_smoothing(this.max_deviation); // ASSUMED  THAT SAME PADDING IS USED

        // set speed limit

        this.path.set_speed_limit_drone();

        // hi-res generation for the tracker

        float resolution_goal=this.path.wp.max_segment_length()/coarseness/_lookahead; // note: no units 
        int current_resolution=1;

        while((float)current_resolution<resolution_goal){
            this.path.double_resolution();
            current_resolution=current_resolution*2;
        }

        // setting speed limit

        this.path.smooth_speed_limit_drone();


    }

    public Vector3 desired_acceleration(Vector3 position, Vector3 velocity, Vector3 right, Vector3 forward,float a_max) // PD tracking of lookahead and keeping constant velocity, heavy copying from original (lecture) script
    {
        Vector3 lookahead_postion=this.path.lookahead_3d(position,this.lookahead);
        Vector3 position_error= lookahead_postion-position;

        Vector3 velocity_error=this.v*this.path.desired_driving_direction_3d(position)-velocity;

        // PD tracker, copied 

        Vector3 desired_acceleration = this.k_p * position_error + this.k_d * velocity_error;
        Vector3 n_velocity=new Vector3(1,0,0);

        if(velocity.magnitude>0.001F){
            n_velocity=velocity/velocity.magnitude;
        }

        
        Vector3 acceleration = Vector3.Dot(desired_acceleration, n_velocity)*n_velocity;
        Vector3 steering = desired_acceleration-acceleration;

        

        if(velocity.magnitude>this.path.find_speed_limit3d(position,this.lookahead)*Mathf.Sqrt(a_max/15F)){
       /*      if(Vector3.Dot(velocity,n_velocity)>0F){ // going forward, needs brake
                acceleration=-1F*this.k_d*(velocity.magnitude-this.path.find_speed_limit3d(position,this.lookahead))*n_velocity;
            }
            else{ // going backward, needs gas
                acceleration=this.k_d*(velocity.magnitude-this.path.find_speed_limit3d(position,this.lookahead))*forward;
            } */

            acceleration=-1F*this.k_d*(velocity.magnitude-this.path.find_speed_limit3d(position,this.lookahead))*n_velocity;
            
        }

        return steering+acceleration;

       
    } 

    public bool is_lookahead_blocked(Vector3 position){

        Vector3 lookahead_postion=this.path.lookahead_3d(position,this.lookahead);
        Vector2 ahead=new Vector2(lookahead_postion.x,lookahead_postion.z);
        Vector2 pos=new Vector2(position.x,position.z);

        Line l=new Line(pos,ahead);
        EmbeddedLine el=new EmbeddedLine(l,this.padding,this.path._terrain_filename);

        return el.intersects_non_transversable();
    }

      public List<float> acceleration_to_command_drone(Vector3 desired_acceleration, Vector3 right, Vector3 forward)
    {
        float horizontal = desired_acceleration.x;
        float vertical = desired_acceleration.z;

        float[] commands={horizontal, vertical};

        return new List<float>(commands);
    }

}

