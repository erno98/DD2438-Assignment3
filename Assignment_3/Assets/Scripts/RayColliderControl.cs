using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class RayController
{
    public Vector3 relative_direction;
    public float max_distance,k_r;
  


    public RayController(Vector3 relative_direction, float max_distance, float k_r)
    {
       

        this.relative_direction=relative_direction;
        this.max_distance=max_distance;
        this.k_r=k_r;
    }

    public Vector3 desired_acceleration(Vector3 position, Vector3 direction) // the direction is supposed to be a TRANSFORMED version of this.relative direction!
    {
         // heavily based on CarAI script

        RaycastHit hit;
        Vector3 acc=new Vector3(0F,0F,0F);
        
        if (Physics.Raycast(position, direction, out hit, this.max_distance))
        {
            acc = direction * (this.max_distance-hit.distance)*(-1F)*this.k_r*(0.1F+10F*(float)Math.Pow(1-(hit.distance-2F)/this.max_distance,3F));
        }

        return acc;
    }

    public Vector3 desired_acceleration_drone(Vector3 position, Vector3 direction) // the direction is supposed to be a TRANSFORMED version of this.relative direction!
    {
         // heavily based on CarAI script

        RaycastHit hit;
        Vector3 acc=new Vector3(0F,0F,0F);
        
        if (Physics.Raycast(position, direction, out hit, this.max_distance))
        {
            acc = direction * (this.max_distance-hit.distance)*(-0.5F)*this.k_r*(0.1F+10F*(float)Math.Pow(1-hit.distance/this.max_distance,2F)+10F*(float)Math.Pow(1-(hit.distance-2F)/this.max_distance,3F));
        }

        return acc;
    }
}