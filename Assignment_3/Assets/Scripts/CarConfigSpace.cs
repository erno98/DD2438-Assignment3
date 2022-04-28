using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class CarConfigSpace
    {
        private Vector3 boxSize;
        private Vector3 padding = new Vector3(0.5f, 0, 0.5f);

        public Vector3 BoxSize
        {
            get
            {
                return boxSize;
            }

            set
            {
                boxSize = value + padding;
            }
        }

        //Checks for a collision at a given point and direction
        public bool Collision(float x, float z, float theta)
        {
            Vector3 position = new Vector3(x, 3, z);
            Quaternion rotation = Quaternion.Euler(0, theta, 0);
            return Physics.CheckBox(position, BoxSize / 2, rotation);
        }
    }
}
