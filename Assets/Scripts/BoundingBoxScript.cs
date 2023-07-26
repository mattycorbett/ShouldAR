using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using System;
using System.Diagnostics;

namespace ShouldAR
{
    public class BoundingBoxScript : MonoBehaviour
    {
        public int staleCounter;
        public Stopwatch detectionStopwatch;


        void Start()
        {
            detectionStopwatch = new Stopwatch();
            detectionStopwatch.Start();
        }


        public void RemoveDetection()
        {
            Destroy(this.gameObject);
        }

        void Update()
        {
            staleCounter += 1;

            //If object has existed for more than the given threshold without update, we treat it as stale and remove
            if (staleCounter > 60)
            {
                RemoveDetection();
            }

 

        }


        //If this gameobject is in the same physical space another bounding box, we compare the time they have existed and remove the older one
        /*void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.tag == "BoundingBox")
            {
                if (collision.gameObject.GetComponent<BoundingBoxScript>().detectionStopwatch.ElapsedMilliseconds < this.detectionStopwatch.ElapsedMilliseconds)
                {
                    staleCounter = 0;
                    this.gameObject.transform.position = collision.gameObject.transform.position;
                    Destroy(collision.gameObject);
                }
                else
                {
                    staleCounter = 0;
                }
            }


        }*/

    }
}