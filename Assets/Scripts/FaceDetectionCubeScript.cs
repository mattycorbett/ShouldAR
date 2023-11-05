using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using System;
using System.Diagnostics;

namespace Mediapipe.Unity
{
    public class FaceDetectionCubeScript : MonoBehaviour
    {
        public Stopwatch detectionStopwatch;
        public int staleLimit = 100;

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

            //If object has existed for more than the given threshold without update, we treat it as stale and remove

            if (detectionStopwatch.ElapsedMilliseconds > staleLimit)
            {
                RemoveDetection();
            }


        }

        //If this gameobject is in the same physical space another bounding box, we compare the time they have existed and remove the older one
        void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.tag == "BoundingBox")
            {
                //UnityEngine.Debug.Log("Collision");
                if (collision.gameObject.GetComponent<FaceDetectionCubeScript>().detectionStopwatch.ElapsedMilliseconds < this.detectionStopwatch.ElapsedMilliseconds)
                {
                    RemoveDetection();
                }
                else
                {
                    //staleCounter = 0;
                }
            }


        }


    }
}