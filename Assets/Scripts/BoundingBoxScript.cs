using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using System;
using System.Diagnostics;

namespace Mediapipe.Unity
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


    }
}