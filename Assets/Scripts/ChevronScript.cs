using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using System;
using System.Diagnostics;

namespace Mediapipe.Unity
{
    public class ChevronScript : MonoBehaviour
    {
        public Stopwatch detectionStopwatch;
        public int staleLimit = 1000;

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



    }
}