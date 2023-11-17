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
        public int staleLimit = 3000;
        Renderer rend;
        public int bystanderGazeHits = 0;
        public int hitThreshold = 10;
        public Transform lastHitPosition;

        void Start()
        {
            rend = GetComponent<Renderer>();
            rend.enabled = false;
            detectionStopwatch = new Stopwatch();
            
        }


        public void RemoveDetection()
        {
            Destroy(this.gameObject);
        }

        void Update()
        {

            //If object has existed for more than the given threshold without update, we treat it as stale and remove
            if (bystanderGazeHits > hitThreshold)
            {
                makeChevronVisible();
                transform.LookAt(lastHitPosition, Camera.main.transform.up);
                bystanderGazeHits = 0;
            }

            if (detectionStopwatch.ElapsedMilliseconds > staleLimit)
            {
                makeChevronInvisible();
            }


        }

        public void makeChevronVisible()
        {
            
            rend.enabled = true;
            detectionStopwatch.Start();
        }

        public void makeChevronInvisible()
        {
            detectionStopwatch.Reset();
            rend.enabled = false;
        }




    }
}