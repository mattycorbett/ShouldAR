using System.Collections;
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.MagicLeap;
using UnityEngine.InputSystem;
using TMPro;

public class IMUTracking : MonoBehaviour
{
    [TooltipAttribute("Smoothing Window Size (Larger value = more smoothing)")]
    public int smoothingWindowSize = 6;

    public int respiratoryFreqCounter;
    public Vector3 acceleration;
    public Vector3 angVelocity;
    private int frameCounter = 0;

    List<float> numbersList = new List<float>();

    // Start is called before the first frame update
    void Start()
    {
        InputSystem.EnableDevice(Accelerometer.current);
        InputSystem.EnableDevice(UnityEngine.InputSystem.Gyroscope.current);
        StartCoroutine(RespiratoryFrequencyCountLoop());
    }

    // Update is called once per frame
    void Update()
    {

        frameCounter += 1;

        acceleration = Accelerometer.current.acceleration.ReadValue();
        angVelocity = UnityEngine.InputSystem.Gyroscope.current.angularVelocity.ReadValue();
        float sumOfSquares = (float)Math.Sqrt(Math.Pow(acceleration.x, 2) + Math.Pow(acceleration.y, 2) + Math.Pow(acceleration.z, 2));
        if (numbersList.Count == 0)
        {
            numbersList.Add(sumOfSquares);
        }
        else if(numbersList[numbersList.Count - 1] != sumOfSquares)
        {
            numbersList.Add(sumOfSquares);
            //numbersList.Add((float)acceleration.y);
            //Debug.Log((float)acceleration.y);
        }


        
        //Debug.Log("Accel:" + acceleration);
        //Debug.Log("Angular Velocity:" + angVelocity);
    }

    List<float> findPeaks(List<float> numbers)
    {
        List<float> peaks = new List<float>();
        for (int i = 0; i < numbers.Count; i++)
        {
            /*if (i == 0)
            {
                if (numbers[i] >= numbers[i + 1])
                {
                    peaks.Add(numbers[i]);
                }
            }*/

            if (i > 0 && i < (numbersList.Count - 1))
            {
                if (numbers[i] >= numbers[i + 1] && numbers[i] >= numbers[i - 1])
                {
                    peaks.Add(numbers[i]);
                }
            }

            /* if (i >= (numbersList.Count - 1))
             {
                 if (numbers[i] >= numbers[i - 1])
                 {
                     peaks.Add(numbers[i]);
                 }
             }*/
        }
        return peaks;
    }

    IEnumerator RespiratoryFrequencyCountLoop()
    {
        while (true)
        {
            Debug.Log(frameCounter + " frames last 60 seconds.");
            frameCounter = 0;
            float[] filteredSignal = new float[numbersList.Count];
            filteredSignal = MovingAverage(numbersList.ToArray(), smoothingWindowSize);
            respiratoryFreqCounter = findPeaks(filteredSignal.ToList()).Count;
            //Debug.Log("There were " + findPeaks(filteredSignal.ToList()).Count + " breaths in the last 60 seconds.");
            numbersList.Clear();
            yield return new WaitForSeconds(60);
        }
    }

    public float[] MovingAverage(float[] data, int size)
    {
        float[] filter = new float[data.Length];
        for (int i = (int)size/2; i < data.Length - (size/2); i++)
        {
            float mean = 0;
            for (var j = i - (size/2); j < i + (size/2); j++)
                mean += data[j];

            filter[i] = mean / size;
        }
        return filter;
    }
}

