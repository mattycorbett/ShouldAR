using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.MagicLeap;
using UnityEngine.InputSystem;
using TMPro;
using System.Linq;
using System;



public class EyeTracking : MonoBehaviour
{
    #region private variables
    private MagicLeapInputs mlInputs;
    private MagicLeapInputs.EyesActions eyesActions;
    private UnityEngine.XR.InputDevice eyesDevice;
    private bool currentlyBlinking = false;
    private int blinkCounter;
    // Was EyeTracking permission granted by user
    private bool permissionGranted = false;
    private readonly MLPermissions.Callbacks permissionCallbacks = new MLPermissions.Callbacks();
    private int numberSecondsElapsed;
    private string previousBehavior = "None";
    private List<double> fixationList;
    private float numFixationsPerSecond;
    private float tempFixationDuration;
    
    private float SDFixationDuration;
    private float skewFixationDuration;
    private float firstFixationDuration;

    private float totalSaccades;

    private float totalPursuits;

    private int blinkRate;
    CSVLogger loggingScript;
    #endregion

    #region public variables
    [TooltipAttribute("Enable CSV Logging")]
    public bool enableLogging;

    [TooltipAttribute("User Eye Gaze Debug Prefab")]
    public GameObject userEyeGazeSphere;

    public GameObject CSVGameObject;

    public float meanFixationDuration;
    #endregion

    private void Awake()
    {
        permissionCallbacks.OnPermissionGranted += OnPermissionGranted;
        permissionCallbacks.OnPermissionDenied += OnPermissionDenied;
        permissionCallbacks.OnPermissionDeniedAndDontAskAgain += OnPermissionDenied;

        InvokeRepeating("CounterLoop", 1.0f, 1.0f);
        InvokeRepeating("BlinkRateCountLoop", 1.0f, 60.0f);
        InvokeRepeating("EyeDataRecorderLoop", 60.0f, 60.0f);
    }

    private void OnDestroy()
    {
        permissionCallbacks.OnPermissionGranted -= OnPermissionGranted;
        permissionCallbacks.OnPermissionDenied -= OnPermissionDenied;
        permissionCallbacks.OnPermissionDeniedAndDontAskAgain -= OnPermissionDenied;

        mlInputs.Disable();
        mlInputs.Dispose();

        InputSubsystem.Extensions.MLEyes.StopTracking();
    }

    void Start()
    {
        if (enableLogging)
        {
            loggingScript = CSVGameObject.GetComponent<CSVLogger>();
        }
        
        fixationList = new List<double>();
        InputSubsystem.Extensions.MLEyes.StartTracking();

        mlInputs = new MagicLeapInputs();
        mlInputs.Enable();
        eyesActions = new MagicLeapInputs.EyesActions(mlInputs);

        blinkCounter = 0;

        if (!eyesDevice.isValid)
        {
            //Locate the input device using the FindMagicLeapDevice Util
            this.eyesDevice = InputSubsystem.Utils.FindMagicLeapDevice(InputDeviceCharacteristics.EyeTracking | InputDeviceCharacteristics.TrackedDevice);
            return;
        }

    }
    void Update()
    {
        var eyes = eyesActions.Data.ReadValue<UnityEngine.InputSystem.XR.Eyes>();
        //Access input such as eyes.fixationPoint.
        userEyeGazeSphere.transform.position = eyes.fixationPoint;
        // Debug.Log("Fixation Confidence: " + trackingState.FixationConfidence);

        //Access the trackingState data
        InputSubsystem.Extensions.TryGetEyeTrackingState(eyesDevice, out InputSubsystem.Extensions.MLEyes.State trackingState);
        MLResult gazeStateResult = MLGazeRecognition.GetState(out MLGazeRecognition.State recognitionState);

        var currentEyeBehavior = recognitionState.Behavior.ToString();

        if (currentEyeBehavior == "Saccade")
        {
            if(previousBehavior != currentEyeBehavior)
            {
                //if a fixation has ended, computer new totals and averages
                if (previousBehavior == "Fixation")
                {
                    if (tempFixationDuration >= 300F)
                    {
                        //If first fixation
                        if (fixationList.Count == 0)
                        {
                            firstFixationDuration = tempFixationDuration;
                        }
                        fixationList.Add((double)tempFixationDuration);
                        tempFixationDuration = 0F;
                    }              
 
                }
            }
            previousBehavior = "Saccade";

        }
        else if (currentEyeBehavior == "Fixation")
        {
            if(previousBehavior != currentEyeBehavior)
            {
                
                
            }
            previousBehavior = "Fixation";
            tempFixationDuration = recognitionState.DurationS * 1000;
        }

        if ((trackingState.RightBlink || trackingState.LeftBlink))
        {
            if (!currentlyBlinking)
            {
                blinkCounter++;
                currentlyBlinking = true;
            }
        }
        else
        {
            currentlyBlinking = false;
        }



    }

    private void OnPermissionDenied(string permission)
    {
        MLPluginLog.Error($"{permission} denied, example won't function.");
    }

    private void OnPermissionGranted(string permission)
    {
        InputSubsystem.Extensions.MLEyes.StartTracking();
        eyesActions = new MagicLeapInputs.EyesActions(mlInputs);
        permissionGranted = true;
    }

    void BlinkRateCountLoop()
    {
        blinkRate = blinkCounter / 60;
        blinkCounter = 0;
    }

    void EyeDataRecorderLoop()
    {
        if (fixationList.Any())
        {
            numFixationsPerSecond = (float)((float)fixationList.Count() / (float)numberSecondsElapsed);
            meanFixationDuration = (float)(fixationList.Sum() / (float)fixationList.Count());
            SDFixationDuration = standardDeviation(fixationList.ToArray(), fixationList.Count());
            skewFixationDuration = skewness(fixationList.ToArray(), fixationList.Count());
            if (enableLogging)
            {
                loggingScript.WriteCSVLineFixations(numFixationsPerSecond, meanFixationDuration, SDFixationDuration, skewFixationDuration, (float)fixationList.Max(), firstFixationDuration);

            }


        }
    }
    void CounterLoop()
    {
        //Debug.Log(numberSecondsElapsed);
        numberSecondsElapsed += 1;
    }
    

    // Function to calculate standard
    // deviation of data.
    static float standardDeviation(double[] arr,
                                            int n)
    {

        double sum = 0;

        // find standard deviation
        // deviation of data.
        for (int i = 0; i < n; i++)
            sum = (arr[i] - mean(arr, n)) *
                  (arr[i] - mean(arr, n));

        return (float)Math.Sqrt(sum / n);
    }

    // Function to calculate skewness.
    static float skewness(double []arr, int n)
    {
        // Find skewness using
        // above formula
        double sum = 0;
         
        for (int i = 0; i < n; i++)
            sum = (arr[i] - mean(arr, n)) *
                  (arr[i] - mean(arr, n)) *
                  (arr[i] - mean(arr, n));            
         
        return (float)sum / (n * standardDeviation(arr, n) *
                        standardDeviation(arr, n) *
                        standardDeviation(arr, n) *
                        standardDeviation(arr, n));
    }

    // Function to calculate
    // mean of data.
    static float mean(double[] arr, int n)
    {
        double sum = 0;

        for (int i = 0; i < n; i++)
            sum = sum + arr[i];

        return (float)sum / n;
    }
}
