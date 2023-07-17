using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.MagicLeap;
using UnityEngine.InputSystem;
using TMPro;



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
    #endregion

    #region public variables
    public int blinkRate;
    public string currentEyeBehavior = "None";
    public double behaviorDuration = 0.0;
    public double behaviorVelocity = 0.0;
    #endregion

    private void Awake()
    {
        permissionCallbacks.OnPermissionGranted += OnPermissionGranted;
        permissionCallbacks.OnPermissionDenied += OnPermissionDenied;
        permissionCallbacks.OnPermissionDeniedAndDontAskAgain += OnPermissionDenied;
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
        InputSubsystem.Extensions.MLEyes.StartTracking();

        mlInputs = new MagicLeapInputs();
        mlInputs.Enable();
        eyesActions = new MagicLeapInputs.EyesActions(mlInputs);

        blinkCounter = 0;

        StartCoroutine(BlinkRateCountLoop());
    }
    void Update()
    {
        var eyes = eyesActions.Data.ReadValue<UnityEngine.InputSystem.XR.Eyes>();
        //Access input such as eyes.fixationPoint.
        //Debug.Log(eyes.fixationPoint);

        if (!eyesDevice.isValid)
        {
            //Locate the input device using the FindMagicLeapDevice Util
            this.eyesDevice = InputSubsystem.Utils.FindMagicLeapDevice(InputDeviceCharacteristics.EyeTracking | InputDeviceCharacteristics.TrackedDevice);
            return;
        }
        //Query for Eye Tracking State data
        InputSubsystem.Extensions.TryGetEyeTrackingState(eyesDevice, out InputSubsystem.Extensions.MLEyes.State trackingState);
        MLResult gazeStateResult = MLGazeRecognition.GetState(out MLGazeRecognition.State recognitionState);

        // Debug.Log("Fixation Confidence: " + trackingState.FixationConfidence);
        //Access the trackingState data
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

        if(recognitionState.Behavior.ToString() == "Saccade")
        {
            //Debug.Log("Saccade of amplitude " + recognitionState.VelocityDegps.ToString() + " for " + recognitionState.DurationS.ToString() + " seconds.");
            currentEyeBehavior = "Saccade";
            behaviorDuration = recognitionState.DurationS;
            behaviorVelocity = recognitionState.VelocityDegps;
        }

        else if (recognitionState.Behavior.ToString() == "Pursuit")
        {
            //Debug.Log("Pursuit of amplitude " + recognitionState.VelocityDegps.ToString() + " for " + recognitionState.DurationS.ToString() + " seconds.");
            currentEyeBehavior = "Pursuit";
            behaviorDuration = recognitionState.DurationS;
            behaviorVelocity = recognitionState.VelocityDegps;
        }
        else
        {
            currentEyeBehavior = "None";
            behaviorDuration = (double)0.0;
            behaviorVelocity = (double)0.0;
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

    IEnumerator BlinkRateCountLoop()
    {
        while (true)
        {
            blinkRate = blinkCounter * 4;
            Debug.Log("Current Blink Rate: " + blinkRate);
            blinkCounter = 0;
            yield return new WaitForSeconds(15);
        }
    }
}
