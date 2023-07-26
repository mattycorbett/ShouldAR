using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class CSVLogger : MonoBehaviour
{

    #region Constants to modify
    private const string CSVHeader = "Timestamp,TimeInMs,BlinkRate(perminute),CurrentEyeBehavior,LengthofBehavior,VelocityofBehavior,RespiratoryFrequenct(perminute),CurrentAccelX,CurrentAccelY,CurrentAccelZ,CurrentAngVelX,CurrentAngVelY,CurrentAngVelZ";
    #endregion

    StreamWriter s;
    FileStream f;
    EyeTracking eyeTrackingScript;
    IMUTracking IMUtrackingScript;

    #region public variables
    public GameObject eyeTrackerObject;
    public GameObject IMUTrackerObject;
    #endregion

    // Start is called before the first frame update
    void Start()
    {
        eyeTrackingScript = eyeTrackerObject.GetComponent<EyeTracking>();
        IMUtrackingScript = IMUTrackerObject.GetComponent<IMUTracking>();
        StartLogger();
        StartCoroutine(DataRecorderLoop());
    }

    // Update is called once per frame
    void Update()
    {
        s.Flush();
        
    }

    IEnumerator DataRecorderLoop()
    {
        while (true)
        {
            if (s != null)
            {
                if (eyeTrackerObject.activeSelf && IMUTrackerObject.activeSelf)
                {
                    WriteLine(System.DateTime.Now.ToString("MM_dd_yyyy__HH_mm_ss") + "," + System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() + "," + eyeTrackingScript.blinkRate.ToString()
                        + "," + eyeTrackingScript.currentEyeBehavior + "," + eyeTrackingScript.behaviorDuration.ToString() + "," + eyeTrackingScript.behaviorVelocity.ToString()
                         + "," + IMUtrackingScript.respiratoryFreqCounter
                         + "," + IMUtrackingScript.acceleration.x.ToString() + "," + IMUtrackingScript.acceleration.y.ToString() + "," + IMUtrackingScript.acceleration.z.ToString()
                        + "," + IMUtrackingScript.angVelocity.x.ToString() + "," + IMUtrackingScript.angVelocity.y.ToString() + "," + IMUtrackingScript.angVelocity.z.ToString());
                }
                else if (eyeTrackerObject.activeSelf && !IMUTrackerObject.activeSelf)
                {
                    WriteLine(System.DateTime.Now.ToString("MM_dd_yyyy__HH_mm_ss") + "," + System.DateTime.Now.Ticks / System.TimeSpan.TicksPerMillisecond + "," + eyeTrackingScript.blinkRate.ToString()
                        + "," + eyeTrackingScript.currentEyeBehavior + "," + eyeTrackingScript.behaviorDuration.ToString() + "," + eyeTrackingScript.behaviorVelocity.ToString());
                }
                else
                {
                    WriteLine(System.DateTime.Now.ToString("MM_dd_yyyy__HH_mm_ss") + "," + System.DateTime.Now.Ticks / System.TimeSpan.TicksPerMillisecond + "," + ""
                        + "," + "" + "," + "" + "," + "" + ","
                        + "," + IMUtrackingScript.respiratoryFreqCounter
                         + "," + IMUtrackingScript.acceleration.x.ToString() + "," + IMUtrackingScript.acceleration.y.ToString() + "," + IMUtrackingScript.acceleration.z.ToString()
                        + "," + IMUtrackingScript.angVelocity.x.ToString() + "," + IMUtrackingScript.angVelocity.y.ToString() + "," + IMUtrackingScript.angVelocity.z.ToString());
                }

            }
            yield return new WaitForSeconds(.01F);

        }
    }
    IEnumerator DataFlushLoop()
    {
        while (true)
        {
            s.Flush();
            yield return new WaitForSeconds(2);
        }
    }
    public void StartLogger()
    {
        string fileName = System.DateTime.Now.ToString("MM_dd_yyyy__HH_mm_ss") + ".csv";
        StartLogger(fileName);
    }

    /// <summary>
    /// Start capturing video to input filename.
    /// </summary>
    /// <param name="fileName">File path to write the video to.</param>
    public void StartLogger(string fileName)
    {
        // Check file fileName extensions
        string extension = System.IO.Path.GetExtension(fileName);
        if (string.IsNullOrEmpty(extension) || !extension.Equals(".csv", System.StringComparison.OrdinalIgnoreCase))
        {
            Debug.LogErrorFormat("Invalid fileName extension '{0}' passed into Logger({1}).\n" +
                "Logs must be saved in {2} format.", extension, fileName, ".csv");
            return;
        }

        string pathName = System.IO.Path.Combine(Application.persistentDataPath, fileName);
        Debug.Log("Logging .csv to path: " + pathName);
        f = new FileStream(pathName, FileMode.OpenOrCreate);
        s = new StreamWriter(f);
        s.WriteLine(CSVHeader);
        s.Flush();
    }

    public void WriteLine(string line)
    {
        s.WriteLine(line);
        
    }

    private void OnDestroy()
    {
        s.Close();
        f.Close();
    }
}
