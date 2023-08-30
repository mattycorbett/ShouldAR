using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class CSVLogger : MonoBehaviour
{

    #region Constants to modify
    private const string CSVHeader = "Timestamp,TimeInMs,Num_of_Fixations,Mean_Fixation_Duration,SD_Fixation_Duration,Skew_Fixation_Duration,Max_Fixation_Duration," +
        "First_Fixation_Duration,Num_of_Saccade,Mean_Saccade_Duration,SD_Saccade_Duration,Skew_Saccade_Duration,Max_Saccade_Duration,Mean_Saccade_Amplitude,SD_Saccade_Amplitude," +
        "Skew_Saccade_Amplitude,Max_Saccade_Amplitude,Mean_Saccade_Direction,SD_Saccade_Direction,Skew_Saccade_Direction,Max_Saccade_Direction,Mean_Saccade_Length,SD_Saccade_Length," +
        "Skew_Saccade_Length,Max_Saccade_Length,Num_of_Blink,Mean_Blink_Duration,SD_Blink_Duration,Skew_Blink_Duration,Max_Blink_Duration,Num_of_Microsac,Mean_Microsac_Peak_Vel," +
        "SD_Microsac_Peak_Vel,Skew_Microsac_Peak_Vel,Max_Microsac_Peak_Vel,Mean_Microsac_Ampl,SD_Microsac_Ampl,Skew_Microsac_Ampl,Max_Microsac_Ampl,Mean_Microsac_Dir,SD_Microsac_Dir," +
        "Skew_Microsac_Dir,Max_Microsac_Dir,Mean_Microsac_H_Amp,SD_Microsac_H_Amp,Skew_Microsac_H_Amp,Max_Microsac_H_Amp,Mean_Microsac_V_Amp,SD_Microsac_V_Amp,Skew_Microsac_V_Amp,Max_Microsac_V_Amp";
    #endregion

    StreamWriter s;
    FileStream f;


    #region public variables

    #endregion

    // Start is called before the first frame update
    void Start()
    {

        StartLogger();

    }

    // Update is called once per frame
    void Update()
    {
        
        
    }

    public void WriteCSVLineFull(float Num_of_Fixations, float Mean_Fixation_Duration, float SD_Fixation_Duration, float Skew_Fixation_Duration, float Max_Fixation_Duration, float First_Fixation_Duration,
        float Num_of_Saccade, float Mean_Saccade_Duration, float SD_Saccade_Duration, float Skew_Saccade_Duration,float Max_Saccade_Duration, float Mean_Saccade_Amplitude,float SD_Saccade_Amplitude, 
        float Skew_Saccade_Amplitude, float Max_Saccade_Amplitude, float Mean_Saccade_Direction, float SD_Saccade_Direction, float Skew_Saccade_Direction, float Max_Saccade_Direction, float Mean_Saccade_Length,
        float SD_Saccade_Length, float Skew_Saccade_Length, float Max_Saccade_Length, float Num_of_Blink, float Mean_Blink_Duration, float SD_Blink_Duration, float Skew_Blink_Duration, float Max_Blink_Duration,
        float Num_of_Microsac, float Mean_Microsac_Peak_Vel, float SD_Microsac_Peak_Vel, float Skew_Microsac_Peak_Vel, float Max_Microsac_Peak_Vel, float Mean_Microsac_Ampl, float SD_Microsac_Ampl, float Skew_Microsac_Ampl,
        float Max_Microsac_Ampl, float Mean_Microsac_Dir, float SD_Microsac_Dir, float Skew_Microsac_Dir, float Max_Microsac_Dir, float Mean_Microsac_H_Amp, float SD_Microsac_H_Amp, float Skew_Microsac_H_Amp, float Max_Microsac_H_Amp,
        float Mean_Microsac_V_Amp, float SD_Microsac_V_Amp, float Skew_Microsac_V_Amp, float Max_Microsac_V_Amp)
    {
        
    }

    public void WriteCSVLineFixations(float Num_of_Fixations, float Mean_Fixation_Duration, float SD_Fixation_Duration, float Skew_Fixation_Duration, float Max_Fixation_Duration, float First_Fixation_Duration)
    {
        WriteLine(System.DateTime.Now.ToString("MM_dd_yyyy__HH_mm_ss") + "," + System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() + "," + Num_of_Fixations.ToString()
                        + "," + Mean_Fixation_Duration.ToString() + "," + SD_Fixation_Duration.ToString() + "," + Skew_Fixation_Duration.ToString() + "," + Max_Fixation_Duration.ToString() + "," + First_Fixation_Duration.ToString());
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
        s.Flush();
    }

    private void OnDestroy()
    {
        s.Close();
        f.Close();
    }
}
