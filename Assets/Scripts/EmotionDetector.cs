#if !(PLATFORM_LUMIN && !UNITY_EDITOR)

#if !UNITY_WSA_10_0


using System;
using System.Collections.Generic;
using System.Collections;
using UnityEngine;
using Microsoft.ML.OnnxRuntime;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.DnnModule;
using OpenCVForUnity.MlModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.UtilsModule;
using OpenCVForUnity.UnityUtils.Helper;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using Microsoft.ML.OnnxRuntime.Tensors;
using System.Threading.Tasks;




public class EmotionDetector : MonoBehaviour
{

    [TooltipAttribute("Path to a binary file of model contains trained weights. It could be a file with extensions .caffemodel (Caffe), .pb (TensorFlow), .t7 or .net (Torch), .weights (Darknet).")]
    public string model;

    /// <summary>
    /// The net.
    /// </summary>
    protected SVM net;
    protected string model_filepath;
    private InferenceSession session;
    private byte[] model_bytes;

    //test input data to test model
    float[] testData = new float[49] {2.55813953e-01F, 3.57288636e+03F, 4.18167360e+03F, 1.73187059e+00F, 1.84680000e+04F, 2.03400000e+03F, 2.55813953e-01F, 3.37022727e+02F, 3.87244679e+02F, 1.77637593e+00F, 1.69900000e+03F, 2.27384235e+01F,
                3.14871975e+01F, 2.27596405e+00F, 1.37337012e+02F, 3.24452170e-02F, 2.12982239e+00F, 5.56550370e-02F, 3.13539789e+00F, 2.27407321e+01F, 3.14903944e+01F, 2.27596405e+00F, 1.37350956e+02F, 5.23255810e-02F,
                8.97777778e+01F, 3.88998001e+01F, 6.45564359e-01F, 1.58000000e+02F, 1.52906977e+00F, 7.69760440e+01F, 4.91369075e+01F, 1.21131419e+00F, 2.54035851e+02F, 6.54831724e+00F, 1.18673735e+01F, 5.14366839e+00F,
                1.10369230e+02F, -4.43390316e+00F, 1.03633209e+02F, -2.99443960e-02F, 1.79343727e+02F, 1.05973979e+00F, 1.25719858e+01F, -1.67415957e+00F,6.85087990e+01F, -3.63974232e-01F, 4.95164742e+00F, -5.58809022e+00F,
                1.35796963e+01F};


    protected virtual void Start()
    {

       if (!string.IsNullOrEmpty(model))
       {
            //load model
            //model_filepath = Path.Combine(Application.streamingAssetsPath, model);
            model_filepath = Utils.getFilePath("OpenCVForUnity/dnn/" + model);
            if (string.IsNullOrEmpty(model_filepath))
            {
                Debug.Log("The file:" + model + " did not exist in the folder.");
            }

            
            
       }

        Run();
    }

    protected virtual void Run()
    {

        if (string.IsNullOrEmpty(model_filepath))
        {
           Debug.LogError(model + " is not loaded”. ");
        }
        else
        {
            //! [Initialize network]
            // Create an InferenceSession from the Model Path.
            try
            {
                //var serializedModel = LoadModelFromEmbeddedResource(model_filepath);
                //string serializedPath = SaveModelToEmbeddedResource(serializedModel, model_filepath);
                //net = SVM.load(serializedPath);
                var options = new SessionOptions();
                options.LogSeverityLevel = OrtLoggingLevel.ORT_LOGGING_LEVEL_INFO;
                options.GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL;
                options.ExecutionMode = ExecutionMode.ORT_SEQUENTIAL;
                options.EnableMemoryPattern = false;
                options.AppendExecutionProvider_DML(0);

                session = new InferenceSession(model_filepath, options);
            }
            catch (Exception e)
            {
                Debug.Log(e.StackTrace);
            }

            //! [Initialize network]
        }

    }

    void OnEnable()
    {

    }

    void OnDisable()
    {

    }

    static byte[] LoadModelFromEmbeddedResource(string path)
    {
        byte[] model = null;

        using (Stream stream = File.OpenRead(path))
        {
            using (MemoryStream memoryStream = new MemoryStream())
            {
                stream.CopyTo(memoryStream);
                model = memoryStream.ToArray();
            }
        }

        return model;
    }

    static string SaveModelToEmbeddedResource(byte[] serializedModel, string model_filepath)
    {
        string model = null;
        string changedFileExtesion = Path.ChangeExtension(model_filepath, ".xml");
        using (Stream stream = File.OpenWrite(changedFileExtesion))
        {
            using (MemoryStream memoryStream = new MemoryStream())
            {
                memoryStream.Write(serializedModel, 0, serializedModel.Length);

            }
        }

        return changedFileExtesion;
    }

    // Update is called once per frame
    protected void Update()
    {
        //Debug.Log(model_filepath);
        Tensor<float> input = new DenseTensor<float>(new int[] { 1, 49 });
        for (int y = 0; y < 49; y++)
        {
            //load tensor with test data
            input[0, y] = testData[y];
            //Debug.Log(input[0, y]);
        }
        try
        {
            var results = GetPrediction(input);
            //var results = GetPredictionOpenCV();
            //print inferred result
            Debug.Log(results.First());
        }
        catch (Exception e)
        {
            //Debug.Log(e.StackTrace);
        }



    }

    protected void postprocess()
    {


    }

    List<Mat> GetPredictionOpenCV()
    {

        // Setup inputs
        Mat blob = new Mat(new Size(1, 49), CvType.CV_32FC1);
        List<Mat> outs = new List<Mat>();
        List<string> output_names = new List<string>();


        for (int y = 0; y < 49; y++)
        {
            //load tensor with test data
            blob.put(0, y, testData[y]);
            //Debug.Log(input[0, y]);
        }

        //Debug.Log("blob=" + blob.dump());
        //load tensor with test data
 




        // Run inference
        try
        {
            // Run a model.
            net.predict(blob);
        }
        catch (Exception e)
        {
            Debug.Log(e.Message);
        }


        return outs;

    }
    
    System.Collections.Generic.IEnumerable<Int64> GetPrediction(Tensor<float> input)
    {
        System.Collections.Generic.IEnumerable<Int64> results = null;

        // Setup inputs
        var inputs = new List<NamedOnnxValue>
            {
                NamedOnnxValue.CreateFromTensor("float_input", input)
            };

        // Run inference
        try
        {
            results = session.Run(inputs).First().AsEnumerable<Int64>();
        }
        catch (Exception e)
        {
            Debug.Log(e.Message + e.StackTrace);
        }


        return results;

    }

}

#endif

#endif
