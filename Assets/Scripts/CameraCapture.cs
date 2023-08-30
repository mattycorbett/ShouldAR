// %BANNER_BEGIN%
// ---------------------------------------------------------------------
// %COPYRIGHT_BEGIN%
// Copyright (c) (2019-2022) Magic Leap, Inc. All Rights Reserved.
// Use of this file is governed by the Software License Agreement, located here: https://www.magicleap.com/software-license-agreement-ml2
// Terms and conditions applicable to third-party materials accompanying this distribution may also be found in the top-level NOTICE file appearing herein.
// %COPYRIGHT_END%
// ---------------------------------------------------------------------
// %BANNER_END%

using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using UnityEngine.XR.MagicLeap;
using UnityEngine.Networking;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.DnnModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.UtilsModule;
using OpenCVForUnity.UnityUtils.Helper;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.UnityUtils;



namespace ShouldAR
{

    #region ConversionExtension
    //https://docs.microsoft.com/en-us/windows/mixed-reality/develop/unity/unity-xrdevice-advanced?tabs=mrtk
    public static class NumericsConversionExtensions
    {
        public static UnityEngine.Vector3 ToUnity(this System.Numerics.Vector3 v) => new UnityEngine.Vector3(v.X, v.Y, -v.Z);
        public static UnityEngine.Quaternion ToUnity(this System.Numerics.Quaternion q) => new UnityEngine.Quaternion(q.X, q.Y, -q.Z, -q.W);
        public static UnityEngine.Matrix4x4 ToUnity(this System.Numerics.Matrix4x4 m) => new UnityEngine.Matrix4x4(
            new Vector4(m.M11, m.M12, -m.M13, m.M14),
            new Vector4(m.M21, m.M22, -m.M23, m.M24),
            new Vector4(-m.M31, -m.M32, m.M33, -m.M34),
            new Vector4(m.M41, m.M42, -m.M43, m.M44));

        public static System.Numerics.Vector3 ToSystem(this UnityEngine.Vector3 v) => new System.Numerics.Vector3(v.x, v.y, -v.z);
        public static System.Numerics.Quaternion ToSystem(this UnityEngine.Quaternion q) => new System.Numerics.Quaternion(q.x, q.y, -q.z, -q.w);
        public static System.Numerics.Matrix4x4 ToSystem(this UnityEngine.Matrix4x4 m) => new System.Numerics.Matrix4x4(
            m.m00, m.m10, -m.m20, m.m30,
            m.m01, m.m11, -m.m21, m.m31,
           -m.m02, -m.m12, m.m22, -m.m32,
            m.m03, m.m13, -m.m23, m.m33);

        public static System.Numerics.Vector3 ToSystemWithoutConversion(UnityEngine.Vector3 vector) => new System.Numerics.Vector3(vector.x, vector.y, vector.z);
    }

    #endregion

    /// <summary>
    /// This class handles video recording and loading based on controller
    /// input.
    /// </summary>
    public class CameraCapture : MonoBehaviour
    {

        [SerializeField, Tooltip("Refrence to the Raw Video Capture Visualizer gameobject for RGB frames")]
        private CameraCaptureVisualizer cameraCaptureVisualizer = null;

        [TooltipAttribute("Facial Detection GameObject")]
        public GameObject detectionComponent;

        [TooltipAttribute("Facial Detection Cube Prefab")]
        public GameObject faceCube;

        [TooltipAttribute("Rear Camera Facial Detection Cube Prefab")]
        public GameObject rearFaceCube;

        [TooltipAttribute("Bystander Eye Gaze Debug Prefab")]
        public GameObject eyeGazeSphere;

        [TooltipAttribute("Temp GameObject")]
        public GameObject tempGO;

        [TooltipAttribute("Enable Facial Detection Using Front-Facing Camera")]
        public bool enableFrontDetection;

        [TooltipAttribute("Enable Facial Detection Using Rear-Facing Camera")]
        public bool enableRearDetection;

        [TooltipAttribute("How often to locate faces? (in terms of frames)")]
        public int inferenceRate;

        [SerializeField, Tooltip("Desired width for the front camera capture")]
        private int frontCaptureWidth = 1280;

        [SerializeField, Tooltip("Desired height for the front camera capture")]
        private int frontCaptureHeight = 720;

        [SerializeField, Tooltip("Desired width for the back camera capture")]
        private int backCaptureWidth = 800;

        [SerializeField, Tooltip("Desired height for the back camera capture")]
        private int backCaptureHeight = 600;

        private bool isCameraConnected;
        private MLCamera.StreamCapability selectedCapability;

        private MagicLeapInputs mlInputs;
        private MagicLeapInputs.ControllerActions controllerActions;

        private MLCamera colorCamera;
        private bool cameraDeviceAvailable = false;
        private bool isCapturing;
        private string poseText;

        private readonly MLPermissions.Callbacks permissionCallbacks = new MLPermissions.Callbacks();

        private Texture2D rawVideoTexturesRGBA;

        private float currentAspectRatio;

        private int samplingCounter = 0;

        float averageFaceWidthInMeters = 0.18f;

        /// <summary>
        /// Using Awake so that Permissions is set before PermissionRequester Start.
        /// </summary>
        void Awake()
        {

            StartCoroutine(FramerateCountLoop());

            if (cameraCaptureVisualizer == null)
            {
                Debug.LogError("Error: CVCameraExample._rawVideoCaptureVisualizer is not set, disabling script.");
                enabled = false;
                return;
            }

            mlInputs = new MagicLeapInputs();
            mlInputs.Enable();
            controllerActions = new MagicLeapInputs.ControllerActions(mlInputs);

            isCapturing = false;

            permissionCallbacks.OnPermissionGranted += OnPermissionGranted;
            permissionCallbacks.OnPermissionDenied += OnPermissionDenied;
            permissionCallbacks.OnPermissionDeniedAndDontAskAgain += OnPermissionDenied;

            MLPermissions.RequestPermission(MLPermission.Camera, permissionCallbacks);

#if UNITY_EDITOR

#endif
        }

        /// <summary>
        /// Stop the camera, unregister callbacks, and stop input and permissions APIs.
        /// </summary>
        void OnDisable()
        {
            permissionCallbacks.OnPermissionGranted -= OnPermissionGranted;
            permissionCallbacks.OnPermissionDenied -= OnPermissionDenied;
            permissionCallbacks.OnPermissionDeniedAndDontAskAgain -= OnPermissionDenied;

            controllerActions.Bumper.performed -= OnButtonDown;
            mlInputs.Dispose();

            if (colorCamera != null && isCameraConnected)
            {
                DisableMLCamera();
            }
        }

        /// <summary>
        /// Display permission error if necessary or update status text.
        /// </summary>
        private void Update()
        {

        }

        IEnumerator GetBackCameraFrame(string uri, Matrix4x4 outMatrix, MLCameraBase.IntrinsicCalibrationParameters intrinsics)
        {
            using (UnityWebRequest webRequest = new UnityWebRequest(uri, UnityWebRequest.kHttpVerbGET))
            {
                webRequest.downloadHandler = new DownloadHandlerTexture();
                // Request and wait for the desired page.
                yield return webRequest.SendWebRequest();


                switch (webRequest.result)
                {
                    case UnityWebRequest.Result.ConnectionError:
                    case UnityWebRequest.Result.DataProcessingError:
                        Debug.Log(": Error: " + webRequest.error);
                        break;
                    case UnityWebRequest.Result.ProtocolError:
                        Debug.Log(": HTTP Error: " + webRequest.error);
                        break;
                    case UnityWebRequest.Result.Success:
                        //Debug.Log("Received JPG of Size: " + webRequest.downloadHandler.data.Length);
                        Texture2D newImage = new Texture2D(backCaptureWidth, backCaptureHeight);
                        
                        newImage = DownloadHandlerTexture.GetContent(webRequest);
                        //GameObject.Find("TestQuad").GetComponent<Renderer>().material.mainTexture = newImage;
                        var faces = detectionComponent.GetComponent<FacialDetection>().RunDetection(newImage, newImage.width, newImage.height);
                        var averagePixelsForFaceAt1Meter = 300;

                        foreach ((Vector2, float, Point[]) face in faces)
                        {

                            float estimatedFaceDepth = averagePixelsForFaceAt1Meter / face.Item2;

                            var CenterLandmarkPosition = CastRayFromPixelToWorldPoint(newImage.width, newImage.height,
                                face.Item1, intrinsics, 75.0f, outMatrix, 1, false);

                            var newObject = Instantiate(tempGO, CenterLandmarkPosition, Quaternion.identity);
                            float gaze_XAngle = Vector2.Angle(new Vector2((float)face.Item3[0].x - (float)face.Item3[1].x, (float)face.Item3[0].y - (float)face.Item3[1].y), face.Item1);
                            float gaze_YAngle = Vector2.Angle(new Vector2((float)face.Item3[2].x, (float)face.Item3[2].y), face.Item1);
                            newObject.transform.Rotate(gaze_XAngle * -1, gaze_YAngle, 0);
                            newObject.transform.RotateAround(outMatrix.GetPosition(), Camera.main.transform.up, 180);

                            rearFaceCube.transform.position = newObject.transform.position;

                            Ray bystanderGazeRay = new Ray(rearFaceCube.transform.position, rearFaceCube.transform.forward);
                            eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);


                            /*MatOfPoint3f op = new MatOfPoint3f();
                            List<Point3> tempOP = new List<Point3>();
                            for (int i = 0; i < face.Item3.Length; i++)
                            {
                                var CenterLandmarkPosition = CastRayFromPixelToWorldPoint(newImage.width, newImage.height, 
                                    new Vector2((float)face.Item3[i].x, (float)face.Item3[i].y), intrinsics, 75.0f, outMatrix, 1, false);

                                tempOP.Add(
                                    new Point3(
                                        CenterLandmarkPosition.x,
                                        CenterLandmarkPosition.y,
                                        CenterLandmarkPosition.z
                                    )
                                );
                            }
                            op.fromList(tempOP);

                            MatOfPoint2f ip = new MatOfPoint2f();
                            List<Point> tempIP = new List<Point>(); 
                            for (int i = 0; i < face.Item3.Length; i++)
                            {
                                tempIP.Add(
                                    new Point(
                                        face.Item3[i].x,
                                        face.Item3[i].x
                                    )
                                );
                            }
                            ip.fromList(tempIP);
                            Mat rvec = new Mat(1, 3, CvType.CV_64FC1);
                            Mat tvec = new Mat(1, 3, CvType.CV_64FC1);
                            Mat cameraMat = new Mat(3, 3, CvType.CV_64FC1);


                            List<double> _rVecList = new List<double>();
                            List<double> _tVecList = new List<double>();

                            cameraMat.put(0,0, outMatrix[0,0], outMatrix[0,1], outMatrix[0,2], outMatrix[0,3], 
                            outMatrix[1,0], outMatrix[1,1], outMatrix[1,2], outMatrix[1,3],
                            outMatrix[2,0], outMatrix[2,1], outMatrix[2,2], outMatrix[2,3],
                            outMatrix[3,0], outMatrix[3,1], outMatrix[3,2], outMatrix[3,3]);

                            Calib3d.solvePnP(op,ip, cameraMat, new MatOfDouble(), rvec, tvec);

                            Converters.Mat_to_vector_double(rvec, _rVecList);
                            Converters.Mat_to_vector_double(tvec, _tVecList);

                            var poseData = ARUtils.ConvertRvecTvecToPoseData(_rVecList, _tVecList);

                            Debug.Log(poseData.pos);
                            var newObject = Instantiate(tempGO, poseData.pos, poseData.rot);
                            rearFaceCube.transform.position = newObject.transform.position;*/

                            //Ray bystanderGazeRay = new Ray(rearFaceCube.transform.position, rearFaceCube.transform.forward);
                            //eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);
                            /*if (Physics.Raycast(rearFaceCube.transform.position, rearFaceCube.transform.TransformDirection(Vector3.forward), out hit, Mathf.Infinity))
                            {
                                eyeGazeSphere.transform.position = hit.position;
                                Debug.Log("Did Hit");
                            }
                            else
                            {
            
                                Debug.Log("Did not Hit");
                            }*/

                            //Debug.DrawRay(newObject.transform.position, Camera.main.transform.position, Color.green, 1, false);

                            GameObject.Destroy(newImage);
                            GameObject.Destroy(newObject);
                        }
                        break;
                }
            }
        }

        /// <summary>
        /// Captures a still image using the device's camera and returns
        /// the data path where it is saved.
        /// </summary>
        /// <param name="fileName">The name of the file to be saved to.</param>
        private void StartVideoCapture()
        {
            MLCamera.OutputFormat outputFormat = MLCamera.OutputFormat.RGBA_8888;
            MLCamera.CaptureConfig captureConfig = new MLCamera.CaptureConfig();
            captureConfig.CaptureFrameRate = MLCamera.CaptureFrameRate._30FPS;
            captureConfig.StreamConfigs = new MLCamera.CaptureStreamConfig[1];
            captureConfig.StreamConfigs[0] = MLCamera.CaptureStreamConfig.Create(selectedCapability, outputFormat);
            MLResult result = colorCamera.PrepareCapture(captureConfig, out MLCamera.Metadata _);
            if (result.IsOk)
            {
                result = colorCamera.PreCaptureAEAWB();
                result = colorCamera.CaptureVideoStart();
                if (!result.IsOk)
                {
                    Debug.LogError("Failed to start video capture!");
                }
                else
                {
                    cameraCaptureVisualizer.DisplayCapture(outputFormat, false);
                }
            }

            isCapturing = result.IsOk;
        }

        private void StopVideoCapture()
        {
            if (isCapturing)
            {
                colorCamera.CaptureVideoStop();
                cameraCaptureVisualizer.HideRenderer();
            }

            isCapturing = false;
        }

        /// <summary>
        /// Connects the MLCamera component and instantiates a new instance
        /// if it was never created.
        /// </summary>
        private IEnumerator EnableMLCamera()
        {
            if (colorCamera != null)
            {
                yield return null;
            }

            while (!cameraDeviceAvailable)
            {
                MLResult result = MLCamera.GetDeviceAvailabilityStatus(MLCamera.Identifier.CV, out cameraDeviceAvailable);
                if (!(result.IsOk && cameraDeviceAvailable))
                {
                    // Wait until camera device is available
                    yield return new WaitForSeconds(1.0f);
                }
            }

            Debug.Log("Camera device available");
            yield return new WaitForSeconds(1.0f);

            MLCamera.ConnectContext context = MLCamera.ConnectContext.Create();
            context.EnableVideoStabilization = true;
            context.CamId = MLCamera.Identifier.CV;

            colorCamera = MLCamera.CreateAndConnect(context);
            if (colorCamera != null)
            {
                Debug.Log("Camera device connected");
                isCameraConnected = true;
                MLCamera.StreamCapability[] streamCapabilities = MLCamera.GetImageStreamCapabilitiesForCamera(colorCamera, MLCamera.CaptureType.Video);
                if (streamCapabilities == null || streamCapabilities.Length <= 0)
                {
                    Debug.LogError("Camera device unable to received stream caps.");
                    yield break;
                }

                if (!MLCamera.TryGetBestFitStreamCapabilityFromCollection(streamCapabilities, frontCaptureWidth, frontCaptureHeight,
                    MLCamera.CaptureType.Video, out selectedCapability))
                {
                    Debug.LogError("Camera device unable to fit stream caps to chosen options.");
                    yield break;
                }

                Debug.Log("Camera device received stream caps");
                colorCamera.OnRawVideoFrameAvailable += OnCaptureRawVideoFrameAvailable;
                controllerActions.Bumper.performed += OnButtonDown;
            }
        }

        /// <summary>
        /// Disconnects the MLCamera if it was ever created or connected.
        /// </summary>
        private void DisableMLCamera()
        {
            if (colorCamera != null)
            {
                colorCamera.OnRawVideoFrameAvailable -= OnCaptureRawVideoFrameAvailable;
                colorCamera.Disconnect();
                // Explicitly set to false here as the disconnect was attempted.
                isCameraConnected = false;
                colorCamera = null;
            }
        }

        /// <summary>
        /// Handles the event for button down.
        /// </summary>
        /// <param name="controllerId">The id of the controller.</param>
        /// <param name="button">The button that is being pressed.</param>
        private void OnButtonDown(InputAction.CallbackContext obj)
        {
            if (!isCapturing)
            {
                StartVideoCapture();
                cameraCaptureVisualizer.HideRenderer();
            }
            else
            {
                StopVideoCapture();
            }
        }

        public Vector3 CastRayFromPixelToWorldPoint(int width, int height, Vector2 pixelPosition, MLCameraBase.IntrinsicCalibrationParameters parameters, float FOV, Matrix4x4 cameraTransformationMatrix, float depth, bool frontCamera = true)
        {
            Vector2 normalizedImagePoint;
            // Step 1: Normalize the image coordinates
            if (frontCamera)
            {
                normalizedImagePoint.x = (pixelPosition.x - parameters.PrincipalPoint.x) / width;
                normalizedImagePoint.y = (pixelPosition.y - parameters.PrincipalPoint.y) / height;

            }
            else
            {
                normalizedImagePoint.x = (pixelPosition.x - width / 2) / width;
                normalizedImagePoint.y = (pixelPosition.y - height / 2) / height;
            }


            // Account for aspect ratio
            normalizedImagePoint.x *= width / (float)height;

            // Account for FOV
            float fovRad = FOV * Mathf.Deg2Rad;
            normalizedImagePoint *= Mathf.Tan(fovRad / 2);

            Vector3 cameraPoint = new Vector3(normalizedImagePoint.x, normalizedImagePoint.y, 1);
            // Step 2: Convert normalized image coordinates to camera coordinates
            //Vector3 cameraPoint = new Vector3(normalizedImagePoint.x, normalizedImagePoint.y, 1);

            // Step 3: Create a 3D ray from camera position in the direction of the camera coordinate
            Ray cameraRay = new Ray(Vector3.zero, cameraPoint);

            // Step 4: Convert the ray to world coordinates
            Quaternion rotation = cameraTransformationMatrix.rotation;
            Vector3 position = cameraTransformationMatrix.MultiplyPoint(cameraRay.origin);
            Vector3 direction = rotation * cameraRay.direction;
            Ray worldRay = new Ray(position, direction);

            // Return the point in world space at the specified depth along the ray
            return worldRay.GetPoint(depth);
        }

        /// <summary>
        /// Handles the event of a new image getting captured.
        /// </summary>
        /// <param name="imageData">The raw data of the image.</param>
        private void OnCaptureRawVideoFrameAvailable(MLCamera.CameraOutput capturedFrame, MLCamera.ResultExtras resultExtras, MLCamera.Metadata metadataHandle)
        {
            MLCamera.FlipFrameVertically(ref capturedFrame);
            MLResult result = MLCVCamera.GetFramePose(resultExtras.VCamTimestamp, out Matrix4x4 outMatrix);

            samplingCounter += 1;

            if (enableRearDetection && samplingCounter >= inferenceRate)
            {
                samplingCounter = 0;
                StartCoroutine(GetBackCameraFrame("http://10.0.0.231/uploads/esp32-cam.jpg", outMatrix, resultExtras.Intrinsics.Value));
            }


            if (enableFrontDetection && samplingCounter >= inferenceRate)
            {
                var pixelsPerMeterAlongX = resultExtras.Intrinsics.Value.FocalLength.x;
                var averagePixelsForFaceAt1Meter = pixelsPerMeterAlongX * averageFaceWidthInMeters;
                samplingCounter = 0;
                UpdateRGBTexture(ref rawVideoTexturesRGBA, capturedFrame.Planes[0]);
                var faces = detectionComponent.GetComponent<FacialDetection>().RunDetection(rawVideoTexturesRGBA, frontCaptureWidth, frontCaptureHeight);
                
                if (result.IsOk && resultExtras.Intrinsics != null && faces.Any())
                {
                    uint width = capturedFrame.Planes[0].Width;
                    uint height = capturedFrame.Planes[0].Height;
                    Vector2 centerPixel = new Vector2(width / 2f, height / 2f);
                    foreach ((Vector2, float, Point[]) face in faces)
                    {
                        float estimatedFaceDepth = averagePixelsForFaceAt1Meter / face.Item2;
                        var CenterFaceposition = CastRayFromPixelToWorldPoint((int)width, (int)height, face.Item1, resultExtras.Intrinsics.Value, resultExtras.Intrinsics.Value.FOV, outMatrix, estimatedFaceDepth);
                        var overlapBoxes = Physics.OverlapBox(CenterFaceposition, faceCube.transform.localScale / 2, Quaternion.identity);
                        if (overlapBoxes.Length > 0 && overlapBoxes[0].gameObject.tag == "BoundingBox")
                        {
                            overlapBoxes[0].gameObject.transform.position = CenterFaceposition;
                            var bboxScript = overlapBoxes[0].gameObject.GetComponent<BoundingBoxScript>();
                            bboxScript.staleCounter = 0;
                        }
                        else
                        {
                            var newObject = Instantiate(faceCube, CenterFaceposition, Quaternion.identity);
 
                        }

                    }

                }

                
            }
            else
            {
                cameraCaptureVisualizer.OnCaptureDataReceived(resultExtras, capturedFrame);
            }
         






        }

        IEnumerator FramerateCountLoop()
        {
            while (true)
            {
                yield return new WaitForSeconds(15);
                Debug.Log("Current Frame Rate: " + (int)(1.0f / Time.smoothDeltaTime));
            }
        }

        private void OnPermissionDenied(string permission)
        {
            MLPluginLog.Error($"{permission} denied, example won't function.");
        }

        private void OnPermissionGranted(string permission)
        {
            StartCoroutine(EnableMLCamera());
        }

        private void UpdateRGBTexture(ref Texture2D videoTextureRGB, MLCamera.PlaneInfo imagePlane)
        {

            int actualWidth = (int)(imagePlane.Width * imagePlane.PixelStride);

            if (videoTextureRGB != null &&
                (videoTextureRGB.width != imagePlane.Width || videoTextureRGB.height != imagePlane.Height))
            {
                Destroy(videoTextureRGB);
                videoTextureRGB = null;
            }

            if (videoTextureRGB == null)
            {
                videoTextureRGB = new Texture2D((int)imagePlane.Width, (int)imagePlane.Height, TextureFormat.RGBA32, false);
                videoTextureRGB.filterMode = FilterMode.Bilinear;
            }

            //SetProperRatio((int)imagePlane.Width, (int)imagePlane.Height, _screenRendererRGB);

            if (imagePlane.Stride != actualWidth)
            {
                var newTextureChannel = new byte[actualWidth * imagePlane.Height];
                for (int i = 0; i < imagePlane.Height; i++)
                {
                    Buffer.BlockCopy(imagePlane.Data, (int)(i * imagePlane.Stride), newTextureChannel, i * actualWidth, actualWidth);
                }
                videoTextureRGB.LoadRawTextureData(newTextureChannel);
            }
            else
            {
                videoTextureRGB.LoadRawTextureData(imagePlane.Data);
            }
            videoTextureRGB.Apply();
        }

        private void SetProperRatio(int textureWidth, int textureHeight, Renderer renderer)
        {
            float ratio = textureWidth / (float)textureHeight;

            if (Math.Abs(currentAspectRatio - ratio) < float.Epsilon)
                return;

            currentAspectRatio = ratio;
            var localScale = renderer.transform.localScale;
            localScale = new Vector3(currentAspectRatio * localScale.y, localScale.y, 1);
            renderer.transform.localScale = localScale;
        }
    }
}

