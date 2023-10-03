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
using System.Numerics;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using UnityEngine.XR.MagicLeap;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.UtilsModule;
using OpenCVForUnity.UnityUtils.Helper;
using OpenCVForUnity.Calib3dModule;


namespace Mediapipe.Unity
{


    /// <summary>
    /// This class handles video recording and loading based on controller
    /// input.
    /// </summary>
    public class CameraCapture : MonoBehaviour
    {

        [TooltipAttribute("Facial Detection Cube Prefab")]
        public GameObject faceCube;

        [TooltipAttribute("Rear Camera Facial Detection Cube Prefab")]
        public GameObject rearFaceCube;

        [TooltipAttribute("Bystander Eye Gaze Debug Prefab")]
        public GameObject eyeGazeSphere;

        [TooltipAttribute("Temp GameObject")]
        public GameObject tempGO;

        [SerializeField, Tooltip("Enable ML2 Front Camera?")]
        private bool enableFrontDetection = false;

        [SerializeField, Tooltip("Desired width for the front camera capture")]
        private int frontCaptureWidth = 1280;

        [SerializeField, Tooltip("Desired height for the front camera capture")]
        private int frontCaptureHeight = 720;

        [SerializeField, Tooltip("Width (in pixels) for the rear camera capture")]
        private int rearCaptureWidth = 800;

        [SerializeField, Tooltip("Height (in pixels) for the rear camera capture")]
        private int rearCaptureHeight = 600;


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
        /// The cameraparam matrix.
        /// </summary>
        Mat camMatrix;

        /// <summary>
        /// The distortion coeffs.
        /// </summary>
        MatOfDouble distCoeffs;

        /// <summary>
        /// Using Awake so that Permissions is set before PermissionRequester Start.
        /// </summary>
        void Awake()
        {

            //set cameraparam
            int max_d = (int)Mathf.Max(rearCaptureWidth, rearCaptureHeight);
            double fx = max_d;
            double fy = max_d;
            double cx = rearCaptureWidth / 2.0f;
            double cy = rearCaptureHeight / 2.0f;
            camMatrix = new Mat(3, 3, CvType.CV_64FC1);
            camMatrix.put(0, 0, fx);
            camMatrix.put(0, 1, 0);
            camMatrix.put(0, 2, cx);
            camMatrix.put(1, 0, 0);
            camMatrix.put(1, 1, fy);
            camMatrix.put(1, 2, cy);
            camMatrix.put(2, 0, 0);
            camMatrix.put(2, 1, 0);
            camMatrix.put(2, 2, 1.0f);
            Debug.Log("camMatrix " + camMatrix.dump());

            distCoeffs = new MatOfDouble(0, 0, 0, 0);
            Debug.Log("distCoeffs " + distCoeffs.dump());

            float imageSizeScale = (float)rearCaptureHeight / (float)rearCaptureWidth;

            Size imageSize = new Size(rearCaptureWidth * imageSizeScale, rearCaptureHeight * imageSizeScale);
            double apertureWidth = 0;
            double apertureHeight = 0;
            double[] fovx = new double[1];
            double[] fovy = new double[1];
            double[] focalLength = new double[1];
            Point principalPoint = new Point(0, 0);
            double[] aspectratio = new double[1];

            Calib3d.calibrationMatrixValues(camMatrix, imageSize, apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectratio);

            Debug.Log("imageSize " + imageSize.ToString());
            Debug.Log("apertureWidth " + apertureWidth);
            Debug.Log("apertureHeight " + apertureHeight);
            Debug.Log("fovx " + fovx[0]);
            Debug.Log("fovy " + fovy[0]);
            Debug.Log("focalLength " + focalLength[0]);
            Debug.Log("principalPoint " + principalPoint.ToString());
            Debug.Log("aspectratio " + aspectratio[0]);

            StartCoroutine(FramerateCountLoop());

#if UNITY_EDITOR

#else
            mlInputs = new MagicLeapInputs();
            mlInputs.Enable();
            controllerActions = new MagicLeapInputs.ControllerActions(mlInputs);

            isCapturing = false;

            permissionCallbacks.OnPermissionGranted += OnPermissionGranted;
            permissionCallbacks.OnPermissionDenied += OnPermissionDenied;
            permissionCallbacks.OnPermissionDeniedAndDontAskAgain += OnPermissionDenied;

            MLPermissions.RequestPermission(MLPermission.Camera, permissionCallbacks);
#endif
        }

        /// <summary>
        /// Stop the camera, unregister callbacks, and stop input and permissions APIs.
        /// </summary>
        void OnDisable()
        {
#if UNITY_EDITOR

#else
            permissionCallbacks.OnPermissionGranted -= OnPermissionGranted;
            permissionCallbacks.OnPermissionDenied -= OnPermissionDenied;
            permissionCallbacks.OnPermissionDeniedAndDontAskAgain -= OnPermissionDenied;

            controllerActions.Bumper.performed -= OnButtonDown;
            mlInputs.Dispose();

            if (colorCamera != null && isCameraConnected)
            {
                DisableMLCamera();
            }
#endif
        }

        /// <summary>
        /// Display permission error if necessary or update status text.
        /// </summary>
        private void Update()
        {
#if UNITY_EDITOR
            var landmarks = GameObject.Find("Solution").GetComponent<IrisTrackingSolution>().currentFaceAndIrisLandmarks;

            if (landmarks != null)
            {
                StartCoroutine(EstimatePositionAndHeadPose(landmarks));
            }
#endif
        }

        private IEnumerator EstimatePositionAndHeadPose(NormalizedLandmarkList landmarks)
        {
            var faceWidthAt1M = 25;
            var faceWidth = (landmarks.Landmark[454].X * (float)rearCaptureWidth) - (landmarks.Landmark[324].X * (float)rearCaptureWidth);

            var depth = faceWidthAt1M / faceWidth;
            Debug.Log(depth);

            /*    // top of the head
                var rightEyeX = (landmarks.Landmark[469].X + landmarks.Landmark[470].X + landmarks.Landmark[471].X + landmarks.Landmark[472].X) / 4;
                var rightEyeY = (landmarks.Landmark[469].Y + landmarks.Landmark[470].Y + landmarks.Landmark[471].Y + landmarks.Landmark[472].Y) / 4;
                var leftEyeX = (landmarks.Landmark[474].X + landmarks.Landmark[475].X + landmarks.Landmark[476].X + landmarks.Landmark[477].X) / 4;
                var leftEyeY = (landmarks.Landmark[474].Y + landmarks.Landmark[475].Y + landmarks.Landmark[476].Y + landmarks.Landmark[477].Y) / 4;
             //Debug.Log($"Right Eye Image Coordinates: {leftEyeX},{leftEyeY}");
             //Debug.Log($"Left Eye Image Coordinates: {rightEyeX},{rightEyeY}");
             var p280 = new UnityEngine.Vector3(landmarks.Landmark[280].X * 800, landmarks.Landmark[280].Y * 600, landmarks.Landmark[280].Z);
             var p50 = new UnityEngine.Vector3(landmarks.Landmark[50].X * 800, landmarks.Landmark[50].Y * 600, 0);
             var p18 = new UnityEngine.Vector3(landmarks.Landmark[18].X * 800, landmarks.Landmark[18].Y * 600, 0);
             var p4 = new UnityEngine.Vector3(landmarks.Landmark[4].X * 800, landmarks.Landmark[4].Y * 600, 0);

             var YawA = Mathf.Sqrt(Mathf.Pow(p50.x - p18.x, 2) + Mathf.Pow(p50.y - p18.y, 2));
             var YawB = Mathf.Sqrt(Mathf.Pow(p280.x - p18.x, 2) + Mathf.Pow(p280.y - p18.y, 2));

             var PitchA = Mathf.Sqrt(Mathf.Pow(p50.x - p4.x, 2) + Mathf.Pow(p50.y - p4.y, 2));
             var PitchB = Mathf.Sqrt(Mathf.Pow(p280.x - p4.x, 2) + Mathf.Pow(p280.y - p4.y, 2));

            var Ver_Dist4Yaw = Mathf.Sqrt(Mathf.Pow(p4.x - p18.x, 2) + Mathf.Pow(p4.y - p18.y, 2));
            var Ver_Dist4Pitch = Mathf.Sqrt(Mathf.Pow(p50.x - p4.x, 2) + Mathf.Pow(p50.y - p4.y, 2));

             var LineR = Mathf.Sqrt(Mathf.Pow(YawA, 2) - Mathf.Pow(Ver_Dist4Yaw, 2));
             var LineL = Mathf.Sqrt(Mathf.Pow(YawB, 2) - Mathf.Pow(Ver_Dist4Yaw, 2));

             float Yaw = 0;
             float Pitch = 0;
             float PitchThetaLeft = 0;
             float PitchThetaRight = 0;

             if (YawA < YawB)
             {
                 Yaw = Mathf.Asin(1 - (LineR / LineL)) * Mathf.Rad2Deg;
             }
             else
             {
                 Yaw = Mathf.Asin(1 - (LineL / LineR)) * Mathf.Rad2Deg;
             }

             if (YawB > Ver_Dist4Pitch)
             {
                 PitchThetaLeft = Mathf.Asin(Ver_Dist4Pitch / PitchB) * Mathf.Rad2Deg;
             }
             else
             {
                 PitchThetaLeft = Mathf.Asin(PitchB / Ver_Dist4Pitch) * Mathf.Rad2Deg;
             }

             if (YawA > Ver_Dist4Pitch)
             {
                 PitchThetaRight = Mathf.Asin(Ver_Dist4Pitch / PitchA) * Mathf.Rad2Deg;
             }
             else
             {
                 PitchThetaRight = Mathf.Asin(PitchA / Ver_Dist4Pitch) * Mathf.Rad2Deg;
             }

             Pitch = (PitchThetaLeft + PitchThetaRight) / 2;

            if (!float.IsNaN(Yaw) && !float.IsNaN(Pitch))
            {
                rearFaceCube.transform.eulerAngles = new UnityEngine.Vector3(Pitch, Yaw, 0);
            }*/


            Mat rvec = new Mat(3, 1, CvType.CV_64FC1);
            Mat tvec = new Mat(3, 1, CvType.CV_64FC1);
            Mat rmat = new Mat(3, 3, CvType.CV_64FC1);

            OpenCVForUnity.CoreModule.Point[] imagePoints = new OpenCVForUnity.CoreModule.Point[6];
            imagePoints[0] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[1].X * (float)rearCaptureWidth, landmarks.Landmark[1].Y * (float)rearCaptureHeight);
            imagePoints[1] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[152].X * (float)rearCaptureWidth, landmarks.Landmark[152].Y * (float)rearCaptureHeight);
            imagePoints[2] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[226].X * (float)rearCaptureWidth, landmarks.Landmark[226].Y * (float)rearCaptureHeight);
            imagePoints[3] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[446].X * (float)rearCaptureWidth, landmarks.Landmark[446].Y * (float)rearCaptureHeight);
            imagePoints[4] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[57].X * (float)rearCaptureWidth, landmarks.Landmark[57].Y * (float)rearCaptureHeight);
            imagePoints[5] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[287].X * (float)rearCaptureWidth, landmarks.Landmark[287].Y * (float)rearCaptureHeight);
            var image_points = new MatOfPoint2f(imagePoints);

            OpenCVForUnity.CoreModule.Point3[] objectPoints = new OpenCVForUnity.CoreModule.Point3[6];
             objectPoints[0] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[1].X * (float)rearCaptureWidth, landmarks.Landmark[1].Y * (float)rearCaptureHeight, landmarks.Landmark[1].Z * 2);
             objectPoints[1] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[152].X * (float)rearCaptureWidth, landmarks.Landmark[152].Y * (float)rearCaptureHeight, landmarks.Landmark[152].Z * 2);
             objectPoints[2] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[226].X * (float)rearCaptureWidth, landmarks.Landmark[226].Y * (float)rearCaptureHeight, landmarks.Landmark[226].Z * 2);
             objectPoints[3] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[446].X * (float)rearCaptureWidth, landmarks.Landmark[446].Y * (float)rearCaptureHeight, landmarks.Landmark[446].Z * 2);
             objectPoints[4] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[57].X * (float)rearCaptureWidth, landmarks.Landmark[57].Y * (float)rearCaptureHeight, landmarks.Landmark[57].Z * 2);
             objectPoints[5] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[287].X * (float)rearCaptureWidth, landmarks.Landmark[287].Y * (float)rearCaptureHeight, landmarks.Landmark[287].Z * 2);

            MatOfPoint3f object_points = new MatOfPoint3f(objectPoints);

            Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec);

            double tvec_x = tvec.get(0, 0)[0];
            double tvec_y = tvec.get(1, 0)[0];
            double tvec_z = tvec.get(2, 0)[0];

            if (double.IsNaN(tvec_z))
            { // if tvec is wrong data, do not use extrinsic guesses. (the estimated object is not in the camera field of view)
                Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec);
            }
            else
            {
                Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec, true, Calib3d.SOLVEPNP_ITERATIVE);
            }

            //Debug.Log(rvec.ToString());
            // Convert to unity pose data.
            //double[] rvecArr = new double[3];
            //rvec.get(0, 0, rvecArr);
            //double[] tvecArr = new double[3];
            //tvec.get(0, 0, tvecArr);
            //PoseData poseData = ARUtils.ConvertRvecTvecToPoseData(rvecArr, tvecArr);
            //Debug.Log(poseData.rot);
            //var outQuat = poseData.rot;

            Mat mtxR = new Mat(3, 3, CvType.CV_64FC1);
            Mat mtxQ = new Mat(3, 3, CvType.CV_64FC1);
            Calib3d.Rodrigues(rvec, rmat);

            var angles = Calib3d.RQDecomp3x3(rmat, mtxR, mtxQ);
            Debug.Log("X = " + angles[0] * 360);
            Debug.Log("Y = " + angles[1] * 360);
            //rearFaceCube.transform.localRotation = outQuat;

            rearFaceCube.transform.eulerAngles = new UnityEngine.Vector3((float)angles[0] * 360, (float)angles[1] * 360, 0);

            yield return null;
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
                }
            }

            isCapturing = result.IsOk;
        }

        private void StopVideoCapture()
        {
            if (isCapturing)
            {
                colorCamera.CaptureVideoStop();
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
            }
            else
            {
                StopVideoCapture();
            }
        }



        /// <summary>
        /// Handles the event of a new image getting captured.
        /// </summary>
        /// <param name="imageData">The raw data of the image.</param>
        private void OnCaptureRawVideoFrameAvailable(MLCamera.CameraOutput capturedFrame, MLCamera.ResultExtras resultExtras, MLCamera.Metadata metadataHandle)
        {
            MLCamera.FlipFrameVertically(ref capturedFrame);
            MLResult result = MLCVCamera.GetFramePose(resultExtras.VCamTimestamp, out UnityEngine.Matrix4x4 outMatrix);

            if (enableFrontDetection)
            {
                UpdateRGBTexture(ref rawVideoTexturesRGBA, capturedFrame.Planes[0]);              
            }
            else
            {

            }
            var landmarks = GameObject.Find("Solution").GetComponent<IrisTrackingSolution>().currentFaceAndIrisLandmarks;

            if (landmarks != null)
            {
                // top of the head
                var rightEyeX = (landmarks.Landmark[469].X + landmarks.Landmark[470].X + landmarks.Landmark[471].X + landmarks.Landmark[472].X) / 4;
                var rightEyeY = (landmarks.Landmark[469].Y + landmarks.Landmark[470].Y + landmarks.Landmark[471].Y + landmarks.Landmark[472].Y) / 4;
                var leftEyeX = (landmarks.Landmark[474].X + landmarks.Landmark[475].X + landmarks.Landmark[476].X + landmarks.Landmark[477].X) / 4;
                var leftEyeY = (landmarks.Landmark[474].Y + landmarks.Landmark[475].Y + landmarks.Landmark[476].Y + landmarks.Landmark[477].Y) / 4;
                //Debug.Log($"Right Eye Image Coordinates: {leftEyeX},{leftEyeY}");
                //Debug.Log($"Left Eye Image Coordinates: {rightEyeX},{rightEyeY}");

                //float estimatedFaceDepth = averagePixelsForFaceAt1Meter / face.Item2;

                var CenterLandmarkPosition = CastRayFromPixelToWorldPoint(800, 600,
                    new UnityEngine.Vector2(landmarks.Landmark[1].X * 800, landmarks.Landmark[1].Y * 600), resultExtras.Intrinsics.Value, 75.0f,
                    outMatrix, 1, false);

                var newObject = Instantiate(tempGO, CenterLandmarkPosition, UnityEngine.Quaternion.identity);

                Mat rvec = new Mat(3, 1, CvType.CV_64FC1);
                Mat tvec = new Mat(3, 1, CvType.CV_64FC1);
                Mat rmat = new Mat(3, 3, CvType.CV_64FC1);

                OpenCVForUnity.CoreModule.Point[] imagePoints = new OpenCVForUnity.CoreModule.Point[6];
                imagePoints[0] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[1].X * (float)rearCaptureWidth, landmarks.Landmark[1].Y * (float)rearCaptureHeight);
                imagePoints[1] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[152].X * (float)rearCaptureWidth, landmarks.Landmark[152].Y * (float)rearCaptureHeight);
                imagePoints[2] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[226].X * (float)rearCaptureWidth, landmarks.Landmark[226].Y * (float)rearCaptureHeight);
                imagePoints[3] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[446].X * (float)rearCaptureWidth, landmarks.Landmark[446].Y * (float)rearCaptureHeight);
                imagePoints[4] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[57].X * (float)rearCaptureWidth, landmarks.Landmark[57].Y * (float)rearCaptureHeight);
                imagePoints[5] = new OpenCVForUnity.CoreModule.Point(landmarks.Landmark[287].X * (float)rearCaptureWidth, landmarks.Landmark[287].Y * (float)rearCaptureHeight);

                var image_points = new MatOfPoint2f(imagePoints);

                OpenCVForUnity.CoreModule.Point3[] objectPoints = new OpenCVForUnity.CoreModule.Point3[6];
                objectPoints[0] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[1].X * (float)rearCaptureWidth, landmarks.Landmark[1].Y * (float)rearCaptureHeight, landmarks.Landmark[1].Z * 5);
                objectPoints[1] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[152].X * (float)rearCaptureWidth, landmarks.Landmark[152].Y * (float)rearCaptureHeight, landmarks.Landmark[152].Z * 5);
                objectPoints[2] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[226].X * (float)rearCaptureWidth, landmarks.Landmark[226].Y * (float)rearCaptureHeight, landmarks.Landmark[226].Z * 5);
                objectPoints[3] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[446].X * (float)rearCaptureWidth, landmarks.Landmark[446].Y * (float)rearCaptureHeight, landmarks.Landmark[446].Z * 5);
                objectPoints[4] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[57].X * (float)rearCaptureWidth, landmarks.Landmark[57].Y * (float)rearCaptureHeight, landmarks.Landmark[57].Z * 5);
                objectPoints[5] = new OpenCVForUnity.CoreModule.Point3(landmarks.Landmark[287].X * (float)rearCaptureWidth, landmarks.Landmark[287].Y * (float)rearCaptureHeight, landmarks.Landmark[287].Z * 5);

                MatOfPoint3f object_points = new MatOfPoint3f(objectPoints);

                Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec);

                double tvec_x = tvec.get(0, 0)[0];
                double tvec_y = tvec.get(1, 0)[0];
                double tvec_z = tvec.get(2, 0)[0];

                if (double.IsNaN(tvec_z))
                { // if tvec is wrong data, do not use extrinsic guesses. (the estimated object is not in the camera field of view)
                    Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec);
                }
                else
                {
                    Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec, true, Calib3d.SOLVEPNP_ITERATIVE);
                }

                Mat mtxR = new Mat(3, 3, CvType.CV_64FC1);
                Mat mtxQ = new Mat(3, 3, CvType.CV_64FC1);
                Calib3d.Rodrigues(rvec, rmat);

                var angles = Calib3d.RQDecomp3x3(rmat, mtxR, mtxQ);
                Debug.Log("X = " + angles[0] * 360);
                Debug.Log("Y = " + angles[1] * 360);


                //match the user's head pose to compensate for user position
                newObject.transform.rotation = Camera.main.transform.rotation;
                //turn temporary GO so that z axis faces user
                newObject.transform.Rotate(0, 180, 0, Space.Self);
                //rotate temp GO to match detected bystander head pose
                newObject.transform.Rotate((float)angles[0] * 360f * -1f, (float)angles[1] * 360f * -1f, 0, Space.Self);
                //rotate temp GO to rear to compansate for rear-facing camera FOV
                newObject.transform.RotateAround(outMatrix.GetPosition(), Camera.main.transform.up, 180);

                rearFaceCube.transform.position = newObject.transform.position;
                rearFaceCube.transform.rotation = newObject.transform.rotation;



                Ray bystanderGazeRay = new Ray(rearFaceCube.transform.position, rearFaceCube.transform.forward);
                eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);
                
                GameObject.Destroy(newObject);

            }




        }

        public UnityEngine.Vector3 CastRayFromPixelToWorldPoint(int width, int height, UnityEngine.Vector2 pixelPosition, MLCameraBase.IntrinsicCalibrationParameters parameters, float FOV, UnityEngine.Matrix4x4 cameraTransformationMatrix, float depth, bool frontCamera = true)
        {
            UnityEngine.Vector2 normalizedImagePoint;
            // Step 1: Normalize the image coordinates
            // If we dont have the camera meta data, use an estimated principal point
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

            UnityEngine.Vector3 cameraPoint = new UnityEngine.Vector3(normalizedImagePoint.x, normalizedImagePoint.y, 1);
            // Step 2: Convert normalized image coordinates to camera coordinates
            //Vector3 cameraPoint = new Vector3(normalizedImagePoint.x, normalizedImagePoint.y, 1);

            // Step 3: Create a 3D ray from camera position in the direction of the camera coordinate
            Ray cameraRay = new Ray(UnityEngine.Vector3.zero, cameraPoint);

            // Step 4: Convert the ray to world coordinates
            UnityEngine.Quaternion rotation = cameraTransformationMatrix.rotation;
            UnityEngine.Vector3 position = cameraTransformationMatrix.MultiplyPoint(cameraRay.origin);
            UnityEngine.Vector3 direction = rotation * cameraRay.direction;
            Ray worldRay = new Ray(position, direction);

            // Return the point in world space at the specified depth along the ray
            return worldRay.GetPoint(depth);
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
            localScale = new UnityEngine.Vector3(currentAspectRatio * localScale.y, localScale.y, 1);
            renderer.transform.localScale = localScale;
        }
    }
}

