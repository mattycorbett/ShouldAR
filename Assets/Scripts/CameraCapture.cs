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
using TMPro;


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

        [TooltipAttribute("User Eye Fixation threshold (in ms)")]
        public float fixationDurationThreshold;

        [TooltipAttribute("Eye Tracking Prefab")]
        public GameObject eyeTrackingObject;

        [TooltipAttribute("SSA Attack Chevron (under Main Camera)")]
        public GameObject attackIndicatorObject;

        [SerializeField, Tooltip("Enable ML2 Front Camera?")]
        private bool enableFrontDetection = false;

        [SerializeField, Tooltip("Desired width for the front camera capture")]
        private int frontCaptureWidth = 1280;

        [SerializeField, Tooltip("Desired height for the front camera capture")]
        private int frontCaptureHeight = 720;

        [SerializeField, Tooltip("Width (in pixels) for the rear camera capture")]
        private int rearCaptureWidth = 1280;

        [SerializeField, Tooltip("Height (in pixels) for the rear camera capture")]
        private int rearCaptureHeight = 720;

        [SerializeField, Tooltip("Rear camera FOV (in degrees)")]
        private int rearCaptureFOV = 55;

        [TooltipAttribute("Use only Head Pose for Bystander Gaze Estimation?")]
        public bool useOnlyHeadPose;

        [TooltipAttribute("Debug Image Quad")]
        public GameObject imageQuad;


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

        EyeTracking eyeTrackingScript;

        IrisTrackingSolution mediapipeSolution;

        Renderer quadRend;

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
            camMatrix.put(0, 0, 1225);
            camMatrix.put(0, 1, 0);
            camMatrix.put(0, 2, 640);
            camMatrix.put(1, 0, 0);
            camMatrix.put(1, 1, 1225);
            camMatrix.put(1, 2, 360);
            camMatrix.put(2, 0, 0);
            camMatrix.put(2, 1, 0);
            camMatrix.put(2, 2, 1.0f);
            Debug.Log("camMatrix " + camMatrix.dump());
   
            distCoeffs = new MatOfDouble(-0.05137476,  0.32424092,  0.00676531, -0.01408409, -0.25836542);

            float imageSizeScale = (float)rearCaptureHeight / (float)rearCaptureWidth;

            //Size imageSize = new Size(rearCaptureWidth * imageSizeScale, rearCaptureHeight * imageSizeScale);
            Size imageSize = new Size(rearCaptureWidth, rearCaptureHeight);
            double apertureWidth = 1;
            double apertureHeight = 1;
            double[] fovx = new double[1];
            double[] fovy = new double[1];
            double[] focalLength = new double[1];
            Point principalPoint = new Point(0, 0);
            double[] aspectratio = new double[1];

            Calib3d.calibrationMatrixValues(camMatrix, imageSize, apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectratio);

            StartCoroutine(FramerateCountLoop());

            eyeTrackingScript = eyeTrackingObject.GetComponent<EyeTracking>();

            mediapipeSolution = GameObject.Find("MediapipeSolution").GetComponent<IrisTrackingSolution>();

            quadRend = imageQuad.GetComponent<Renderer>();

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

            //StartVideoCapture();
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
            var FaceAndIrisLandmarks = mediapipeSolution.currentFaceAndIrisLandmarks;
            var FaceLandmarks = mediapipeSolution.currentFaceLandmarks;

            if (FaceAndIrisLandmarks != null && !useOnlyHeadPose)
            {
                var angles = EstimateHeadPoseFromFaceAndIrisLandmarks(FaceAndIrisLandmarks);
                var rearFace = Instantiate(rearFaceCube, new UnityEngine.Vector3(0, 0, -2), UnityEngine.Quaternion.identity);
                rearFace.transform.eulerAngles = new UnityEngine.Vector3((float)angles[0], (float)angles[1], (float)angles[2]);

                RaycastHit hitData;
                Ray bystanderGazeRay = new Ray(rearFace.transform.position, rearFace.transform.forward);
                var hitSuccess = Physics.Raycast(bystanderGazeRay, out hitData);
                eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);
                if (hitSuccess)
                {
                //Debug.Log(hitData.collider.tag);
                    if (hitData.collider.tag == "EyeGaze")
                    {
                        attackIndicatorObject.GetComponent<ChevronScript>().bystanderGazeHits += 1;
                        attackIndicatorObject.GetComponent<ChevronScript>().lastHitPosition = rearFace.transform;
                    }

                }
                Destroy(rearFace);
            }

            if (FaceLandmarks != null && (useOnlyHeadPose || FaceAndIrisLandmarks == null))
            {

                var angles = EstimateHeadPoseFromFaceLandmarks(FaceLandmarks[0]);
                var rearFace = Instantiate(rearFaceCube, new UnityEngine.Vector3(0, 0, -2), UnityEngine.Quaternion.identity);
                rearFace.transform.eulerAngles = new UnityEngine.Vector3((float)angles[0], (float)angles[1], (float)angles[2]);

                RaycastHit hitData;
                Ray bystanderGazeRay = new Ray(rearFace.transform.position, rearFace.transform.forward);
                var hitSuccess = Physics.Raycast(bystanderGazeRay, out hitData);
                eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);

                if (hitSuccess)
                {
                Debug.Log(hitData.collider.tag);
                    if (hitData.collider.tag == "EyeGaze")
                    {
                    
                        attackIndicatorObject.GetComponent<ChevronScript>().bystanderGazeHits += 1;
                        attackIndicatorObject.GetComponent<ChevronScript>().lastHitPosition = rearFace.transform;
                    }

                }
                Destroy(rearFace);

            }
#endif
        }




        double[] EstimateHeadPoseFromFaceAndIrisLandmarks(NormalizedLandmarkList FaceAndIrisLandmarks)
            {


                Mat rvec = new Mat(3, 1, CvType.CV_64FC1);
                Mat tvec = new Mat(3, 1, CvType.CV_64FC1);
                Mat rmat = new Mat(3, 3, CvType.CV_64FC1);

                OpenCVForUnity.CoreModule.Point[] imagePoints = new OpenCVForUnity.CoreModule.Point[6];
                imagePoints[0] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[1].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[1].Y * (float)rearCaptureHeight));//tip of nose
                imagePoints[1] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[33].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[33].Y * (float)rearCaptureHeight));//left outside eyelid
                imagePoints[2] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[263].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[263].Y * (float)rearCaptureHeight));//right outside eyelid
                imagePoints[3] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[61].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[61].Y * (float)rearCaptureHeight));//left outside mouth
                imagePoints[4] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[291].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[291].Y * (float)rearCaptureHeight));//right outside mouth
                imagePoints[5] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[199].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[199].Y * (float)rearCaptureHeight));//chin
                var image_points = new MatOfPoint2f(imagePoints);

                //Debug.Log("Image Points" + image_points.dump());

                OpenCVForUnity.CoreModule.Point3[] objectPoints = new OpenCVForUnity.CoreModule.Point3[6];
                objectPoints[0] = new OpenCVForUnity.CoreModule.Point3(0, 0, -7.475604);//tip of nose
                objectPoints[1] = new OpenCVForUnity.CoreModule.Point3(-4.445859, -3.790856, -3.173422);//left outside eyelid
                objectPoints[2] = new OpenCVForUnity.CoreModule.Point3(4.445859, -3.790856, -3.173422);//right outside eyelid
                objectPoints[3] = new OpenCVForUnity.CoreModule.Point3(-2.456206, 3.215756, -4.283884);//left outside mouth
                objectPoints[4] = new OpenCVForUnity.CoreModule.Point3(2.456206, 3.215756, -4.283884);//right outside mouth
                objectPoints[5] = new OpenCVForUnity.CoreModule.Point3(0, 8.276513, -4.264492); //chin

                var object_points = new MatOfPoint3f(objectPoints);

                Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec);

                double tvec_x = tvec.get(0, 0)[0];
                double tvec_y = tvec.get(1, 0)[0];
                double tvec_z = tvec.get(2, 0)[0];

                if (!double.IsNaN(tvec_z))
                { // if tvec is wrong data, do not use extrinsic guesses. (the estimated object is not in the camera field of view)
                    Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec, true, Calib3d.SOLVEPNP_ITERATIVE);
                }

                Mat mtxR = new Mat(3, 3, CvType.CV_64FC1);
                Mat mtxQ = new Mat(3, 3, CvType.CV_64FC1);

                Calib3d.Rodrigues(rvec, rmat);
                // Debug.Log("RMAT" + rmat.dump());
                var angles = Calib3d.RQDecomp3x3(rmat, mtxR, mtxQ);


                //https://storage.googleapis.com/mediapipe-assets/documentation/mediapipe_face_landmark_fullsize.png
                UnityEngine.Vector3 eyelid_159_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[159].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[159].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[159].Z * 5));
                UnityEngine.Vector3 eyelid_158_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[158].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[158].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[158].Z * 5));
                UnityEngine.Vector3 eyelid_157_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[157].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[157].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[157].Z * 5));
                UnityEngine.Vector3 eyelid_173_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[173].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[173].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[173].Z * 5));
                UnityEngine.Vector3 eyelid_133_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[133].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[133].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[133].Z * 5));
                UnityEngine.Vector3 eyelid_155_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[155].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[155].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[155].Z * 5));
                UnityEngine.Vector3 eyelid_154_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[154].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[154].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[154].Z * 5));
                UnityEngine.Vector3 eyelid_153_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[153].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[153].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[153].Z * 5));
                UnityEngine.Vector3 eyelid_145_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[145].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[145].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[145].Z * 5));
                UnityEngine.Vector3 eyelid_144_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[144].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[144].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[144].Z * 5));
                UnityEngine.Vector3 eyelid_163_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[163].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[163].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[163].Z * 5));
                UnityEngine.Vector3 eyelid_7_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[7].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[7].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[7].Z * 5));
                UnityEngine.Vector3 eyelid_33_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[33].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[33].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[33].Z * 5));
                UnityEngine.Vector3 eyelid_246_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[246].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[246].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[246].Z * 5));
                UnityEngine.Vector3 eyelid_161_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[161].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[161].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[161].Z * 5));
                UnityEngine.Vector3 eyelid_160_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[160].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[160].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[160].Z * 5));

                UnityEngine.Vector3 eyelid_center_unity_coords = (eyelid_33_unity_coords + eyelid_133_unity_coords) / 2;
                UnityEngine.Vector3 eyelid_left_inside_unity_coords = (eyelid_157_unity_coords + eyelid_154_unity_coords) / 2;
                UnityEngine.Vector3 eyelid_left_outside_unity_coords = (eyelid_161_unity_coords + eyelid_163_unity_coords) / 2;
                UnityEngine.Vector3 eyelid_left_up_unity_coords = eyelid_159_unity_coords;
                UnityEngine.Vector3 eyelid_left_down_unity_coords = eyelid_145_unity_coords;

                UnityEngine.Vector3 iris_left_point_unity_coords = new UnityEngine.Vector3((float)rearCaptureWidth - (FaceAndIrisLandmarks.Landmark[468].X * (float)rearCaptureWidth), (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[468].Y * (float)rearCaptureHeight), (FaceAndIrisLandmarks.Landmark[468].Z * 5));

                var radius = UnityEngine.Vector3.Distance(eyelid_33_unity_coords, eyelid_133_unity_coords) / 2;

                List<double> distanceList = new List<double>();
                //Add distances to List in order to efficiently poll which index is lower
                distanceList.Add(UnityEngine.Vector3.Distance(iris_left_point_unity_coords, eyelid_center_unity_coords));
                //distanceList.Add(UnityEngine.Vector3.Distance(iris_left_point_unity_coords, eyelid_left_up_unity_coords));
                distanceList.Add(UnityEngine.Vector3.Distance(iris_left_point_unity_coords, eyelid_left_inside_unity_coords));
                distanceList.Add(UnityEngine.Vector3.Distance(iris_left_point_unity_coords, eyelid_left_outside_unity_coords));
                //distanceList.Add(UnityEngine.Vector3.Distance(iris_left_point_unity_coords, eyelid_left_down_unity_coords));


                double minValue = distanceList.Min();
                int minIndex = distanceList.IndexOf(minValue);
            //Debug.Log(minIndex);

            //var testMat = get_3d_realigned_landmarks_pos(FaceAndIrisLandmarks, angles);

            if (minIndex != 0)
                {
                    //if eyes are directed left (relative to face)
                    if (minIndex == 1)
                    {
                        angles[1] = angles[1] - 30;
                        //Debug.Log("Compensated Angles: " + angles[0] + "," + angles[1] + "," + angles[2]);
                    }
                    //if eyes are directed right (relative to face)
                    else if (minIndex == 2)
                    {
                        angles[1] = angles[1] + 30;
                        //Debug.Log("Compensated Angles: " + angles[0] + "," + angles[1] + "," + angles[2]);
                    }
                }


                //Mat testPoint = new Mat(3, 1, CvType.CV_64FC1);
                //testPoint.put(0, 0, testMat.get(0, 0)[0], testMat.get(0, 0)[1], testMat.get(0, 0)[2]);      
                //Debug.Log(testPoint.dump());

            return angles;
            }

            OpenCVForUnity.CoreModule.Mat get_3d_realigned_landmarks_pos(NormalizedLandmarkList LandmarksList, double[] angles)
        {
            Mat XRot = new Mat(3, 3, CvType.CV_64FC1);
            Mat YRot = new Mat(3, 3, CvType.CV_64FC1);
            Mat ZRot = new Mat(3, 3, CvType.CV_64FC1);


            //XRot.put(0, 0, 1f, 0f, 0f, 0f, Math.Cos((float)angles[0] * Mathf.Deg2Rad), -Math.Sin((float)angles[0] * Mathf.Deg2Rad), 0f, Math.Sin((float)angles[0] * Mathf.Deg2Rad), Math.Cos((float)angles[0] * Mathf.Deg2Rad));
            //YRot.put(0, 0, Math.Cos((float)angles[1] * Mathf.Deg2Rad), 0f, Math.Sin((float)angles[1] * Mathf.Deg2Rad), 0f, 1f, 0f, -Math.Sin((float)angles[1] * Mathf.Deg2Rad), 0f, Math.Cos((float)angles[1] * Mathf.Deg2Rad));
            //ZRot.put(0, 0, Math.Cos((float)angles[2] * Mathf.Deg2Rad), -Math.Sin((float)angles[2] * Mathf.Deg2Rad), 0, Math.Sin((float)angles[2] * Mathf.Deg2Rad), Math.Cos((float)angles[2] * Mathf.Deg2Rad), 0f, 0f, 0f, 1f);

            //Left handed
            XRot.put(0, 0, 1f, 0f, 0f, 0f, Math.Cos((float)angles[0] * Mathf.Deg2Rad), Math.Sin((float)angles[0] * Mathf.Deg2Rad), 0f, -Math.Sin((float)angles[0] * Mathf.Deg2Rad), Math.Cos((float)angles[0] * Mathf.Deg2Rad));
            YRot.put(0, 0, Math.Cos((float)angles[1] * Mathf.Deg2Rad), 0f, -Math.Sin((float)angles[1] * Mathf.Deg2Rad), 0f, 1f, 0f, Math.Sin((float)angles[1] * Mathf.Deg2Rad), 0f, Math.Cos((float)angles[1] * Mathf.Deg2Rad));
            ZRot.put(0, 0, Math.Cos((float)angles[2] * Mathf.Deg2Rad), Math.Sin((float)angles[2] * Mathf.Deg2Rad), 0, -Math.Sin((float)angles[2] * Mathf.Deg2Rad), Math.Cos((float)angles[2] * Mathf.Deg2Rad), 0f, 0f, 0f, 1f);
            var invRot = ZRot * YRot * XRot;

            int i = 0;
            OpenCVForUnity.CoreModule.Point3[] landmarkpoints = new OpenCVForUnity.CoreModule.Point3[478];
            OpenCVForUnity.CoreModule.Point3 center = new OpenCVForUnity.CoreModule.Point3(LandmarksList.Landmark[1].X * (float)rearCaptureWidth, LandmarksList.Landmark[1].Y * (float)rearCaptureHeight, LandmarksList.Landmark[1].Z);
            foreach (NormalizedLandmark landmark in LandmarksList.Landmark)
            {
                landmarkpoints[i] = new OpenCVForUnity.CoreModule.Point3((landmark.X * (float)rearCaptureWidth), (landmark.Y * (float)rearCaptureHeight), (landmark.Z));
                landmarkpoints[i] = landmarkpoints[i] - center;
                Mat testPoint = new Mat(3, 1, CvType.CV_64FC1);
                testPoint.put(0, 0, landmarkpoints[i].x, landmarkpoints[i].y, landmarkpoints[i].z);
                var outPoint = (invRot.t() * testPoint);
                landmarkpoints[i] = new OpenCVForUnity.CoreModule.Point3(outPoint.get(0, 0)[0], outPoint.get(1, 0)[0], outPoint.get(2, 0)[0]);
                landmarkpoints[i] = landmarkpoints[i] + center;

                i += 1;
            }
            MatOfPoint3f MatLandmarks = new MatOfPoint3f(landmarkpoints);

            return MatLandmarks;
        }


        double[] EstimateHeadPoseFromFaceLandmarks(NormalizedLandmarkList FaceAndIrisLandmarks)
            {

                Mat rvec = new Mat(3, 1, CvType.CV_64FC1);
                Mat tvec = new Mat(3, 1, CvType.CV_64FC1);
                Mat rmat = new Mat(3, 3, CvType.CV_64FC1);


                OpenCVForUnity.CoreModule.Point[] imagePoints = new OpenCVForUnity.CoreModule.Point[6];
                imagePoints[0] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[1].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[1].Y * (float)rearCaptureHeight));//tip of nose
                imagePoints[1] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[33].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[33].Y * (float)rearCaptureHeight));//left outside eyelid
                imagePoints[2] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[263].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[263].Y * (float)rearCaptureHeight));//right outside eyelid
                imagePoints[3] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[61].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[61].Y * (float)rearCaptureHeight));//left outside mouth
                imagePoints[4] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[291].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[291].Y * (float)rearCaptureHeight));//right outside mouth
                imagePoints[5] = new OpenCVForUnity.CoreModule.Point((FaceAndIrisLandmarks.Landmark[199].X * (float)rearCaptureWidth), (FaceAndIrisLandmarks.Landmark[199].Y * (float)rearCaptureHeight));//chin
                var image_points = new MatOfPoint2f(imagePoints);

                OpenCVForUnity.CoreModule.Point3[] objectPoints = new OpenCVForUnity.CoreModule.Point3[6];
                objectPoints[0] = new OpenCVForUnity.CoreModule.Point3(0, 0, -7.475604);//tip of nose
                objectPoints[1] = new OpenCVForUnity.CoreModule.Point3(-4.445859, -3.790856, -3.173422);//left outside eyelid
                objectPoints[2] = new OpenCVForUnity.CoreModule.Point3(4.445859, -3.790856, -3.173422);//right outside eyelid
                objectPoints[3] = new OpenCVForUnity.CoreModule.Point3(-2.456206, 3.215756, -4.283884);//left outside mouth
                objectPoints[4] = new OpenCVForUnity.CoreModule.Point3(2.456206, 3.215756, -4.283884);//right outside mouth
                objectPoints[5] = new OpenCVForUnity.CoreModule.Point3(0, 8.276513, -4.264492); //chin

                MatOfPoint3f object_points = new MatOfPoint3f(objectPoints);

                Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec);

                double tvec_x = tvec.get(0, 0)[0];
                double tvec_y = tvec.get(1, 0)[0];
                double tvec_z = tvec.get(2, 0)[0];

                if (!double.IsNaN(tvec_z))
                { // if tvec is wrong data, do not use extrinsic guesses. (the estimated object is not in the camera field of view)
                    Calib3d.solvePnP(object_points, image_points, camMatrix, distCoeffs, rvec, tvec, true, Calib3d.SOLVEPNP_ITERATIVE);
                }


                Mat mtxR = new Mat(3, 3, CvType.CV_64FC1);
                Mat mtxQ = new Mat(3, 3, CvType.CV_64FC1);
                Calib3d.Rodrigues(rvec, rmat);

                var angles = Calib3d.RQDecomp3x3(rmat, mtxR, mtxQ);

                /*if ((float)angles[0] < 0)
                {
                    angles[0] = -180f - (float)angles[0];
                }
                else
                {
                    angles[0] = 180f - (float)angles[0];
                }*/

                return angles;
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
                //turn off debug image quad 
                quadRend.enabled = false;
            }
            else
            {
                StopVideoCapture();
                //turn on debug image quad 
                quadRend.enabled = true;
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
            var FaceAndIrisLandmarks = mediapipeSolution.currentFaceAndIrisLandmarks;
            var FaceLandmarks = mediapipeSolution.currentFaceLandmarks;
            var faceDetection = mediapipeSolution.currentFaceList;
            //if full landmarks are avalable, we have access to iris and a 3D face model, we use this more-accurate result
            if (FaceAndIrisLandmarks != null && !useOnlyHeadPose)
            {

                var angles = EstimateHeadPoseFromFaceAndIrisLandmarks(FaceAndIrisLandmarks);

                var irisLeftMinX = Mathf.Min(FaceAndIrisLandmarks.Landmark[468].X, FaceAndIrisLandmarks.Landmark[469].X, FaceAndIrisLandmarks.Landmark[470].X, FaceAndIrisLandmarks.Landmark[471].X, FaceAndIrisLandmarks.Landmark[472].X);
                var irisLeftMaxX = Mathf.Max(FaceAndIrisLandmarks.Landmark[468].X, FaceAndIrisLandmarks.Landmark[469].X, FaceAndIrisLandmarks.Landmark[470].X, FaceAndIrisLandmarks.Landmark[471].X, FaceAndIrisLandmarks.Landmark[472].X);

                //estimate depth using MediaPipe iris width method
                var dx = (irisLeftMaxX * (float)rearCaptureWidth) - (irisLeftMinX * (float)rearCaptureWidth);
                var dX = 11.7;

                var normalizedFocaleX = 1.40625;
                var fx = Math.Min(rearCaptureWidth, rearCaptureHeight) * normalizedFocaleX;
                var dZ = (fx * (dX / dx)) / 1000.0;
                Debug.Log("Depth: " + dZ);

                var CenterLandmarkPosition = CastRayFromPixelToWorldPoint(rearCaptureWidth, rearCaptureHeight,
                    new UnityEngine.Vector2(FaceAndIrisLandmarks.Landmark[8].X * (float)rearCaptureWidth, (float)rearCaptureHeight - (FaceAndIrisLandmarks.Landmark[8].Y * (float)rearCaptureHeight)), resultExtras.Intrinsics.Value, rearCaptureFOV,
                    outMatrix, (float)dZ, false);

                var newObject = Instantiate(tempGO, CenterLandmarkPosition, UnityEngine.Quaternion.identity);

                //match the user's head pose to compensate for user position
                newObject.transform.rotation = Camera.main.transform.rotation;

                //rotate temp GO to match detected bystander head pose
                newObject.transform.Rotate((float)angles[0] * -1f, (float)angles[1], (float)angles[2], Space.Self);

                //rotate temp GO to rear to compensate for rear-facing camera FOV
                newObject.transform.RotateAround(outMatrix.GetPosition(), Camera.main.transform.up, 180);
                //turn temporary GO so that z axis faces user
                newObject.transform.Rotate(0, 180, 0, Space.Self);

                var rearFace = Instantiate(rearFaceCube, newObject.transform.position, newObject.transform.rotation);

                RaycastHit hitData;
                Ray bystanderGazeRay = new Ray(rearFace.transform.position, rearFace.transform.forward);
                var hitSuccess = Physics.Raycast(bystanderGazeRay, out hitData);
                eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);

                    if (hitSuccess)
                    {
                        if (hitData.collider.tag == "EyeGaze" && eyeTrackingScript.meanFixationDuration > fixationDurationThreshold)
                        {
                            attackIndicatorObject.GetComponent<ChevronScript>().bystanderGazeHits += 1;
                            attackIndicatorObject.GetComponent<ChevronScript>().lastHitPosition = rearFace.transform;
                        }

                    }

                    GameObject.Destroy(newObject);
                GameObject.Destroy(rearFace);
            }
            //if full landmarks are not avalable, we use a less accurate face detection and only estimate head pose
            
            if (FaceLandmarks != null && (useOnlyHeadPose || FaceAndIrisLandmarks == null))
            {

                float faceWidthAt1M = 25f;
                float faceWidth = (FaceLandmarks[0].Landmark[454].X * (float)rearCaptureWidth) - (FaceLandmarks[0].Landmark[324].X * (float)rearCaptureWidth);
                float depth = faceWidthAt1M / faceWidth;
                Debug.Log("Face depth estimated at " + depth + " meters");


                var angles = EstimateHeadPoseFromFaceLandmarks(FaceLandmarks[0]);
                var CenterLandmarkPosition = CastRayFromPixelToWorldPoint(rearCaptureWidth, rearCaptureHeight,
                    new UnityEngine.Vector2(FaceLandmarks[0].Landmark[8].X * (float)rearCaptureWidth, (float)rearCaptureHeight - (FaceLandmarks[0].Landmark[8].Y * (float)rearCaptureHeight)), resultExtras.Intrinsics.Value, rearCaptureFOV,
                    outMatrix, depth, false);

                var newObject = Instantiate(tempGO, CenterLandmarkPosition, UnityEngine.Quaternion.identity);

                //match the user's head pose to compensate for user position
                newObject.transform.rotation = Camera.main.transform.rotation;

                //rotate temp GO to match detected bystander head pose
                newObject.transform.Rotate((float)angles[0] * -1f, (float)angles[1], (float)angles[2], Space.Self);

                //rotate temp GO to rear to compensate for rear-facing camera FOV
                newObject.transform.RotateAround(outMatrix.GetPosition(), Camera.main.transform.up, 180);
                //turn temporary GO so that z axis faces user
                newObject.transform.Rotate(0, 180, 0, Space.Self);

                var rearFace = Instantiate(rearFaceCube, newObject.transform.position, newObject.transform.rotation);

                RaycastHit hitData;
                Ray bystanderGazeRay = new Ray(rearFace.transform.position, rearFace.transform.forward);
                var hitSuccess = Physics.Raycast(bystanderGazeRay, out hitData);
                eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);

                    if (hitSuccess)
                    {
                        if (hitData.collider.tag == "EyeGaze" && eyeTrackingScript.meanFixationDuration > fixationDurationThreshold)
                        {
                            attackIndicatorObject.GetComponent<ChevronScript>().bystanderGazeHits += 1;
                            attackIndicatorObject.GetComponent<ChevronScript>().lastHitPosition = rearFace.transform;
                        }

                    }

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
                normalizedImagePoint.x = (pixelPosition.x - (width / 2)) / width;
                normalizedImagePoint.y = (pixelPosition.y - (height / 2)) / height;
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

