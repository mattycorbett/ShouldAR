using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using UnityEngine.XR.MagicLeap;

namespace Mediapipe.Unity
{
    public class BystanderFaceLocalizer : MonoBehaviour
    {

        [TooltipAttribute("Facial Detection Cube Prefab")]
        public GameObject faceCube;

        [TooltipAttribute("Rear Camera Facial Detection Cube Prefab")]
        public GameObject rearFaceCube;

        [TooltipAttribute("Bystander Eye Gaze Debug Prefab")]
        public GameObject eyeGazeSphere;

        [TooltipAttribute("Temp GameObject")]
        public GameObject tempGO;

        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }

        public void LocalizeFacesFromDetection(NormalizedLandmarkList landmarks)
        {
            var averagePixelsForFaceAt1Meter = 300;

            if (landmarks != null)
            {
                // top of the head
                var rightEyeX = (landmarks.Landmark[469].X + landmarks.Landmark[470].X + landmarks.Landmark[471].X + landmarks.Landmark[472].X) / 4;
                var rightEyeY = (landmarks.Landmark[469].Y + landmarks.Landmark[470].Y + landmarks.Landmark[471].Y + landmarks.Landmark[472].Y) / 4;
                var leftEyeX = (landmarks.Landmark[474].X + landmarks.Landmark[475].X + landmarks.Landmark[476].X + landmarks.Landmark[477].X) / 4;
                var leftEyeY = (landmarks.Landmark[474].Y + landmarks.Landmark[475].Y + landmarks.Landmark[476].Y + landmarks.Landmark[477].Y) / 4;
                Debug.Log($"Right Eye Image Coordinates: {leftEyeX},{leftEyeY}");
                Debug.Log($"Left Eye Image Coordinates: {rightEyeX},{rightEyeY}");

            }


            /*foreach ((Vector2, float) face in faces)
            {

                float estimatedFaceDepth = averagePixelsForFaceAt1Meter / face.Item2;

                var CenterLandmarkPosition = CastRayFromPixelToWorldPoint(newImage.width, newImage.height,
                    face.Item1, intrinsics, 75.0f, outMatrix, 1, false);

                var newObject = Instantiate(tempGO, CenterLandmarkPosition, Quaternion.identity);


                //float gaze_YAngle = Vector2.SignedAngle(new Vector2((float)(face.Item3[0].x + face.Item3[1].x) / 2.0f, (float)(face.Item3[0].y + face.Item3[1].y) / 2.0f), face.Item1);
                //float gaze_XAngle = Vector2.SignedAngle(new Vector2((float)face.Item3[2].x, (float)face.Item3[2].y), face.Item1);
                //Debug.Log("YAngle = " + gaze_YAngle);
                //Debug.Log("XAngle = " + gaze_XAngle);

                //newObject.transform.LookAt(Camera.main.transform);
                newObject.transform.Rotate(0, 180, 0, Space.Self);
                newObject.transform.rotation = Camera.main.transform.rotation;
                newObject.transform.Rotate(0, 180, 0, Space.Self);
                newObject.transform.RotateAround(outMatrix.GetPosition(), Camera.main.transform.up, 180);
                //newObject.transform.Rotate(gaze_XAngle, gaze_YAngle, 0, Space.Self);


                rearFaceCube.transform.position = newObject.transform.position;
                rearFaceCube.transform.rotation = newObject.transform.rotation;

                Ray bystanderGazeRay = new Ray(rearFaceCube.transform.position, rearFaceCube.transform.forward);
                eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);

                GameObject.Destroy(newImage);
                GameObject.Destroy(newObject);
            }*/
        }
        public Vector3 CastRayFromPixelToWorldPoint(int width, int height, Vector2 pixelPosition, MLCameraBase.IntrinsicCalibrationParameters parameters, float FOV, Matrix4x4 cameraTransformationMatrix, float depth, bool frontCamera = true)
        {
            Vector2 normalizedImagePoint;
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
    }
}

