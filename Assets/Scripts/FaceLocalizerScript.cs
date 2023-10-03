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
    public class FaceLocalizerScript : MonoBehaviour
    {

        [TooltipAttribute("Facial Detection Cube Prefab")]
        public GameObject faceCube;

        [TooltipAttribute("Rear Camera Facial Detection Cube Prefab")]
        public GameObject rearFaceCube;

        [TooltipAttribute("Bystander Eye Gaze Debug Prefab")]
        public GameObject eyeGazeSphere;

        [TooltipAttribute("Temp GameObject")]
        public GameObject tempGO;

        public CameraCapture cameraCapture;


        // Start is called before the first frame update
        void Start()
        {
            cameraCapture = GameObject.Find("CameraCapture").GetComponent<CameraCapture>();
            
        }

        // Update is called once per frame
        void Update()
        {

        }

        public void LocalizeFacesFromDetection(NormalizedLandmarkList landmarks)
        {
            var averagePixelsForFaceAt1Meter = 300;

           /* if (landmarks != null)
            {
                // top of the head
                var rightEyeX = (landmarks.Landmark[469].X + landmarks.Landmark[470].X + landmarks.Landmark[471].X + landmarks.Landmark[472].X) / 4;
                var rightEyeY = (landmarks.Landmark[469].Y + landmarks.Landmark[470].Y + landmarks.Landmark[471].Y + landmarks.Landmark[472].Y) / 4;
                var leftEyeX = (landmarks.Landmark[474].X + landmarks.Landmark[475].X + landmarks.Landmark[476].X + landmarks.Landmark[477].X) / 4;
                var leftEyeY = (landmarks.Landmark[474].Y + landmarks.Landmark[475].Y + landmarks.Landmark[476].Y + landmarks.Landmark[477].Y) / 4;
                Debug.Log($"Right Eye Image Coordinates: {leftEyeX},{leftEyeY}");
                Debug.Log($"Left Eye Image Coordinates: {rightEyeX},{rightEyeY}");

                //float estimatedFaceDepth = averagePixelsForFaceAt1Meter / face.Item2;

                var CenterLandmarkPosition = CastRayFromPixelToWorldPoint(800, 600,
                    new Vector2(landmarks.Landmark[1].X * 800, landmarks.Landmark[1].Y * 600), cameraCapture.intrinsicCalibParam, 75.0f,
                    cameraCapture.transformMatrix, 1, false);

                /*var newObject = Instantiate(tempGO, CenterLandmarkPosition, Quaternion.identity);

                newObject.transform.Rotate(0, 180, 0, Space.Self);
                newObject.transform.rotation = Camera.main.transform.rotation;
                newObject.transform.Rotate(0, 180, 0, Space.Self);
                newObject.transform.RotateAround(GameObject.Find("CameraCapture").GetComponent<CameraCapture>().transformMatrix.GetPosition(), Camera.main.transform.up, 180);
                //newObject.transform.Rotate(gaze_XAngle, gaze_YAngle, 0, Space.Self);


                rearFaceCube.transform.position = newObject.transform.position;
                rearFaceCube.transform.rotation = newObject.transform.rotation;

                Ray bystanderGazeRay = new Ray(rearFaceCube.transform.position, rearFaceCube.transform.forward);
                eyeGazeSphere.transform.position = bystanderGazeRay.GetPoint(3);

                GameObject.Destroy(newObject);


            }*/

        }

    }
}

