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
using UnityEngine.UI;
using UnityEngine.Networking;



namespace Mediapipe.Unity
{


    /// <summary>
    /// This class handles video recording and loading based on controller
    /// input.
    /// </summary>
    public class WebCameraCapture : MonoBehaviour
    {
        
        public Texture2D rawVideoTexturesRGBA;

        [TooltipAttribute("Rear Camera Capture Display Quad")]
        public GameObject debugImageQuad;

        [TooltipAttribute("Time (in frames) between rear camera updates")]
        public int timelimit;
        private int timer;
        /// <summary>
        /// Using Awake so that Permissions is set before PermissionRequester Start.
        /// </summary>
        void Awake()
        {
            timer = 0;
            rawVideoTexturesRGBA = new Texture2D(800, 600);
        }

        /// <summary>
        /// Stop the camera, unregister callbacks, and stop input and permissions APIs.
        /// </summary>
        void OnDisable()
        {

        }

        /// <summary>
        /// Display permission error if necessary or update status text.
        /// </summary>
        private void Update()
        {
            timer += 1;
            if (timer > timelimit) {
                timer = 0;
                StartCoroutine(GetWebImage());
                debugImageQuad.GetComponent<Renderer>().material.mainTexture = rawVideoTexturesRGBA;
            }

        }

        IEnumerator GetWebImage()
        {
            using (UnityWebRequest webRequest = new UnityWebRequest("http://10.0.0.231/uploads/esp32-cam.jpg", UnityWebRequest.kHttpVerbGET))
            {
                webRequest.downloadHandler = new DownloadHandlerTexture();
                yield return webRequest.SendWebRequest();
                while (!webRequest.isDone)
                {
                    //Wait each frame in each loop OR Unity would freeze
                    yield return null;
                }

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
                        //Debug.Log("Success...");
                        rawVideoTexturesRGBA = DownloadHandlerTexture.GetContent(webRequest);
                        rawVideoTexturesRGBA.Apply();
                        break;
                }
            }
        }

    }
}

