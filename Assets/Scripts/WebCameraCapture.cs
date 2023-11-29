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
using System.Net.NetworkInformation;
using System.Net;
using System.IO;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Buffers.Binary;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Networking;
using UnityEngine.Experimental.Rendering;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.UtilsModule;
using OpenCVForUnity.UnityUtils.Helper;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.ImgcodecsModule;

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

        [TooltipAttribute("Show Captured Rear Camera Frame (for debugging)?")]
        public bool showCapturedImage;

        [TooltipAttribute("Server (this device) IP Address")]
        public string serverAddress;

        [TooltipAttribute("Is camera Fisheye? (Use fisheye_undistort)")]
        public bool undistort;

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
            rawVideoTexturesRGBA = new Texture2D(1280, 720, TextureFormat.BGRA32, false);

            UnityEngine.Color[] pixels = Enumerable.Repeat(UnityEngine.Color.white, 1280 * 720).ToArray();
            rawVideoTexturesRGBA.SetPixels(pixels);
            rawVideoTexturesRGBA.Apply();

            camMatrix = new Mat(3, 3, CvType.CV_64FC1);
            camMatrix.put(0, 0, 434.71738971070573);
            camMatrix.put(0, 1, 0);
            camMatrix.put(0, 2, 635.3948268126796);
            camMatrix.put(1, 0, 0);
            camMatrix.put(1, 1, 435.10553668263424);
            camMatrix.put(1, 2, 372.6527083640845);
            camMatrix.put(2, 0, 0);
            camMatrix.put(2, 1, 0);
            camMatrix.put(2, 2, 1.0f);

            //distCoeffs = new MatOfDouble(0, 0, 0, 0);
            distCoeffs = new MatOfDouble(0.29632513879582006, 1.8871756682399388, -4.1241686060131, 2.6896060690761225);

            GetSocketImage();


        }

        /// <summary>
        /// Stop the camera, unregister callbacks, and stop input and permissions APIs.
        /// </summary>
        void OnDisable()
        {

        }

        void OnDestroy()
        {

        }

        /// <summary>
        /// Display permission error if necessary or update status text.
        /// </summary>
        private void Update()
        {
            if (showCapturedImage)
            {
                debugImageQuad.GetComponent<Renderer>().material.mainTexture = rawVideoTexturesRGBA;
            }
            

        }

        public static bool IsListeningPortAvailable(int port) =>
                    !IPGlobalProperties.GetIPGlobalProperties().GetActiveTcpListeners().Any(x => x.Port == port);

        async void GetSocketImage()
        {

           /* while (!IsListeningPortAvailable(6464))
            {
                Debug.Log("Waiting for open socket....");
                
            }*/

            Service();

        }

        void UndistortImage(byte[] tempByteArray)
        {
            Mat buff = new Mat(1, tempByteArray.Length, CvType.CV_8UC1);
            Utils.copyToMat<byte>(tempByteArray, buff);
            
            Mat img_result = Imgcodecs.imdecode(buff, Imgcodecs.IMREAD_COLOR);
            Mat outMat = new Mat();

            Size size = new Size(img_result.width(), img_result.height());
            Mat E = Mat.eye(3, 3, CvType.CV_64FC1);
            Mat map1 = new Mat();
            Mat map2 = new Mat();
            Mat new_K = new Mat();
            Mat scaled_K = camMatrix * 1;
            scaled_K.put(2, 2, new double[] { 1.0 });

            Size dim2 = size.clone();
            Size dim3 = size.clone();
            //Debug.Log(scaled_K.dump());
            Calib3d.fisheye_estimateNewCameraMatrixForUndistortRectify(scaled_K, distCoeffs, dim2, E, new_K, 1.0);
            //Debug.Log(new_K.dump());

            Calib3d.fisheye_initUndistortRectifyMap(scaled_K, distCoeffs, E, new_K, dim3, CvType.CV_16SC2, map1, map2);
            Imgproc.remap(img_result, outMat, map1, map2, Imgproc.INTER_LINEAR, Core.BORDER_CONSTANT);
            // OpenCVForUnity.Calib3dModule.Calib3d.fisheye_undistortImage(img_result, outMat, camMatrix, distCoeffs, camMatrix, size);
            OpenCVForUnity.UnityUtils.Utils.matToTexture2D(outMat, rawVideoTexturesRGBA, false, 0, true);
            //rawVideoTexturesRGBA.Apply();
        }


        async void Service()
        {
            System.Net.IPAddress ipaddress = System.Net.IPAddress.Parse(serverAddress);
            IPEndPoint ipEndPoint = new(ipaddress, 6464);
            

            using (Socket listener = new Socket(ipEndPoint.AddressFamily, SocketType.Stream, ProtocolType.Tcp))
            {
                listener.Bind(ipEndPoint);
                listener.Listen(100);
                while (true)
                {
                    Socket handler = await listener.AcceptAsync();
                    using (NetworkStream myNetworkStream = new NetworkStream(handler))
                    {
                        // Check to see if this NetworkStream is readable.
                        if (myNetworkStream.CanRead)
                        {

                            using (var reader = new StreamReader(myNetworkStream))
                            {
                                while (true)
                                {
                                    try
                                    {
                                        int lengthBufferSize = 4;
                                        byte[] myLengthBuffer = new byte[lengthBufferSize];
                                        byte[] tempLengthByteArray = new byte[lengthBufferSize];
                                        int totalLengthBytes = 0;
                                        int numberOfLengthBytesRead = 0;
                                        //read in integer value that contains length of next read (the image itself)
                                        do
                                        {
                                            if (lengthBufferSize > (4 - totalLengthBytes))
                                            {
                                                lengthBufferSize = 4 - totalLengthBytes;
                                            }

                                            numberOfLengthBytesRead = await myNetworkStream.ReadAsync(myLengthBuffer, 0, lengthBufferSize);
                                            //Debug.Log("Read length bytes: " + numberOfLengthBytesRead);
                                            Array.Copy(myLengthBuffer, 0, tempLengthByteArray, totalLengthBytes, numberOfLengthBytesRead);
                                            totalLengthBytes += numberOfLengthBytesRead;
                                        }
                                        while (totalLengthBytes < 4);

                                        //read in the image, and only the exact number of bytes required

                                        int imageSize = BitConverter.ToInt32(myLengthBuffer, 0);

                                        if (imageSize > 5000000 || imageSize == null)
                                        {
                                            Debug.Log("Image corrupted, breaking out of receive loop");
                                            reader.Close();
                                            myNetworkStream.Close();
                                            break;
                                        }

                                        int totalBytes = 0;
                                        int bufferSize = 65000;
                                        byte[] myReadBuffer = new byte[bufferSize];
                                        byte[] tempByteArray = new byte[imageSize];
                                        int numberOfBytesRead = 0;
                                        //Debug.Log("Expected image size " + imageSize);
                                        // Incoming message may be larger than the buffer size.
                                        do
                                        {
                                            if (bufferSize > ((int)imageSize - totalBytes))
                                            {
                                                bufferSize = (int)imageSize - totalBytes;
                                            }
                                            //Debug.Log("Reading....");
                                            numberOfBytesRead = await myNetworkStream.ReadAsync(myReadBuffer, 0, bufferSize);
                                            Array.Copy(myReadBuffer, 0, tempByteArray, totalBytes, numberOfBytesRead);
                                            totalBytes += numberOfBytesRead;
                                        }
                                        while (totalBytes < imageSize);

                                        //load image on Texture2D
                                        //Debug.Log("You received an image of length : " + totalBytes);
                                        if (undistort)
                                        {
                                            UndistortImage(tempByteArray);
                                        }
                                        else
                                        {
                                            ImageConversion.LoadImage(rawVideoTexturesRGBA, tempByteArray);
                                        }
   
                                    }
                                    catch (Exception exception)
                                    {
                                        Debug.Log(exception.Message + " " + exception.StackTrace);
                                    }


                                }


                                reader.Close();
                            }

                        }
                        else
                        {
                            Debug.Log("Sorry.  You cannot read from this NetworkStream.");

                        }

                        myNetworkStream.Close();
                    }
                    handler.Close();
                }

                listener.Close();
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

