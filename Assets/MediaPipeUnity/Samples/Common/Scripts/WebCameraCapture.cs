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
            rawVideoTexturesRGBA = new Texture2D(1024, 768);
            //rawVideoTexturesRGBA = new Texture2D(800, 600, TextureFormat.RGBA32, false, true);
            GetSocketImage();
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
                
                //rawVideoTexturesRGBA.LoadRawTextureData(bytes);
                //rawVideoTexturesRGBA.Apply();
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

        async void Service()
        {
            System.Net.IPAddress ipaddress = System.Net.IPAddress.Parse("10.0.0.99");
            IPEndPoint ipEndPoint = new(ipaddress, 6464);
            

            using (Socket listener = new Socket(ipEndPoint.AddressFamily, SocketType.Stream, ProtocolType.Tcp))
            {
                listener.Bind(ipEndPoint);
                listener.Listen(100);
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
                                    //numberOfLengthBytesRead = await myNetworkStream.ReadAsync(myLengthBuffer, 0, 4);
                                    do
                                    {
                                        if (lengthBufferSize > (4 - totalLengthBytes))
                                        {
                                            lengthBufferSize = 4 - totalLengthBytes;
                                        }

                                        numberOfLengthBytesRead = await myNetworkStream.ReadAsync(myLengthBuffer, 0, 4);
                                        //Debug.Log("Read length bytes: " + numberOfLengthBytesRead);
                                        Array.Copy(myLengthBuffer, 0, tempLengthByteArray, totalLengthBytes, numberOfLengthBytesRead);
                                        totalLengthBytes += numberOfLengthBytesRead;
                                    }
                                    while (totalLengthBytes < 4);

                                    //Debug.Log(totalLengthBytes);
                                    
                                    int imageSize = BitConverter.ToInt32(myLengthBuffer, 0);

                                    if(imageSize > 5000000 || imageSize == null)
                                    {
                                        Debug.Log("Image corrupted, breaking out of receive loop");
                                        break;
                                    }

                                    int totalBytes = 0;
                                    int bufferSize = 65000;
                                    byte[] myReadBuffer = new byte[bufferSize];
                                    byte[] tempByteArray = new byte[imageSize];
                                    int numberOfBytesRead = 0;
                                    Debug.Log("Expected image size " + imageSize);
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

                                    // Print out the received message to the console.
                                    Debug.Log("You received an image of length : " + totalBytes);
 
                                    ImageConversion.LoadImage(rawVideoTexturesRGBA, tempByteArray);
                                }
                                catch (Exception exception)
                                {
                                    Debug.Log(exception.Message);
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

