using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;
using Mediapipe.Unity.CoordinateSystem;

using Stopwatch = System.Diagnostics.Stopwatch;

    namespace Mediapipe.Unity
    {
    public class FaceTrackingGraph : MonoBehaviour//: GraphRunner
    {

        [SerializeField] private TextAsset _irisConfigAsset;
        [SerializeField] private int timeoutMicrosec;

        public RotationAngle rotation { get; private set; } = 0;

        private CalculatorGraph _irisGraph;
        private StreamingAssetsResourceManager _resourceManager;

        private const string _FaceDetectionsStreamName = "face_detections";
        private const string _FaceRectStreamName = "face_rect";
        private const string _FaceLandmarksWithIrisStreamName = "face_landmarks_with_iris";

        private WebCamTexture _webCamTexture;
        private OutputStream<DetectionVectorPacket, List<Detection>> _faceDetectionsStream;
        private OutputStream<NormalizedRectPacket, NormalizedRect> _faceRectStream;
        private OutputStream<NormalizedLandmarkListPacket, NormalizedLandmarkList> _faceLandmarksWithIrisStream;
        private Color32[] eyeColors;
        private Stopwatch stopwatch;


        private IEnumerator Start()
        {

            /*using (var system = new AndroidJavaClass("java.lang.System"))
            {
                system.CallStatic("loadLibrary", "mediapipe_jni");
            }*/


#if UNITY_EDITOR

#endif

           // yield return GpuManager.Initialize();

           // if (!GpuManager.IsInitialized)
           // {
           //     throw new System.Exception("Failed to initialize GPU resources");
           // }


            eyeColors = new Color32[10 * 10];
            for (var i = 0; i < eyeColors.Length; ++i)
            {
                eyeColors[i] = UnityEngine.Color.green;
            }

            _resourceManager = new StreamingAssetsResourceManager();
            yield return _resourceManager.PrepareAssetAsync("face_detection_short_range.bytes");
            yield return _resourceManager.PrepareAssetAsync("face_landmark_with_attention.bytes");
            yield return _resourceManager.PrepareAssetAsync("face_landmark.bytes");
            yield return _resourceManager.PrepareAssetAsync("iris_landmark.bytes");

            Debug.Log("1");
            stopwatch = new Stopwatch();

            //_irisGraph.SetGpuResources(GpuManager.GpuResources);//.AssertOk();

            Debug.Log("1.5");
            _irisGraph = new CalculatorGraph();
            try
            {
                var baseConfig = _irisConfigAsset.text == null ? null : CalculatorGraphConfig.Parser.ParseFromTextFormat(_irisConfigAsset.text);
                if (baseConfig == null)
                {
                    throw new InvalidOperationException("Failed to get the text config. Check if the config is set to GraphRunner");
                }
                _irisGraph.Initialize(baseConfig); //var status = _irisGraph.Initialize(baseConfig);

            }
            catch (Exception e)
            {
                Debug.Log(e.ToString());
            }
            //_irisGraph = new CalculatorGraph(_irisConfigAsset.text);
            Debug.Log("2");
            _faceDetectionsStream = new OutputStream<DetectionVectorPacket, List<Detection>>(_irisGraph, _FaceDetectionsStreamName, true, timeoutMicrosec);
            _faceRectStream = new OutputStream<NormalizedRectPacket, NormalizedRect>(_irisGraph, _FaceRectStreamName, true, timeoutMicrosec);
            _faceLandmarksWithIrisStream = new OutputStream<NormalizedLandmarkListPacket, NormalizedLandmarkList>(_irisGraph, _FaceLandmarksWithIrisStreamName, true, timeoutMicrosec);
            Debug.Log("3");

            Debug.Log("5");
            _faceDetectionsStream.StartPolling();//.AssertOk();
            Debug.Log("6");
            _faceRectStream.StartPolling();//.AssertOk();
            Debug.Log("7");
            _faceLandmarksWithIrisStream.StartPolling();//.AssertOk();
            
            //_faceGraph.StartRun().AssertOk();
            Debug.Log("8");
            var sidePacket = new PacketMap();// new SidePacket();
            sidePacket.Emplace("input_rotation", new IntPacket(0));
            sidePacket.Emplace("input_horizontally_flipped", new BoolPacket(false));
            sidePacket.Emplace("input_vertically_flipped", new BoolPacket(true));

            _irisGraph.StartRun(sidePacket);//.AssertOk();
            Debug.Log("9");
            stopwatch.Start();

            #if UNITY_EDITOR

#endif

        }

        public List<(Vector2, float)> RunDetection(Texture2D rawVideoTexturesRGBA, int width, int height)
        {
            Debug.Log("Here1");
            var imageFrame = new ImageFrame(ImageFormat.Types.Format.Srgba, width, height, width * 4, rawVideoTexturesRGBA.GetRawTextureData<byte>());
            var currentTimestamp = stopwatch.ElapsedTicks / (System.TimeSpan.TicksPerMillisecond / 1000);
            Debug.Log("Here2");
            _irisGraph.AddPacketToInputStream("input_video", new ImageFramePacket(imageFrame, new Timestamp(currentTimestamp)));//.AssertOk();

            Debug.Log("Here3");

            Debug.Log("Here4");

            Debug.Log("Here5");
            TryGetNext(out var faceDetections, out NormalizedRect faceRect, out NormalizedLandmarkList faceLandmarksWithIris);
            Debug.Log("Here6");
            if (faceRect != null)
            {
                Debug.Log($"Image Coordinates (Face X Coord): {(int)(faceRect.XCenter * width)}");
                Debug.Log($"Image Coordinates (Face Y Coord): {(int)(faceRect.YCenter * height)}");

            }
            Debug.Log("Here7");

            var outputList = new List<(Vector2, float)>();

            if (faceLandmarksWithIris != null)
            {
                // top of the head
                var rightEyeX = (faceLandmarksWithIris.Landmark[469].X + faceLandmarksWithIris.Landmark[470].X + faceLandmarksWithIris.Landmark[471].X + faceLandmarksWithIris.Landmark[472].X) / 4;
                var rightEyeY = (faceLandmarksWithIris.Landmark[469].Y + faceLandmarksWithIris.Landmark[470].Y + faceLandmarksWithIris.Landmark[471].Y + faceLandmarksWithIris.Landmark[472].Y) / 4;
                var leftEyeX = (faceLandmarksWithIris.Landmark[474].X + faceLandmarksWithIris.Landmark[475].X + faceLandmarksWithIris.Landmark[476].X + faceLandmarksWithIris.Landmark[477].X) / 4;
                var leftEyeY = (faceLandmarksWithIris.Landmark[474].Y + faceLandmarksWithIris.Landmark[475].Y + faceLandmarksWithIris.Landmark[476].Y + faceLandmarksWithIris.Landmark[477].Y) / 4;
                Debug.Log($"Right Eye Image Coordinates: {leftEyeX},{leftEyeY}");
                Debug.Log($"Left Eye Image Coordinates: {rightEyeX},{rightEyeY}");
                Debug.Log("Here8");
                outputList.Add((new Vector2((faceRect.XCenter * width), height - (faceRect.YCenter * height)), faceRect.Width));

            }

            return outputList;
        }

      /*  public override void StartRun(ImageSource imageSource)
        {
            if (runningMode.IsSynchronous())
            {
                _faceDetectionsStream.StartPolling().AssertOk();
                _faceRectStream.StartPolling().AssertOk();
                _faceLandmarksWithIrisStream.StartPolling().AssertOk();
            }
            StartRun(BuildSidePacket(imageSource));
        }

        protected override IList<WaitForResult> RequestDependentAssets()
        {
            return new List<WaitForResult> {
        WaitForAsset("face_detection_short_range.bytes"),
        WaitForAsset("face_landmark.bytes"),
        WaitForAsset("iris_landmark.bytes"),
      };
        }

        private SidePacket BuildSidePacket(ImageSource imageSource)
        {
            var sidePacket = new SidePacket();
            SetImageTransformationOptions(sidePacket, imageSource);
            return sidePacket;
        }*/

        protected bool TryGetNext<TPacket, TValue>(OutputStream<TPacket, TValue> stream, out TValue value, bool allowBlock, long currentTimestampMicrosec) where TPacket : Packet<TValue>, new()
        {
            var result = stream.TryGetNext(out value, allowBlock);
            return result || allowBlock || stream.ResetTimestampIfTimedOut(currentTimestampMicrosec, timeoutMicrosec);
        }

        protected void StartRun(PacketMap sidePacket)
        {
            _irisGraph.StartRun(sidePacket);//.AssertOk();
        }

        public bool TryGetNext(out List<Detection> faceDetections, out NormalizedRect faceRect, out NormalizedLandmarkList faceLandmarksWithIris, bool allowBlock = true)
        {
            var currentTimestampMicrosec = stopwatch.ElapsedTicks / (System.TimeSpan.TicksPerMillisecond / 1000);
            var r1 = TryGetNext(_faceDetectionsStream, out faceDetections, allowBlock, currentTimestampMicrosec);
            var r2 = TryGetNext(_faceRectStream, out faceRect, allowBlock, currentTimestampMicrosec);
            var r3 = TryGetNext(_faceLandmarksWithIrisStream, out faceLandmarksWithIris, allowBlock, currentTimestampMicrosec);

            return r1 || r2 || r3;
        }



        private void OnDestroy()
        {

            if (_irisGraph != null)
            {
                try
                {
                    _irisGraph.CloseInputStream("input_video");//.AssertOk();
                    _irisGraph.WaitUntilDone();//.AssertOk();
                }
                finally
                {

                    _irisGraph.Dispose();
                }
            }

        }
    }
}