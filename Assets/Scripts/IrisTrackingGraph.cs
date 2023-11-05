// Copyright (c) 2021 homuler
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

using System;
using System.Collections.Generic;

namespace Mediapipe.Unity
{
  public class IrisTrackingGraph : GraphRunner
  {
        public event EventHandler<OutputEventArgs<List<Detection>>> OnFaceDetectionsOutput
        {
            add => _faceDetectionsStream.AddListener(value);
            remove => _faceDetectionsStream.RemoveListener(value);
        }

        public event EventHandler<OutputEventArgs<NormalizedRect>> OnFaceRectOutput
        {
            add => _faceRectStream.AddListener(value);
            remove => _faceRectStream.RemoveListener(value);
        }

        public event EventHandler<OutputEventArgs<NormalizedLandmarkList>> OnFaceLandmarksWithIrisOutput
        {
            add => _faceLandmarksWithIrisStream.AddListener(value);
            remove => _faceLandmarksWithIrisStream.RemoveListener(value);
        }

        public event EventHandler<OutputEventArgs<List<NormalizedLandmarkList>>> OnFaceLandmarksOutput
        {
            add => _faceLandmarksStream.AddListener(value);
            remove => _faceLandmarksStream.RemoveListener(value);
        }

        private const string _InputStreamName = "input_video";

        private const string _FaceDetectionsStreamName = "face_detections";
        private const string _FaceRectStreamName = "face_rect";
        private const string _FaceLandmarksWithIrisStreamName = "face_landmarks_with_iris";
        private const string _FaceLandmarksStreamName = "multi_face_landmarks";
        

        private OutputStream<DetectionVectorPacket, List<Detection>> _faceDetectionsStream;
        private OutputStream<NormalizedRectPacket, NormalizedRect> _faceRectStream;
        private OutputStream<NormalizedLandmarkListPacket, NormalizedLandmarkList> _faceLandmarksWithIrisStream;
        private OutputStream<NormalizedLandmarkListVectorPacket, List<NormalizedLandmarkList>> _faceLandmarksStream;


        public override void StartRun(ImageSource imageSource)
        {
            if (runningMode.IsSynchronous())
            {
                _faceDetectionsStream.StartPolling();
                _faceRectStream.StartPolling();
                _faceLandmarksWithIrisStream.StartPolling();
                _faceLandmarksStream.StartPolling();
            }
            StartRun(BuildSidePacket(imageSource));
        }

        public override void Stop()
        {
            _faceDetectionsStream?.Close();
            _faceDetectionsStream = null;
            _faceRectStream?.Close();
            _faceRectStream = null;
            _faceLandmarksWithIrisStream?.Close();
            _faceLandmarksWithIrisStream = null;
            base.Stop();
        }

        public void AddTextureFrameToInputStream(TextureFrame textureFrame)
        {
            AddTextureFrameToInputStream(_InputStreamName, textureFrame);
        }

        public bool TryGetNext(out List<Detection> faceDetections, out NormalizedRect faceRect, out NormalizedLandmarkList faceLandmarksWithIris, out List<NormalizedLandmarkList> faceLandmarks, bool allowBlock = true)
        {
            var currentTimestampMicrosec = GetCurrentTimestampMicrosec();
            var r1 = TryGetNext(_faceDetectionsStream, out faceDetections, allowBlock, currentTimestampMicrosec);
            var r2 = TryGetNext(_faceRectStream, out faceRect, allowBlock, currentTimestampMicrosec);
            var r3 = TryGetNext(_faceLandmarksWithIrisStream, out faceLandmarksWithIris, allowBlock, currentTimestampMicrosec);
            var r4 = TryGetNext(_faceLandmarksStream, out faceLandmarks, allowBlock, currentTimestampMicrosec);

            return r1 || r2 || r3 || r4;
        }

        protected override IList<WaitForResult> RequestDependentAssets()
        {
            return new List<WaitForResult> {
        WaitForAsset("face_detection_full_range_sparse.bytes"),
        WaitForAsset("face_landmark.bytes"),
        WaitForAsset("iris_landmark.bytes"),
      };
        }

        protected override void ConfigureCalculatorGraph(CalculatorGraphConfig config)
        {
            if (runningMode == RunningMode.NonBlockingSync)
            {
                _faceDetectionsStream = new OutputStream<DetectionVectorPacket, List<Detection>>(
                    calculatorGraph, _FaceDetectionsStreamName, config.AddPacketPresenceCalculator(_FaceDetectionsStreamName), timeoutMicrosec);
                _faceRectStream = new OutputStream<NormalizedRectPacket, NormalizedRect>(
                    calculatorGraph, _FaceRectStreamName, config.AddPacketPresenceCalculator(_FaceRectStreamName), timeoutMicrosec);
                _faceLandmarksWithIrisStream = new OutputStream<NormalizedLandmarkListPacket, NormalizedLandmarkList>(
                    calculatorGraph, _FaceLandmarksWithIrisStreamName, config.AddPacketPresenceCalculator(_FaceLandmarksWithIrisStreamName), timeoutMicrosec);
                _faceLandmarksStream = new OutputStream<NormalizedLandmarkListVectorPacket, List<NormalizedLandmarkList>>(
    calculatorGraph, _FaceLandmarksStreamName, config.AddPacketPresenceCalculator(_FaceLandmarksStreamName), timeoutMicrosec);
            }
            else
            {
                _faceDetectionsStream = new OutputStream<DetectionVectorPacket, List<Detection>>(calculatorGraph, _FaceDetectionsStreamName, true, timeoutMicrosec);
                _faceRectStream = new OutputStream<NormalizedRectPacket, NormalizedRect>(calculatorGraph, _FaceRectStreamName, true, timeoutMicrosec);
                _faceLandmarksWithIrisStream = new OutputStream<NormalizedLandmarkListPacket, NormalizedLandmarkList>(calculatorGraph, _FaceLandmarksWithIrisStreamName, true, timeoutMicrosec);
                _faceLandmarksStream = new OutputStream<NormalizedLandmarkListVectorPacket, List<NormalizedLandmarkList>>(calculatorGraph, _FaceLandmarksStreamName, true, timeoutMicrosec);
            }
            calculatorGraph.Initialize(config);
        }

        private PacketMap BuildSidePacket(ImageSource imageSource)
        {
            var sidePacket = new PacketMap();
            SetImageTransformationOptions(sidePacket, imageSource);
            return sidePacket;
        }
    }
}
