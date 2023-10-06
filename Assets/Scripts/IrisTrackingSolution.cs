// Copyright (c) 2021 homuler
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Mediapipe.Unity
{
  public class IrisTrackingSolution : ImageSourceSolution<IrisTrackingGraph>
  {

        public NormalizedLandmarkList currentFaceAndIrisLandmarks;
        public List<Detection> currentFaceList;

        protected override void OnStartRun()
        {
            if (!runningMode.IsSynchronous())
            {
                graphRunner.OnFaceDetectionsOutput += OnFaceDetectionsOutput;
                graphRunner.OnFaceRectOutput += OnFaceRectOutput;
                graphRunner.OnFaceLandmarksWithIrisOutput += OnFaceLandmarksWithIrisOutput;
            }

            var imageSource = ImageSourceProvider.ImageSource;
        }

        protected override void AddTextureFrameToInputStream(TextureFrame textureFrame)
        {
            graphRunner.AddTextureFrameToInputStream(textureFrame);
        }

        protected override IEnumerator WaitForNextValue()
        {
            List<Detection> faceDetections = null;
            NormalizedRect faceRect = null;
            NormalizedLandmarkList faceLandmarksWithIris = null;

            if (runningMode == RunningMode.Sync)
            {
                var _ = graphRunner.TryGetNext(out faceDetections, out faceRect, out faceLandmarksWithIris, true);
            }
            else if (runningMode == RunningMode.NonBlockingSync)
            {
                yield return new WaitUntil(() => graphRunner.TryGetNext(out faceDetections, out faceRect, out faceLandmarksWithIris, false));
            }

        }

        private void OnFaceDetectionsOutput(object stream, OutputEventArgs<List<Detection>> eventArgs)
        {
            //_faceDetectionsAnnotationController.DrawLater(eventArgs.value);
            //currentFaceList = eventArgs.value;
        }

        private void OnFaceRectOutput(object stream, OutputEventArgs<NormalizedRect> eventArgs)
        {
            //_faceRectAnnotationController.DrawLater(eventArgs.value);
        }

        private void OnFaceLandmarksWithIrisOutput(object stream, OutputEventArgs<NormalizedLandmarkList> eventArgs)
        {
            //_faceLandmarksWithIrisAnnotationController.DrawLater(eventArgs.value);
            currentFaceAndIrisLandmarks = eventArgs.value;

        }

    }
}
