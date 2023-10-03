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
            currentFaceList = eventArgs.value;
    }

    private void OnFaceRectOutput(object stream, OutputEventArgs<NormalizedRect> eventArgs)
    {
      //_faceRectAnnotationController.DrawLater(eventArgs.value);
    }

    private void OnFaceLandmarksWithIrisOutput(object stream, OutputEventArgs<NormalizedLandmarkList> eventArgs)
    {
            //_faceLandmarksWithIrisAnnotationController.DrawLater(eventArgs.value);
            currentFaceAndIrisLandmarks = eventArgs.value;
            /*if (eventArgs.value != null)
            {
                // top of the head
                var rightEyeX = (eventArgs.value.Landmark[469].X + eventArgs.value.Landmark[470].X + eventArgs.value.Landmark[471].X + eventArgs.value.Landmark[472].X) / 4;
                var rightEyeY = (eventArgs.value.Landmark[469].Y + eventArgs.value.Landmark[470].Y + eventArgs.value.Landmark[471].Y + eventArgs.value.Landmark[472].Y) / 4;
                var leftEyeX = (eventArgs.value.Landmark[474].X + eventArgs.value.Landmark[475].X + eventArgs.value.Landmark[476].X + eventArgs.value.Landmark[477].X) / 4;
                var leftEyeY = (eventArgs.value.Landmark[474].Y + eventArgs.value.Landmark[475].Y + eventArgs.value.Landmark[476].Y + eventArgs.value.Landmark[477].Y) / 4;
                Debug.Log($"Right Eye Image Coordinates: {leftEyeX},{leftEyeY}");
                Debug.Log($"Left Eye Image Coordinates: {rightEyeX},{rightEyeY}");


            }*/

        }

  }
}
