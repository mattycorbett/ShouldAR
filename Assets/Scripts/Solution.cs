// Copyright (c) 2021 homuler
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

using System.Collections;
using UnityEngine;

namespace Mediapipe.Unity
{
  public abstract class Solution : MonoBehaviour
  {
        private static readonly string _BootstrapName = nameof(Bootstrap);

        [SerializeField] private GameObject _bootstrapPrefab;

        private Texture2D oldTexture;

#pragma warning disable IDE1006
        // TODO: make it static
        protected virtual string TAG => GetType().Name;
#pragma warning restore IDE1006

        public Bootstrap bootstrap { get; private set; }
        protected bool isPaused;

        protected virtual IEnumerator Start()
        {
            oldTexture = new Texture2D(1024, 768);
            bootstrap = FindBootstrap();
            yield return new WaitUntil(() => bootstrap.isFinished);

            Play();
        }


        /// <summary>
        ///   Start the main program from the beginning.
        /// </summary>
        public virtual void Play()
        {
            isPaused = false;
        }

        /// <summary>
        ///   Pause the main program.
        /// <summary>
        public virtual void Pause()
        {
            isPaused = true;
        }

        /// <summary>
        ///    Resume the main program.
        ///    If the main program has not begun, it'll do nothing.
        /// </summary>
        public virtual void Resume()
        {
            isPaused = false;
        }

        /// <summary>
        ///   Stops the main program.
        /// </summary>
        public virtual void Stop()
        {
            isPaused = true;
        }

        protected static void SetupAnnotationController<T>(AnnotationController<T> annotationController, ImageSource imageSource, bool expectedToBeMirrored = false) where T : HierarchicalAnnotation
        {
            annotationController.isMirrored = expectedToBeMirrored ^ imageSource.isHorizontallyFlipped ^ imageSource.isFrontFacing;
            annotationController.rotationAngle = imageSource.rotation.Reverse();
        }

        protected void ReadFromImageSource(ImageSource imageSource, TextureFrame textureFrame)
        {

            var sourceTexture = imageSource.GetCurrentTexture();
            // For some reason, when the image is coiped on GPU, latency tends to be high.
            // So even when OpenGL ES is available, use CPU to copy images.
            var textureType = sourceTexture.GetType();
            if (textureType == typeof(WebCamTexture))
            {
                textureFrame.ReadTextureFromOnCPU((WebCamTexture)sourceTexture);
            }
            else if (textureType == typeof(Texture2D))
            {
                textureFrame.ReadTextureFromOnCPU((Texture2D)sourceTexture);
            }
            else
            {
                textureFrame.ReadTextureFromOnCPU(sourceTexture);
            }



        }

        protected Bootstrap FindBootstrap()
        {
            var bootstrapObj = GameObject.Find(_BootstrapName);

            if (bootstrapObj == null)
            {
                Debug.Log("Initializing the Bootstrap GameObject");
                bootstrapObj = Instantiate(_bootstrapPrefab);
                bootstrapObj.name = _BootstrapName;
                DontDestroyOnLoad(bootstrapObj);
            }

            return bootstrapObj.GetComponent<Bootstrap>();
        }

        private bool CompareTexture(Texture2D first, Texture2D second)
        {
            if(first == null)
            {
                return true;
            }
            else
            {
                UnityEngine.Color[] firstPix = first.GetPixels();
                UnityEngine.Color[] secondPix = second.GetPixels();
                if (firstPix.Length != secondPix.Length)
                {
                    return false;
                }
                for (int i = 0; i < firstPix.Length; i++)
                {
                    if (firstPix[i] != secondPix[i])
                    {
                        return false;
                    }
                }

                return true;
            }

        }
    }
}
