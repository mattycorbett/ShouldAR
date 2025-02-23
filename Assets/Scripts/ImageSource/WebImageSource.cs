// Copyright (c) 2021 homuler
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

using System;
using System.Collections;
using System.Linq;
using UnityEngine;
using UnityEngine.Networking;

namespace Mediapipe.Unity
{
  public class WebImageSource : ImageSource
  {

    [SerializeField] private string _uri;

    [SerializeField]
    private ResolutionStruct[] _defaultAvailableResolutions = new ResolutionStruct[] {
      new ResolutionStruct(512, 512, 0),
      new ResolutionStruct(640, 480, 0),
        new ResolutionStruct(800, 600, 0),
      new ResolutionStruct(1280, 720, 0),
    };

    public WebImageSource(string uri, ResolutionStruct[] defaultAvailableResolutions)
    {
      _uri = uri;
      _defaultAvailableResolutions = defaultAvailableResolutions;
    }

        private Texture2D _outputTexture;
        private Texture2D _nullTexture = null;
        private Texture _image;
        private Texture image
    {
      get
      {
        if (_image == null && _uri != null && _uri.Length > 0)
        {
          image = GetCurrentTexture();
        }
        return _image;
      }
      set
      {
        _image = value;
        resolution = GetDefaultResolution();
      }
    }

    public override double frameRate => 0;

    public override string sourceName => image != null ? image.name : null;

    public override string[] sourceCandidateNames => _uri.Cast<char>().Cast<string>().ToArray();

    public override ResolutionStruct[] availableResolutions => _defaultAvailableResolutions;

    public override bool isPrepared => _outputTexture != null;

    private bool _isPlaying = false;
    public override bool isPlaying => _isPlaying;

    public override void SelectSource(int sourceId)
    {
      if (sourceId < 0 || sourceId >= _uri.Length)
      {
        throw new ArgumentException($"Invalid source ID: {sourceId}");
      }

            image = GetCurrentTexture();
    }

    public override IEnumerator Play()
    {
      if (image == null)
      {
        throw new InvalidOperationException("Image is not selected");
      }
      if (isPlaying)
      {
        yield break;
      }

      _isPlaying = true;
      yield return null;
    }

    public override IEnumerator Resume()
    {
      if (!isPrepared)
      {
        throw new InvalidOperationException("Image is not prepared");
      }
      _isPlaying = true;

      yield return null;
    }

    public override void Pause()
    {
      _isPlaying = false;
    }
    public override void Stop()
    {
      _isPlaying = false;
      _outputTexture = null;
    }

    public override Texture GetCurrentTexture()
    {
            _outputTexture = GameObject.Find("WebCameraCapture").GetComponent<WebCameraCapture>().rawVideoTexturesRGBA;
            return _outputTexture;

        }



        private ResolutionStruct GetDefaultResolution()
        {
            var resolutions = availableResolutions;

            return (resolutions == null || resolutions.Length == 0) ? new ResolutionStruct() : resolutions[0];
        }

  }
}
