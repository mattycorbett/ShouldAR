﻿// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

using UnityEngine;

namespace Mediapipe.Unity.IrisTracking
{
    /// <summary>
    /// This solver determines the position and orientation of an object as a directional indicator. 
    /// From the point of reference of the SolverHandler Tracked Target, this indicator will orient towards the DirectionalTarget supplied.
    /// If the Directional Target is deemed within view of our frame of reference, then all renderers under this Solver will be disabled. They will be enabled otherwise
    /// </summary>
    [AddComponentMenu("Scripts")]
    public class DirectionalIndicator : MonoBehaviour
    {
        /// <summary>
        /// The GameObject transform to point the indicator towards when this object is not in view. 
        /// The frame of reference for viewing is defined by the Solver Handler Tracked Target Type
        /// </summary>
        [Tooltip("The GameObject transform to point the indicator towards when this object is not in view.\nThe frame of reference for viewing is defined by the Solver Handler Tracked Target Type")]
        public Transform DirectionalTarget;

        /// <summary>
        /// The minimum scale for the indicator object
        /// </summary>
        [Tooltip("The offset from center to place the indicator. If frame of reference is the camera, then the object will be this distance from center of screen")]
        [Min(0.0f)]
        public float MinIndicatorScale = 0.05f;

        /// <summary>
        /// The maximum scale for the indicator object
        /// </summary>
        [Tooltip("The offset from center to place the indicator. If frame of reference is the camera, then the object will be this distance from center of screen")]
        [Min(0.0f)]
        public float MaxIndicatorScale = 0.2f;

        /// <summary>
        /// Multiplier factor to increase or decrease FOV range for testing if object is visible and thus turn off indicator
        /// </summary>
        [Tooltip("Multiplier factor to increase or decrease FOV range for testing if object is visible and thus turn off indicator")]
        [Min(0.1f)]
        public float VisibilityScaleFactor = 0.8f;

        /// <summary>
        /// The offset from center to place the indicator. If frame of reference is the camera, then the object will be this distance from center of screen
        /// </summary>
        [Tooltip("The offset from center to place the indicator. If frame of reference is the camera, then the object will be this distance from center of screen")]
        [Min(0.0f)]
        public float ViewOffset = 0.3f;

        private bool indicatorShown = false;

        void Start()
        {


        }

        /// <inheritdoc />
        void Update()
        {


            // SolverUpdate is generally called in LateUpdate, at a time when it's possible that the DirectionalTarget
            // has already been destroyed. This ensures that if the object has been destroyed, we don't access invalid
            if (DirectionalTarget == null)
            {
                return;
            }

            // This is the frame of reference to use when solving for the position of this.gameobject
            // The frame of reference will likely be the main camera
            var solverReferenceFrame = Camera.main.transform;

            Vector3 origin = solverReferenceFrame.position + solverReferenceFrame.forward;

            Vector3 trackerToTargetDirection = (DirectionalTarget.position - solverReferenceFrame.position).normalized;

            // Project the vector (from the frame of reference (SolverHandler target) to the Directional Target) onto the "viewable" plane
            Vector3 indicatorDirection = Vector3.ProjectOnPlane(trackerToTargetDirection, -solverReferenceFrame.forward).normalized;

            // If the our indicator direction is 0, set the direction to the right.
            // This will only happen if the frame of reference (SolverHandler target) is facing directly away from the directional target.
            if (indicatorDirection == Vector3.zero)
            {
                indicatorDirection = solverReferenceFrame.right;
            }

            // The final position is translated from the center of the frame of reference plane along the indicator direction vector.
            Vector3 GoalPosition = origin + indicatorDirection * ViewOffset;

            // Find the rotation from the facing direction to the target object.
            Quaternion GoalRotation = Quaternion.LookRotation(solverReferenceFrame.forward, indicatorDirection);

            // Scale the solver based to be more prominent if the object is far away from the field of view
            float minVisiblityAngle = VisibilityScaleFactor * Camera.main.fieldOfView * 0.5f;

            float angleToVisbilityFOV = Vector3.Angle(trackerToTargetDirection - solverReferenceFrame.position, solverReferenceFrame.forward) - minVisiblityAngle;
            float visibilityScale = 180f - minVisiblityAngle;

            Vector3 GoalScale = Vector3.one * Mathf.Lerp(MinIndicatorScale, MaxIndicatorScale, angleToVisbilityFOV / visibilityScale);
        }
    }
}
