using UnityEngine;

/// <summary>
/// Gives a game object's pose (e.g. the block) in the Quest's own tracking space: the frame the
/// headset reports its head and controller poses in. Units are metres, axes are Unity's
/// (x right, y up, z forward). With a Floor Level tracking origin, y = 0 is the floor.
///
/// Unity world space is only the same as tracking space while the OVRCameraRig sits at the
/// origin. Locomotion (snap turn, teleport, slide from Meta's interaction rig) moves the rig
/// and breaks that. This script always goes through the rig's TrackingSpace, so its result is
/// right whatever the rig does. Two limits, reported by the properties below:
///  - RigMoved: the rig moved in Unity world since Start. The numbers are still right, but the
///    virtual object has slid in the real room, so it no longer marks the same real place.
///  - Recentered: the headset was recentered since Start (Floor Level / Eye Level origins only).
///    Tracking space itself moved in the room, so the same numbers mean a different real place.
///
/// No ROS dependency: this file works on its own in any project with the Meta XR Core SDK.
/// To send a pose to the robot, use RobotWorldFrame.ToRosPose instead (vr_world, ROS axes).
/// </summary>
public class TrackingSpacePose : MonoBehaviour
{
    [Tooltip("Object to measure. Defaults to this GameObject. Put this script on an object that " +
             "stays active if the target gets hidden, and point Target at whatever is visible.")]
    public Transform target;

    [Tooltip("Found automatically if empty.")]
    public OVRCameraRig cameraRig;

    [Tooltip("Log the pose to the Console / logcat this often, in seconds. 0 = never.")]
    public float logEverySeconds;

    Pose _rigHome;
    bool _hasHome;
    int _recenterHome;
    float _nextLog;

    /// <summary>The rig's TrackingSpace transform, or null if there is no rig.</summary>
    public Transform TrackingSpace => cameraRig != null ? cameraRig.trackingSpace : null;

    /// <summary>The target's pose in tracking space, computed now.</summary>
    public Pose Pose => target != null ? ToTrackingSpace(TrackingSpace, target.position, target.rotation) : Pose.identity;

    /// <summary>The target's position in tracking space, computed now.</summary>
    public Vector3 Position => Pose.position;

    /// <summary>
    /// False when there is no rig or target, the target is hidden, or the headset isn't tracked.
    /// </summary>
    public bool IsValid => TrackingSpace != null && target != null && target.gameObject.activeInHierarchy
                           && (!Application.isPlaying || OVRManager.isHmdPresent);

    /// <summary>The rig moved in Unity world (locomotion) since Start or the last MarkHome.</summary>
    public bool RigMoved
    {
        get
        {
            if (!_hasHome || cameraRig == null) return false;
            Transform rig = cameraRig.transform;
            return (rig.position - _rigHome.position).sqrMagnitude > 1e-6f
                   || Quaternion.Angle(rig.rotation, _rigHome.rotation) > 0.01f;
        }
    }

    /// <summary>The headset was recentered since Start or the last MarkHome.</summary>
    public bool Recentered => Application.isPlaying && OVRPlugin.GetLocalTrackingSpaceRecenterCount() != _recenterHome;

    void Reset() => target = transform;

    void Awake()
    {
        if (target == null) target = transform;
        if (cameraRig == null) cameraRig = FindAnyObjectByType<OVRCameraRig>();
        if (cameraRig == null) Debug.LogError("[TrackingSpacePose] No OVRCameraRig in the scene.", this);
    }

    void Start() => MarkHome();

    /// <summary>Take the rig's current place and the current recenter count as the reference.</summary>
    public void MarkHome()
    {
        if (cameraRig != null)
        {
            _rigHome = new Pose(cameraRig.transform.position, cameraRig.transform.rotation);
            _hasHome = true;
        }
        if (Application.isPlaying) _recenterHome = OVRPlugin.GetLocalTrackingSpaceRecenterCount();
    }

    void LateUpdate()
    {
        if (logEverySeconds <= 0f || Time.unscaledTime < _nextLog) return;
        _nextLog = Time.unscaledTime + logEverySeconds;
        Pose p = Pose;
        Debug.Log($"[TrackingSpacePose] {(target != null ? target.name : "?")} in tracking space: " +
                  $"pos=({p.position.x:F3}, {p.position.y:F3}, {p.position.z:F3}) m, " +
                  $"yaw={p.rotation.eulerAngles.y:F1} deg, valid={IsValid}, rigMoved={RigMoved}, recentered={Recentered}", this);
    }

    /// <summary>World pose -> tracking-space pose. A null trackingSpace means world = tracking space.</summary>
    public static Pose ToTrackingSpace(Transform trackingSpace, Vector3 worldPosition, Quaternion worldRotation)
    {
        if (trackingSpace == null) return new Pose(worldPosition, worldRotation);
        return new Pose(trackingSpace.InverseTransformPoint(worldPosition),
                        Quaternion.Inverse(trackingSpace.rotation) * worldRotation);
    }

    /// <summary>Tracking-space pose -> world pose (e.g. to place an object at a real-room spot).</summary>
    public static Pose ToWorld(Transform trackingSpace, Pose trackingPose)
    {
        if (trackingSpace == null) return trackingPose;
        return new Pose(trackingSpace.TransformPoint(trackingPose.position),
                        trackingSpace.rotation * trackingPose.rotation);
    }
}
