using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Defines vr_world, the frame every pose exchanged with ROS is expressed in.
///
/// Put this on an empty GameObject placed once in the room, and never move it after
/// registering (if you do, register again).
///
/// Game code: IsRegistered says whether the robot registration holds right now;
/// onRegistered / onRegistrationLost fire when that changes (start or pause the game there).
///
/// Recentering the headset (long-press the Meta button) moves Unity's whole world in the
/// room, this object included, unless it carries a localized OVRSpatialAnchor. Then
/// vr_world is somewhere else and the robot registration is wrong. This script notices
/// the recenter and sets RegistrationStale: QuestStatePublisher then reports everything
/// untracked, so the robot side stops following, until a registration script succeeds
/// again. With a Stage tracking origin (recentering off) or an anchor on this object,
/// the frame stays put.
///
/// Axis convention: ROS-TCP-Connector's FLU (x forward, y left, z up), which matches
/// ROS, so vr_world is right-handed and z-up like base_link.
/// </summary>
public class RobotWorldFrame : MonoBehaviour
{
    /// <summary>ROS frame id of this frame (header.frame_id of everything sent from Unity).</summary>
    public const string RosFrameId = "vr_world";

    public static RobotWorldFrame Instance { get; private set; }

    /// <summary>
    /// True after a recenter moved this frame in the room: the robot registration no longer
    /// holds. Cleared by MarkRegistered(), which the registration scripts call on success.
    /// </summary>
    public bool RegistrationStale { get; private set; }

    /// <summary>
    /// Goes up by one at every recenter that moves this frame. Registration data (sightings,
    /// touches, samples) collected under an older value are in the old vr_world: drop them.
    /// </summary>
    public int Generation { get; private set; }

    /// <summary>
    /// True once a registration succeeded in this app session and no recenter has moved
    /// vr_world since. A registration saved on the robot PC from an earlier session doesn't
    /// count: the headset's world can differ between sessions, so register at every start.
    /// </summary>
    public bool IsRegistered { get; private set; }

    [Tooltip("Fires after every successful registration (e.g. hide the calibration UI, start the game).")]
    public UnityEvent onRegistered = new UnityEvent();

    [Tooltip("Fires when a recenter breaks the registration (e.g. pause, show the calibration UI again).")]
    public UnityEvent onRegistrationLost = new UnityEvent();

    int _recenterCount;

    void Awake()
    {
        if (Instance != null && Instance != this)
            Debug.LogWarning("More than one RobotWorldFrame in the scene; using the latest.");
        Instance = this;
    }

    void Start()
    {
        // Count from now: recenters before the app started don't matter.
        _recenterCount = OVRPlugin.GetLocalTrackingSpaceRecenterCount();
        OVRManager.TrackingOriginChangePending += OnTrackingOriginChangePending;
    }

    void OnDestroy() => OVRManager.TrackingOriginChangePending -= OnTrackingOriginChangePending;

    void Update()
    {
        int count = OVRPlugin.GetLocalTrackingSpaceRecenterCount();
        if (count == _recenterCount) return;
        _recenterCount = count;
        Invalidate("The headset was recentered");
    }

    void OnTrackingOriginChangePending(OVRManager.TrackingOrigin origin, OVRPose? poseInPreviousSpace)
    {
        if (poseInPreviousSpace.HasValue) Invalidate($"The tracking origin ({origin}) moved");
    }

    void Invalidate(string why)
    {
        if (TryGetComponent(out OVRSpatialAnchor anchor) && anchor.Localized) return;   // stays in the room
        if (!RegistrationStale)
            Debug.LogWarning($"[RobotWorldFrame] {why}, so vr_world moved in the room and the robot " +
                             "registration no longer holds. Poses are sent as untracked until you register again.");
        RegistrationStale = true;
        Generation++;
        if (!IsRegistered) return;
        IsRegistered = false;
        onRegistrationLost.Invoke();
    }

    /// <summary>
    /// Call after a successful registration from data collected at <paramref name="generation"/>:
    /// vr_world and base_link agree again, unless a recenter happened since.
    /// </summary>
    public void MarkRegistered(int generation)
    {
        if (generation != Generation) return;      // recentered since: the new result is wrong too
        RegistrationStale = false;
        IsRegistered = true;
        onRegistered.Invoke();
    }

    /// <summary>Generation of RobotWorldFrame.Instance (0 if there is none).</summary>
    public static int CurrentGeneration => Instance != null ? Instance.Generation : 0;

    /// <summary>MarkRegistered on RobotWorldFrame.Instance, if there is one.</summary>
    public static void Registered(int generation)
    {
        if (Instance != null) Instance.MarkRegistered(generation);
    }

    public PointMsg ToRosPoint(Vector3 worldPosition) =>
        transform.InverseTransformPoint(worldPosition).To<FLU>();

    public QuaternionMsg ToRosRotation(Quaternion worldRotation) =>
        (Quaternion.Inverse(transform.rotation) * worldRotation).To<FLU>();

    /// <summary>
    /// The resulting ROS frame has x = the object's Unity forward, y = Unity left, z = Unity up.
    /// So a camera-like pose (forward = viewing direction, up = image up) becomes the "link"
    /// convention the ROS side expects: x = viewing direction, y = image-left, z = image-up.
    /// </summary>
    public PoseMsg ToRosPose(Vector3 worldPosition, Quaternion worldRotation) =>
        new PoseMsg(ToRosPoint(worldPosition), ToRosRotation(worldRotation));

    public PoseMsg ToRosPose(Transform t) => ToRosPose(t.position, t.rotation);

    public Vector3 FromRosPoint(PointMsg p) => transform.TransformPoint(p.From<FLU>());

    public Quaternion FromRosRotation(QuaternionMsg q) => transform.rotation * q.From<FLU>();

    /// <summary>Apply a ROS pose (in vr_world) to a Unity transform.</summary>
    public void Apply(PoseMsg pose, Transform target)
    {
        target.SetPositionAndRotation(FromRosPoint(pose.position), FromRosRotation(pose.orientation));
    }
}
