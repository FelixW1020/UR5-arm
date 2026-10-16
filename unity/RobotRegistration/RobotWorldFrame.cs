using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>
/// Defines vr_world, the frame every pose exchanged with ROS is expressed in.
///
/// Put this on an empty GameObject placed once in the room. Poses are taken relative
/// to it (not to Unity's world origin), so recentering the headset doesn't break the
/// robot registration. Add an OVRSpatialAnchor to the same object and persist it if
/// you want the alignment to survive app restarts. Never move it after registering;
/// if you do, register again.
///
/// Axis convention: ROS-TCP-Connector's FLU (x forward, y left, z up), which matches
/// ROS, so vr_world is right-handed and z-up like base_link.
/// </summary>
public class RobotWorldFrame : MonoBehaviour
{
    public static RobotWorldFrame Instance { get; private set; }

    void Awake()
    {
        if (Instance != null && Instance != this)
            Debug.LogWarning("More than one RobotWorldFrame in the scene; using the latest.");
        Instance = this;
    }

    public PointMsg ToRosPoint(Vector3 worldPosition) =>
        transform.InverseTransformPoint(worldPosition).To<FLU>();

    public QuaternionMsg ToRosRotation(Quaternion worldRotation) =>
        (Quaternion.Inverse(transform.rotation) * worldRotation).To<FLU>();

    public PoseMsg ToRosPose(Transform t) =>
        new PoseMsg(ToRosPoint(t.position), ToRosRotation(t.rotation));

    public Vector3 FromRosPoint(PointMsg p) => transform.TransformPoint(p.From<FLU>());

    public Quaternion FromRosRotation(QuaternionMsg q) => transform.rotation * q.From<FLU>();

    /// <summary>Apply a ROS pose (in vr_world) to a Unity transform.</summary>
    public void Apply(PoseMsg pose, Transform target)
    {
        target.SetPositionAndRotation(FromRosPoint(pose.position), FromRosRotation(pose.orientation));
    }
}
