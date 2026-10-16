using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Moves this GameObject to a PoseStamped published in vr_world, e.g.
///   /vr/robot_base_pose                      -> root of the imported UR5 model
///   /vr/object_localization/target_pose      -> the cup in the game
/// </summary>
public class RosPoseFollower : MonoBehaviour
{
    public string topic = "/vr/object_localization/target_pose";
    [Tooltip("Smoothing time constant in seconds (0 = snap).")]
    public float smoothing = 0.1f;
    public bool applyRotation = true;

    Vector3 _targetPos;
    Quaternion _targetRot;
    bool _hasPose;

    void Start()
    {
        _targetRot = transform.rotation;
        ROSConnection.GetOrCreateInstance().Subscribe<PoseStampedMsg>(topic, OnPose);
    }

    void OnPose(PoseStampedMsg msg)
    {
        var frame = RobotWorldFrame.Instance;
        if (frame == null) return;
        _targetPos = frame.FromRosPoint(msg.pose.position);
        _targetRot = frame.FromRosRotation(msg.pose.orientation);
        if (!_hasPose) transform.SetPositionAndRotation(_targetPos, applyRotation ? _targetRot : transform.rotation);
        _hasPose = true;
    }

    void Update()
    {
        if (!_hasPose) return;
        float k = smoothing <= 0f ? 1f : 1f - Mathf.Exp(-Time.deltaTime / smoothing);
        transform.position = Vector3.Lerp(transform.position, _targetPos, k);
        if (applyRotation) transform.rotation = Quaternion.Slerp(transform.rotation, _targetRot, k);
    }
}
