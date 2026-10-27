using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Moves this GameObject to a PoseStamped published in vr_world, e.g.
///   /vr/robot_base_pose                      -> RobotBase, the parent of everything on the robot
///   /vr/object_localization/target_pose      -> the cup in the game
/// </summary>
public class RosPoseFollower : MonoBehaviour
{
    public string topic = "/vr/object_localization/target_pose";
    [Tooltip("Smoothing time constant in seconds (0 = snap).")]
    public float smoothing = 0.1f;
    public bool applyRotation = true;
    [Tooltip("Ignore poses until the headset is calibrated in this session (RobotWorldFrame.IsRegistered). " +
             "Without it the object first jumps to where the robot PC's last saved calibration puts it.")]
    public bool waitForRegistration;

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
        // The subscription outlives a destroyed follower (e.g. its scene was unloaded), and an
        // exception here would stop the other subscribers of this topic from getting the message.
        if (this == null) return;
        var frame = RobotWorldFrame.Instance;
        if (frame == null || (waitForRegistration && !frame.IsRegistered)) return;
        _targetPos = frame.FromRosPoint(msg.pose.position);
        _targetRot = frame.FromRosRotation(msg.pose.orientation);
        if (!_hasPose) transform.SetPositionAndRotation(_targetPos, applyRotation ? _targetRot : transform.rotation);
        _hasPose = true;
    }

    void Update()
    {
        if (!_hasPose) return;
        // Unscaled: keeps following while the game is paused with Time.timeScale = 0.
        float k = smoothing <= 0f ? 1f : 1f - Mathf.Exp(-Time.unscaledDeltaTime / smoothing);
        transform.position = Vector3.Lerp(transform.position, _targetPos, k);
        if (applyRotation) transform.rotation = Quaternion.Slerp(transform.rotation, _targetRot, k);
    }
}
