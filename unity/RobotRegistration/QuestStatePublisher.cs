using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.VrInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Streams the headset and both Touch controllers to ROS on /quest/state (vr_interfaces/QuestState).
/// On the ROS side vr_tracking's quest_bridge turns this into TF (vr_world -> quest_head, ...),
/// PoseStamped topics in base_link and sensor_msgs/Joy, which is what the robot code consumes.
///
/// Every pose goes through RobotWorldFrame (vr_world, ROS FLU axes), so an anchor's Unity
/// forward becomes ROS x: quest_head x = gaze direction, controller x = pointing direction.
/// Inputs are read per controller (Controller.LTouch / RTouch): with an individual controller
/// OVRInput maps X/A to Button.One and Y/B to Button.Two (Three/Four only exist on the combined
/// Controller.Touch).
/// </summary>
public class QuestStatePublisher : MonoBehaviour
{
    [Tooltip("Found automatically if empty. Its CenterEyeAnchor and Left/RightControllerAnchor are sent.")]
    public OVRCameraRig cameraRig;
    public string topic = "/quest/state";
    [Range(1f, 120f)] public float rateHz = 30f;

    ROSConnection _ros;
    float _next;

    void Start()
    {
        if (cameraRig == null) cameraRig = FindAnyObjectByType<OVRCameraRig>();
        if (cameraRig == null) Debug.LogError("[QuestStatePublisher] No OVRCameraRig in the scene.");
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher(topic, QuestStateMsg.k_RosMessageName);
    }

    // LateUpdate: the rig has moved its anchors to this frame's tracking poses by now.
    void LateUpdate()
    {
        var frame = RobotWorldFrame.Instance;
        float now = Time.unscaledTime, period = 1f / Mathf.Max(rateHz, 0.1f);
        if (frame == null || cameraRig == null || now < _next) return;
        _next = now - _next > period ? now + period : _next + period;   // no burst after a stall

        // After a recenter vr_world has moved, so the poses would be wrong in base_link: send
        // them as untracked (buttons still go through) until the headset is registered again.
        bool valid = !frame.RegistrationStale;
        _ros.Publish(topic, new QuestStateMsg
        {
            header = new HeaderMsg { stamp = RosUtil.Stamp(DateTime.UtcNow), frame_id = RobotWorldFrame.RosFrameId },
            head_tracked = valid && OVRPlugin.GetNodePositionTracked(OVRPlugin.Node.Head),
            head = frame.ToRosPose(cameraRig.centerEyeAnchor),
            left = Controller(frame, cameraRig.leftControllerAnchor, OVRInput.Controller.LTouch, valid),
            right = Controller(frame, cameraRig.rightControllerAnchor, OVRInput.Controller.RTouch, valid),
        });
    }

    static ControllerStateMsg Controller(RobotWorldFrame frame, Transform anchor, OVRInput.Controller c, bool valid)
    {
        var stick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, c);
        return new ControllerStateMsg
        {
            tracked = valid && anchor != null && OVRInput.GetControllerPositionTracked(c),
            pose = anchor != null ? frame.ToRosPose(anchor) : new PoseMsg(),
            index_trigger = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, c),
            hand_trigger = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, c),
            thumbstick_x = stick.x,
            thumbstick_y = stick.y,
            button_primary = OVRInput.Get(OVRInput.Button.One, c),       // A / X
            button_secondary = OVRInput.Get(OVRInput.Button.Two, c),     // B / Y
            thumbstick_pressed = OVRInput.Get(OVRInput.Button.PrimaryThumbstick, c),
            // Only the left controller has a menu button (the right one's is reserved by the OS).
            menu = c == OVRInput.Controller.LTouch && OVRInput.Get(OVRInput.Button.Start, c),
        };
    }
}
