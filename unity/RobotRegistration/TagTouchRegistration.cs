using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.VrInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Register vr_world to the robot by touching each ArUco tag's centre with the controller tip.
///
///   Right index trigger : record the tip position for the current tag id, go to the next
///   B                   : undo the last touch
///   A                   : send all touches to /vr_registration/register_tags (and save)
///
/// Touches are positions only (has_orientation = false), so at least 2 tags are needed (3+
/// spread out recommended). Other tag sources can call AddObservation() and Submit() directly;
/// to register by just looking at the tags, use PassthroughTagRegistration instead.
/// A headset recenter discards the touches made before it.
/// </summary>
public class TagTouchRegistration : MonoBehaviour
{
    [Tooltip("Child of RightControllerAnchor at the physical tip of the controller. " +
             "Put a small sphere on it and adjust localPosition in passthrough until it " +
             "sits exactly on the tip.")]
    public Transform controllerTip;

    [Tooltip("Tag ids in the order you will touch them (must exist in the ROS tag map).")]
    public int[] tagOrder = { 0, 1, 2, 3, 4, 5 };

    public string serviceName = "/vr_registration/register_tags";
    public bool saveOnRos = true;
    public float serviceTimeoutS = 10f;
    public TextMesh status;               // optional world-space label
    public UnityEvent onRegistered = new UnityEvent();     // e.g. TagMapVisualizer.Refresh

    readonly List<(int id, Vector3 world)> _touches = new List<(int, Vector3)>();
    ROSConnection _ros;
    bool _waiting;
    int _generation;                      // RobotWorldFrame.Generation of the touches

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterRosService(serviceName, RegisterTagsRequest.k_RosMessageName,
                                RegisterTagsResponse.k_RosMessageName);
        ShowNext();
    }

    void Update()
    {
        if (_waiting) return;
        if (_touches.Count > 0 && RobotWorldFrame.CurrentGeneration != _generation)
        {
            _touches.Clear();     // touched in the old vr_world
            SetStatus($"Headset recentered: touch the tags again, starting with tag {tagOrder[0]}.");
        }
        if (OVRInput.GetDown(OVRInput.Button.PrimaryIndexTrigger, OVRInput.Controller.RTouch))
        {
            if (_touches.Count < tagOrder.Length)
                AddObservation(tagOrder[_touches.Count], controllerTip.position);
            ShowNext();
        }
        else if (OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.RTouch) && _touches.Count > 0)
        {
            _touches.RemoveAt(_touches.Count - 1);
            ShowNext();
        }
        else if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.RTouch))
        {
            Submit();
        }
    }

    public void AddObservation(int tagId, Vector3 worldPosition)
    {
        if (_touches.Count == 0) _generation = RobotWorldFrame.CurrentGeneration;
        _touches.Add((tagId, worldPosition));
    }

    public async void Submit()
    {
        var frame = RobotWorldFrame.Instance;
        if (frame == null) { SetStatus("No RobotWorldFrame in scene."); return; }
        if (_touches.Count < 2) { SetStatus("Touch at least 2 tags (3+ recommended)."); return; }

        // A touch gives the tag centre only: no orientation (identity, ignored by ROS).
        var obs = _touches.Select(t => new TagObservationMsg(
            t.id, frame.ToRosPoint(t.world), false, new QuaternionMsg(0, 0, 0, 1))).ToArray();
        int generation = _generation;
        _waiting = true;
        SetStatus($"Registering with {obs.Length} tags...");
        var r = await _ros.CallService<RegisterTagsResponse>(
            serviceName, new RegisterTagsRequest(obs, saveOnRos), serviceTimeoutS);
        _waiting = false;
        if (r == null) { SetStatus($"No reply from {serviceName}. Is vr.launch.py running?"); return; }
        SetStatus(r.message);
        if (!r.success) return;
        _touches.Clear();
        RobotWorldFrame.Registered(generation);
        onRegistered?.Invoke();
    }

    void ShowNext()
    {
        SetStatus(_touches.Count < tagOrder.Length
            ? $"Touch tag {tagOrder[_touches.Count]} ({_touches.Count}/{tagOrder.Length}). A = register, B = undo"
            : $"All {tagOrder.Length} tags touched. Press A to register.");
    }

    void SetStatus(string s)
    {
        Debug.Log($"[TagTouchRegistration] {s}");
        if (status != null) status.text = s;
    }
}
