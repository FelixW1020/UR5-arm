using System.Collections.Generic;
using System.Linq;
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
/// Other tag sources (e.g. a Passthrough Camera API ArUco detector) can call
/// AddObservation() and Submit() directly instead of using the controller.
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
    public TextMesh status;               // optional world-space label
    public UnityEvent onRegistered;       // e.g. TagMapVisualizer.Refresh

    readonly List<(int id, Vector3 world)> _touches = new List<(int, Vector3)>();
    ROSConnection _ros;
    bool _waiting;

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterRosService<RegisterTagsRequest, RegisterTagsResponse>(serviceName);
        ShowNext();
    }

    void Update()
    {
        if (_waiting) return;
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

    public void AddObservation(int tagId, Vector3 worldPosition) => _touches.Add((tagId, worldPosition));

    public void Submit()
    {
        var frame = RobotWorldFrame.Instance;
        if (frame == null) { SetStatus("No RobotWorldFrame in scene."); return; }
        if (_touches.Count < 2) { SetStatus("Touch at least 2 tags (3+ recommended)."); return; }

        var obs = _touches.Select(t => new TagObservationMsg(t.id, frame.ToRosPoint(t.world))).ToArray();
        _waiting = true;
        SetStatus($"Registering with {obs.Length} tags...");
        _ros.SendServiceMessage<RegisterTagsResponse>(
            serviceName, new RegisterTagsRequest(obs, saveOnRos), OnResponse);
    }

    void OnResponse(RegisterTagsResponse r)
    {
        _waiting = false;
        SetStatus(r.message);
        if (!r.success) return;
        _touches.Clear();
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
