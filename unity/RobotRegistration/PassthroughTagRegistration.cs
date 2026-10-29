using RosMessageTypes.VrInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Register vr_world to the robot by LOOKING at the ArUco tags: no touching, no clamping.
/// The headset's passthrough camera streams to vr_tag_detector, which poses each tag in vr_world
/// and fuses the sightings; vr_registration pairs them with the robot's tag map. One tag is
/// enough (its orientation fixes the heading); 2-4 tags spread over the table are better.
///
///   Enable this component  : frames start streaming (old sightings are cleared)
///   Look at each tag from 0.4-1.2 m and hold your head still for ~2 s per tag
///   A                      : register (/vr_tag_detector/register, saved on the robot PC)
///   B                      : clear the sightings and stream again
///
/// A and B are also TagTouchRegistration's buttons: enable only one registration script at a time.
/// In a game, untick useControllerButtons to keep A/B for gameplay and call Register() and
/// ResetSightings() from your own UI instead.
/// After a headset recenter the old sightings are cleared automatically: look at the tags again.
/// </summary>
public class PassthroughTagRegistration : MonoBehaviour
{
    [Tooltip("Same GameObject if empty. Enabled while this component is.")]
    public PassthroughFrameStreamer streamer;

    public string registerService = "/vr_tag_detector/register";
    public string resetService = "/vr_tag_detector/reset";
    public bool saveOnRos = true;
    public bool resetWhenStarting = true;
    [Tooltip("Stop the camera once registered (B starts it again).")]
    public bool stopStreamingWhenRegistered = true;

    [Tooltip("Off: A/B do nothing here; your own UI calls Register() and ResetSightings().")]
    public bool useControllerButtons = true;
    public OVRInput.Controller controller = OVRInput.Controller.RTouch;
    public OVRInput.Button registerButton = OVRInput.Button.One;    // A on RTouch
    public OVRInput.Button resetButton = OVRInput.Button.Two;       // B on RTouch
    [Tooltip("Seconds to wait for a reply (vr_tag_detector itself waits up to ~12 s for vr_registration).")]
    public float serviceTimeoutS = 20f;

    public TextMesh status;               // optional world-space label
    public UnityEvent onRegistered = new UnityEvent();     // e.g. TagMapVisualizer.Refresh

    /// <summary>True while a register or reset call is waiting for ROS (Register() is ignored).</summary>
    public bool IsBusy => _busy;

    /// <summary>What the status label shows: camera state, instructions, last reply.</summary>
    public string StatusText { get; private set; } = "";

    ROSConnection _ros;
    bool _busy;
    string _reply = "";
    float _nextStatus;
    int _generation;                      // RobotWorldFrame.Generation of the current sightings

    void Awake()
    {
        if (streamer == null) streamer = GetComponent<PassthroughFrameStreamer>();
    }

    void EnsureRos()
    {
        if (_ros != null) return;
        _ros = ROSConnection.GetOrCreateInstance();
        // By name: in a player build the generated Register() runs after the first scene's
        // OnEnable, so the generic overload would announce the service with no type.
        _ros.RegisterRosService(registerService, SolveRegistrationRequest.k_RosMessageName,
                                SolveRegistrationResponse.k_RosMessageName);
        TriggerResult.Register();
        _ros.RegisterRosService(resetService, TriggerResult.k_RosMessageName, TriggerResult.k_RosMessageName);
    }

    void OnEnable()
    {
        EnsureRos();
        _reply = "";
        StartStreaming();
        if (resetWhenStarting) Clear(blockButtons: false);   // don't lock A if ROS isn't up yet
    }

    void OnDisable()
    {
        if (streamer != null) streamer.enabled = false;
    }

    void Update()
    {
        if (!_busy)
        {
            // Sightings from before a recenter are in the old vr_world: start over.
            if (RobotWorldFrame.CurrentGeneration != _generation) ResetSightings();
            else if (useControllerButtons && OVRInput.GetDown(registerButton, controller)) Register();
            else if (useControllerButtons && OVRInput.GetDown(resetButton, controller)) ResetSightings();
        }
        if (Time.unscaledTime >= _nextStatus) ShowStatus();
    }

    void StartStreaming()
    {
        if (streamer != null) streamer.enabled = true;
        else _reply = "No PassthroughFrameStreamer assigned.";
    }

    public async void Register()
    {
        if (_busy) return;
        EnsureRos();
        int generation = _generation;
        _busy = true;
        _reply = "Registering...";
        ShowStatus();
        var r = await _ros.CallService<SolveRegistrationResponse>(
            registerService, new SolveRegistrationRequest(saveOnRos), serviceTimeoutS);
        _busy = false;
        if (r == null)
        {
            _reply = $"No reply from {registerService}. Is vr_tag_detector running?";
        }
        else
        {
            _reply = r.message;
            if (r.success)
            {
                if (stopStreamingWhenRegistered && streamer != null) streamer.enabled = false;
                RobotWorldFrame.Registered(generation);
                onRegistered?.Invoke();
            }
        }
        Debug.Log($"[PassthroughTagRegistration] {_reply}");
        ShowStatus();
    }

    /// <summary>Forget everything vr_tag_detector has seen so far and (re)start streaming.</summary>
    public void ResetSightings()
    {
        StartStreaming();
        Clear(blockButtons: true);
    }

    async void Clear(bool blockButtons)
    {
        EnsureRos();
        _generation = RobotWorldFrame.CurrentGeneration;
        _busy |= blockButtons;
        var r = await _ros.CallService<TriggerResult>(resetService, new EmptyRequest(TriggerResult.k_RosMessageName),
                                                      Mathf.Min(serviceTimeoutS, 5f));
        if (blockButtons) _busy = false;
        _reply = r == null ? $"No reply from {resetService}. Is vr_tag_detector running?"
                           : "Sightings cleared.";
        ShowStatus();
    }

    void ShowStatus()
    {
        _nextStatus = Time.unscaledTime + 0.25f;
        bool streaming = streamer != null && streamer.isActiveAndEnabled;
        string camera = streaming ? streamer.Status : "Camera off.";
        string keys = useControllerButtons ? " A = register, B = start over." : "";
        string help = streaming ? "Look at the tag(s) from 0.4-1.2 m, hold still ~2 s each." + keys
                                : keys.TrimStart();
        string text = $"{camera}\n{help}" + (_reply.Length > 0 ? $"\n{_reply}" : "");
        if (RobotWorldFrame.Instance != null && RobotWorldFrame.Instance.RegistrationStale)
            text = "Headset recentered: the robot registration no longer holds. Register again.\n" + text;
        StatusText = text;
        if (status != null) status.text = text;
    }
}
