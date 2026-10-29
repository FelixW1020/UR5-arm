using RosMessageTypes.VrInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Most accurate registration: clamp the RIGHT controller rigidly to the UR5 tool flange,
/// move the arm through 10-15 varied poses (rotate the wrist a lot), and at each pose,
/// once the arm is still, press X on the LEFT controller. Press Y to solve.
/// Keep the clamped controller in view of the headset so it stays optically tracked.
///
/// X and Y are Button.One / Button.Two when the LEFT controller is named on its own
/// (Controller.LTouch); Button.Three / Four only exist on the combined Controller.Touch.
/// A headset recenter clears the samples taken before it (they are in the old vr_world).
/// </summary>
public class ControllerFlangeRegistration : MonoBehaviour
{
    [Tooltip("RightControllerAnchor of the OVRCameraRig (the clamped controller).")]
    public Transform clampedController;

    public string addSampleService = "/vr_registration/add_controller_sample";
    public string solveService = "/vr_registration/solve_controller";
    public string resetService = "/vr_registration/reset_controller_samples";
    public bool saveOnRos = true;
    public float serviceTimeoutS = 10f;
    public TextMesh status;
    public UnityEvent onRegistered = new UnityEvent();

    ROSConnection _ros;
    bool _waiting;
    int _samples;                         // added since the last reset
    int _generation;                      // RobotWorldFrame.Generation of those samples

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterRosService(addSampleService, AddControllerSampleRequest.k_RosMessageName,
                                AddControllerSampleResponse.k_RosMessageName);
        _ros.RegisterRosService(solveService, SolveRegistrationRequest.k_RosMessageName,
                                SolveRegistrationResponse.k_RosMessageName);
        TriggerResult.Register();
        _ros.RegisterRosService(resetService, TriggerResult.k_RosMessageName, TriggerResult.k_RosMessageName);
        SetStatus("Move the arm, then X = add sample, Y = solve.");
    }

    void Update()
    {
        if (_waiting || RobotWorldFrame.Instance == null) return;
        if (_samples > 0 && RobotWorldFrame.CurrentGeneration != _generation)
            ResetSamples();
        else if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.LTouch))     // X
            AddSample();
        else if (OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.LTouch))     // Y
            Solve();
    }

    async void AddSample()
    {
        if (!OVRInput.GetControllerPositionTracked(OVRInput.Controller.RTouch))
        {
            SetStatus("Clamped controller is not tracked; look at it and try again.");
            return;
        }
        _waiting = true;
        int generation = RobotWorldFrame.CurrentGeneration;
        var req = new AddControllerSampleRequest(RobotWorldFrame.Instance.ToRosPose(clampedController));
        var r = await _ros.CallService<AddControllerSampleResponse>(addSampleService, req, serviceTimeoutS);
        _waiting = false;
        SetStatus(r != null ? r.message : $"No reply from {addSampleService}. Is vr.launch.py running?");
        if (r == null || !r.success) return;
        if (_samples == 0) _generation = generation;
        _samples = (int)r.num_samples;
    }

    async void ResetSamples()
    {
        _waiting = true;
        var r = await _ros.CallService<TriggerResult>(resetService, new EmptyRequest(TriggerResult.k_RosMessageName),
                                                      serviceTimeoutS);
        _waiting = false;
        if (r == null) { SetStatus($"No reply from {resetService}. Is vr.launch.py running?"); return; }
        _samples = 0;
        SetStatus("Headset recentered: the samples so far were cleared. Start again with X.");
    }

    async void Solve()
    {
        _waiting = true;
        SetStatus("Solving...");
        var r = await _ros.CallService<SolveRegistrationResponse>(
            solveService, new SolveRegistrationRequest(saveOnRos), serviceTimeoutS);
        _waiting = false;
        if (r == null) { SetStatus($"No reply from {solveService}. Is vr.launch.py running?"); return; }
        SetStatus(r.message);
        if (!r.success) return;
        RobotWorldFrame.Registered(_generation);
        onRegistered?.Invoke();
    }

    void SetStatus(string s)
    {
        Debug.Log($"[ControllerFlangeRegistration] {s}");
        if (status != null) status.text = s;
    }
}
