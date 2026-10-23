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
/// </summary>
public class ControllerFlangeRegistration : MonoBehaviour
{
    [Tooltip("RightControllerAnchor of the OVRCameraRig (the clamped controller).")]
    public Transform clampedController;

    public string addSampleService = "/vr_registration/add_controller_sample";
    public string solveService = "/vr_registration/solve_controller";
    public bool saveOnRos = true;
    public float serviceTimeoutS = 10f;
    public TextMesh status;
    public UnityEvent onRegistered;

    ROSConnection _ros;
    bool _waiting;

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterRosService<AddControllerSampleRequest, AddControllerSampleResponse>(addSampleService);
        _ros.RegisterRosService<SolveRegistrationRequest, SolveRegistrationResponse>(solveService);
        SetStatus("Move the arm, then X = add sample, Y = solve.");
    }

    void Update()
    {
        if (_waiting || RobotWorldFrame.Instance == null) return;
        if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.LTouch))          // X
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
        var req = new AddControllerSampleRequest(RobotWorldFrame.Instance.ToRosPose(clampedController));
        var r = await _ros.CallService<AddControllerSampleResponse>(addSampleService, req, serviceTimeoutS);
        _waiting = false;
        SetStatus(r != null ? r.message : $"No reply from {addSampleService}. Is vr.launch.py running?");
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
        if (r.success) onRegistered?.Invoke();
    }

    void SetStatus(string s)
    {
        Debug.Log($"[ControllerFlangeRegistration] {s}");
        if (status != null) status.text = s;
    }
}
