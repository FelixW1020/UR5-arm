using RosMessageTypes.VrInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Most accurate registration: clamp the RIGHT controller rigidly to the UR5 tool flange,
/// move the arm through 10-15 varied poses (rotate the wrist a lot), and at each pose,
/// once the arm is still, press X on the LEFT controller. Press Y to solve.
/// Keep the clamped controller in view of the headset so it stays optically tracked.
/// </summary>
public class ControllerFlangeRegistration : MonoBehaviour
{
    [Tooltip("RightControllerAnchor of the OVRCameraRig (the clamped controller).")]
    public Transform clampedController;

    public string addSampleService = "/vr_registration/add_controller_sample";
    public string solveService = "/vr_registration/solve_controller";
    public bool saveOnRos = true;
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
        if (OVRInput.GetDown(OVRInput.Button.Three, OVRInput.Controller.LTouch))        // X
        {
            if (!OVRInput.GetControllerPositionTracked(OVRInput.Controller.RTouch))
            {
                SetStatus("Clamped controller is not tracked; look at it and try again.");
                return;
            }
            _waiting = true;
            var req = new AddControllerSampleRequest(RobotWorldFrame.Instance.ToRosPose(clampedController));
            _ros.SendServiceMessage<AddControllerSampleResponse>(addSampleService, req, r =>
            {
                _waiting = false;
                SetStatus(r.message);
            });
        }
        else if (OVRInput.GetDown(OVRInput.Button.Four, OVRInput.Controller.LTouch))   // Y
        {
            _waiting = true;
            SetStatus("Solving...");
            _ros.SendServiceMessage<SolveRegistrationResponse>(solveService,
                new SolveRegistrationRequest(saveOnRos), r =>
                {
                    _waiting = false;
                    SetStatus(r.message);
                    if (r.success) onRegistered?.Invoke();
                });
        }
    }

    void SetStatus(string s)
    {
        Debug.Log($"[ControllerFlangeRegistration] {s}");
        if (status != null) status.text = s;
    }
}
