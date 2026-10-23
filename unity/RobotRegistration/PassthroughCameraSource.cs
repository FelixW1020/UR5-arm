using System;
using Meta.XR;
using UnityEngine;

/// <summary>
/// The only script that talks to Meta's Passthrough Camera API (PCA): MRUK's
/// PassthroughCameraAccess component, MRUK v81 or newer. A later SDK change is a fix here only.
///
/// What it hands PassthroughFrameStreamer, independent of the SDK's own conventions:
///  - Pose: the Unity WORLD pose of the camera when the latest image was captured, oriented like
///    a Unity Camera (forward = viewing direction, up = image up). PCA's GetCameraPose() is the
///    head pose at the image timestamp times the lens offset, already with that orientation, but
///    in the OVRCameraRig's TRACKING space (it matches world only while the rig sits at the
///    origin). It is mapped through cameraRig.trackingSpace here.
///  - Intrinsics: pinhole K for an upright image of any size, OpenCV convention (x right, y down,
///    origin at the centre of the top-left pixel). PCA's FocalLength/PrincipalPoint are in sensor
///    pixels, centre-cropped to CurrentResolution by private code, and its projection
///    (WorldToViewportPoint) uses a viewport with origin bottom-left and y up. Instead of copying
///    the crop logic or guessing the principal point's origin, K is read off WorldToViewportPoint
///    itself, so it always agrees with how Meta maps pixels to rays. (That mapping counts the
///    principal point's y from the bottom, unlike Android's Camera2 metadata; principalPointFromTop
///    is the escape hatch if a headset test shows Meta got it wrong.)
/// </summary>
public class PassthroughCameraSource : MonoBehaviour
{
    public const string CameraPermission = "horizonos.permission.HEADSET_CAMERA";

    [Tooltip("MRUK PassthroughCameraAccess (same GameObject if empty). Leave the component " +
             "DISABLED in the scene: it is switched on only while frames are streamed.")]
    public PassthroughCameraAccess cameraAccess;

    [Tooltip("Found automatically if empty. PCA poses are relative to its TrackingSpace.")]
    public OVRCameraRig cameraRig;

    [Tooltip("Request horizonos.permission.HEADSET_CAMERA when streaming starts. Untick if OVRManager " +
             "('request on startup') or another script already asks: Meta recommends requesting " +
             "permissions from one place only.")]
    public bool requestPermission = true;

    [Tooltip("Turn the camera off while not streaming (saves power).")]
    public bool stopCameraWhenIdle = true;

    [Tooltip("Diagnostic, normally off. Off = the principal point is where Meta's own projection " +
             "puts it (y counted from the image bottom). On = y counted from the top (Android " +
             "Camera2 convention). Only matters if the logged cy is far from the image centre; " +
             "see unity/README.md 'Verify the camera conventions'.")]
    public bool principalPointFromTop;

    bool? _supported;
    bool _requested, _warnedPose;
    Vector4 _viewport;            // viewport = (ax * x/z + bx, ay * y/z + by); (ax, bx, ay, by)
    Vector2Int _viewportResolution;

    void Awake()
    {
        if (cameraAccess == null) cameraAccess = GetComponent<PassthroughCameraAccess>();
        if (cameraRig == null) cameraRig = FindAnyObjectByType<OVRCameraRig>();
    }

    bool Supported => _supported ??= PassthroughCameraAccess.IsSupported;   // Quest 3/3S, Horizon OS v74+

    public static bool HasPermission
    {
        get
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            return UnityEngine.Android.Permission.HasUserAuthorizedPermission(CameraPermission);
#else
            return true;
#endif
        }
    }

    public void SetStreaming(bool on)
    {
        if (cameraAccess == null || !Supported) return;
        _viewportResolution = default;
        if (on)
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            if (requestPermission && !_requested && !HasPermission)
                UnityEngine.Android.Permission.RequestUserPermission(CameraPermission);
#endif
            _requested = true;
            cameraAccess.enabled = true;   // without the permission it waits for it, then starts
        }
        // Only stop it once it has the permission: while it waits, disabling it doesn't stop its
        // wait coroutine (MRUK v81-v207), which then starts the camera behind our back.
        else if (stopCameraWhenIdle && HasPermission)
        {
            cameraAccess.enabled = false;
        }
    }

    /// <summary>Null when images can be read; otherwise a message for the user.</summary>
    public string NotReadyReason
    {
        get
        {
            if (cameraAccess == null) return "No PassthroughCameraAccess component assigned.";
            if (!Supported) return "The Passthrough Camera API needs a Quest 3/3S on Horizon OS v74+.";
            if (!HasPermission)
                return _requested && requestPermission
                    ? "Waiting for the headset camera permission. If you denied it, allow it for " +
                      "this app in the headset's permission settings."
                    : "No headset camera permission (requestPermission is off: request it elsewhere).";
            if (!cameraAccess.enabled) return "PassthroughCameraAccess is off (it disables itself if it " +
                                              "fails to start; see logcat).";
            if (!cameraAccess.IsPlaying) return "Starting the passthrough camera...";
            return null;
        }
    }

    /// <summary>True in the frame in which a new camera image arrived.</summary>
    public bool HasNewImageThisFrame => cameraAccess.IsUpdatedThisFrame;

    /// <summary>The latest camera image. It displays upright in Unity (row 0 = image bottom).</summary>
    public Texture Texture => cameraAccess.GetTexture();

    /// <summary>Capture time of the latest image (headset wall clock, UTC).</summary>
    public DateTime ImageTimeUtc => cameraAccess.Timestamp;

    /// <summary>Unity world pose of the camera at ImageTimeUtc: forward = viewing direction, up = image up.</summary>
    public bool TryGetImagePose(out Pose world)
    {
        if (!TryGetTrackingSpacePose(out world)) return false;
        var space = cameraRig != null ? cameraRig.trackingSpace : null;
        if (space != null)
            world = new Pose(space.TransformPoint(world.position), space.rotation * world.rotation);

        var head = cameraRig != null ? cameraRig.centerEyeAnchor : null;
        if (!_warnedPose && head != null && Vector3.Distance(head.position, world.position) > 0.3f)
        {
            _warnedPose = true;
            Debug.LogWarning("[PassthroughCameraSource] The passthrough camera pose is " +
                             $"{Vector3.Distance(head.position, world.position):F2} m from the " +
                             "CenterEyeAnchor; the PCA pose space doesn't match this rig's tracking space.");
        }
        return true;
    }

    /// <summary>
    /// OpenCV pinhole intrinsics for the current image resampled (stretched) to width x height.
    /// The images carry no distortion model: PCA only exposes a pinhole model.
    /// </summary>
    public bool TryGetIntrinsics(int width, int height, out double fx, out double fy, out double cx, out double cy)
    {
        fx = fy = cx = cy = 0.0;
        var resolution = cameraAccess.CurrentResolution;
        if (resolution != _viewportResolution)
        {
            // WorldToViewportPoint projects with GetCameraPose(), i.e. in tracking space, so probe
            // it with tracking-space points at camera-local (x, y, 1). Its projection is affine in
            // (x/z, y/z), so two points give the whole mapping.
            if (!TryGetTrackingSpacePose(out var cam)) return false;
            Vector2 v0 = cameraAccess.WorldToViewportPoint(cam.position + cam.rotation * new Vector3(0f, 0f, 1f));
            Vector2 v1 = cameraAccess.WorldToViewportPoint(cam.position + cam.rotation * new Vector3(1f, 1f, 1f));
            Vector2 a = v1 - v0;
            if (!(a.x > 0f && a.y > 0f))
            {
                Debug.LogError($"[PassthroughCameraSource] Unexpected PCA projection (scale {a}); " +
                               "the SDK's viewport convention changed. Fix TryGetIntrinsics.");
                return false;
            }
            _viewport = new Vector4(a.x, v0.x, a.y, v0.y);
            _viewportResolution = resolution;
            Map(_viewport, principalPointFromTop, resolution.x, resolution.y, out fx, out fy, out cx, out cy);
            Debug.Log($"[PassthroughCameraSource] K for {resolution.x}x{resolution.y}: fx {fx:F1} fy {fy:F1} " +
                      $"cx {cx:F1} cy {cy:F1} (principal point {cx - (resolution.x - 1) / 2.0:F1}, " +
                      $"{cy - (resolution.y - 1) / 2.0:F1} px from the centre)");
        }
        Map(_viewport, principalPointFromTop, width, height, out fx, out fy, out cx, out cy);
        return true;
    }

    // Viewport (0..1, origin bottom-left, y up) -> OpenCV pixels for a W x H image whose row 0 is
    // the top: u = vx * W - 0.5, v = (1 - vy) * H - 0.5 (camera y up = optical y down).
    static void Map(Vector4 vp, bool fromTop, int width, int height,
                    out double fx, out double fy, out double cx, out double cy)
    {
        fx = vp.x * width;
        cx = vp.y * width - 0.5;
        fy = vp.z * height;
        cy = (fromTop ? vp.w : 1.0 - vp.w) * height - 0.5;
    }

    bool TryGetTrackingSpacePose(out Pose pose)
    {
        pose = cameraAccess.GetCameraPose();
        float n = Quaternion.Dot(pose.rotation, pose.rotation);   // default(Pose) when unavailable
        return Mathf.Abs(n - 1f) < 1e-3f;
    }
}
