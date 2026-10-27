using System;
using System.Threading.Tasks;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.VrInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;

/// <summary>
/// While enabled, sends passthrough camera frames to ROS on /quest/passthrough
/// (vr_interfaces/PassthroughFrame) for vr_tag_detector, which finds the ArUco tags and
/// registers vr_world to the robot. Each message is self-contained:
///   image        JPEG, upright (row 0 = top of the picture)
///   camera_info  K for exactly that JPEG's size, OpenCV pixel convention (origin top-left)
///   camera_pose  camera pose in vr_world at the image's capture time, "link" convention
///                (x = viewing direction, y = image-left, z = image-up)
/// The camera itself is reached only through PassthroughCameraSource.
///
/// Pixels and pose must describe the same instant, so a frame is read one Unity frame after it
/// arrives (a GPU copy issued in the arrival frame can still see the previous image, see
/// PassthroughCameraAccess.GetTexture()), and frames are skipped while the head turns or moves
/// faster than the limits below. JPEG encoding runs on a worker thread.
/// </summary>
public class PassthroughFrameStreamer : MonoBehaviour
{
    [Tooltip("Same GameObject if empty.")]
    public PassthroughCameraSource cameraSource;
    public string topic = "/quest/passthrough";
    [Range(0.5f, 10f)] public float rateHz = 3f;
    [Range(10, 100)] public int jpegQuality = 90;
    [Tooltip("Resize before sending (intrinsics are scaled to match). 1 = full camera resolution.")]
    [Range(0.25f, 1f)] public float downscale = 1f;
    [Tooltip("Leave off unless /vr_tag_detector/debug_image shows the picture upside down " +
             "(only the pixels are flipped; K and the pose already describe the upright image).")]
    public bool flipVertically;
    [Tooltip("Skip frames while the camera turns faster than this (deg/s).")]
    public float maxAngularSpeedDegS = 10f;
    [Tooltip("Skip frames while the camera moves faster than this (m/s).")]
    public float maxLinearSpeedMS = 0.1f;

    public int FramesSent { get; private set; }
    public string Status { get; private set; } = "Not streaming.";

    const string OpticalFrameId = "quest_camera_optical";   // informational, nested headers only
    const int MaxWaitFrames = 8;    // camera updating every display frame: stop waiting for a gap

    ROSConnection _ros;
    RenderTexture _target;
    Pose _pose;
    DateTime _poseTime, _sentTime;
    bool _hasPose, _moving, _inFlight;
    int _newImageStreak;
    float _nextCapture;

    void Awake()
    {
        if (cameraSource == null) cameraSource = GetComponent<PassthroughCameraSource>();
    }

    void OnEnable()
    {
        if (_ros == null)
        {
            _ros = ROSConnection.GetOrCreateInstance();
            // By name: in a player build the generated Register() runs after the first scene's
            // OnEnable, so RegisterPublisher<T> here would announce the topic with no type.
            _ros.RegisterPublisher(topic, PassthroughFrameMsg.k_RosMessageName);
        }
        _hasPose = _moving = false;
        if (cameraSource == null) cameraSource = GetComponent<PassthroughCameraSource>();   // before Awake
        if (cameraSource != null) cameraSource.SetStreaming(true);
        Status = "Starting...";
    }

    void OnDisable()
    {
        if (cameraSource != null) cameraSource.SetStreaming(false);
        Status = "Not streaming.";
    }

    void OnDestroy()
    {
        if (_target != null) { _target.Release(); Destroy(_target); }
    }

    void LateUpdate()
    {
        if (cameraSource == null) { Status = "No PassthroughCameraSource."; return; }
        string notReady = cameraSource.NotReadyReason;
        if (notReady != null) { Status = notReady; _hasPose = false; return; }
        if (RobotWorldFrame.Instance == null) { Status = "No RobotWorldFrame in the scene."; return; }

        if (cameraSource.HasNewImageThisFrame)
        {
            if (!cameraSource.TryGetImagePose(out var pose)) return;
            var time = cameraSource.ImageTimeUtc;
            double dt = (time - _poseTime).TotalSeconds;
            if (_hasPose && dt > 1e-4)
                _moving = Quaternion.Angle(_pose.rotation, pose.rotation) / dt > maxAngularSpeedDegS
                          || Vector3.Distance(_pose.position, pose.position) / dt > maxLinearSpeedMS;
            _pose = pose;
            _poseTime = time;
            _hasPose = true;
            if (++_newImageStreak <= MaxWaitFrames) return;
        }
        else
        {
            _newImageStreak = 0;
        }

        if (!_hasPose || _inFlight || Time.unscaledTime < _nextCapture || _poseTime == _sentTime) return;
        if (_moving)
        {
            Status = "Hold still: the headset is moving too fast for a sharp, well-posed frame.";
            return;
        }
        Capture();
    }

    void Capture()
    {
        var source = cameraSource.Texture;
        if (source == null) return;
        int w = Mathf.Max(32, Mathf.RoundToInt(source.width * downscale));
        int h = Mathf.Max(32, Mathf.RoundToInt(source.height * downscale));
        if (!cameraSource.TryGetIntrinsics(w, h, out double fx, out double fy, out double cx, out double cy))
        {
            Status = "Camera intrinsics are not available yet.";
            return;
        }

        if (_target == null || _target.width != w || _target.height != h)
        {
            if (_target != null) { _target.Release(); Destroy(_target); }
            // Same sRGB-ness as the camera texture, so the copy keeps the bytes as they are.
            var format = QualitySettings.activeColorSpace == ColorSpace.Linear
                ? GraphicsFormat.R8G8B8A8_SRGB : GraphicsFormat.R8G8B8A8_UNorm;
            _target = new RenderTexture(w, h, 0, format) { useMipMap = false, autoGenerateMips = false };
            _target.Create();
        }
        // Copy (and resize) so the next camera image can't race the readback. Blit leaves its
        // destination active; restore whatever was.
        var active = RenderTexture.active;
        Graphics.Blit(source, _target);
        RenderTexture.active = active;

        var stamp = RosUtil.Stamp(_poseTime == default ? DateTime.UtcNow : _poseTime);
        var info = new CameraInfoMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = OpticalFrameId },
            height = (uint)h,
            width = (uint)w,
            distortion_model = "plumb_bob",
            d = new double[5],
            k = new[] { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 },
            r = new[] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },
            p = new[] { fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0 },
        };
        var cameraPose = RobotWorldFrame.Instance.ToRosPose(_pose.position, _pose.rotation);

        _inFlight = true;
        _sentTime = _poseTime;
        _nextCapture = Time.unscaledTime + 1f / Mathf.Max(rateHz, 0.1f);
        AsyncGPUReadback.Request(_target, 0, request => OnReadback(request, w, h, stamp, info, cameraPose));
    }

    async void OnReadback(AsyncGPUReadbackRequest request, int w, int h, TimeMsg stamp,
                          CameraInfoMsg info, PoseMsg cameraPose)
    {
        try
        {
            if (request.hasError) { Status = "GPU readback failed."; return; }
            var rgba = request.GetData<byte>().ToArray();   // only valid inside this callback
            if (rgba.Length != w * h * 4) { Status = $"Unexpected readback size {rgba.Length}."; return; }
            bool flip = flipVertically;
            int quality = jpegQuality;
            var jpeg = await Task.Run(() => EncodeJpeg(rgba, w, h, flip, quality));
            if (!isActiveAndEnabled) return;

            _ros.Publish(topic, new PassthroughFrameMsg
            {
                header = new HeaderMsg { stamp = stamp, frame_id = RobotWorldFrame.RosFrameId },
                image = new CompressedImageMsg(new HeaderMsg { stamp = stamp, frame_id = OpticalFrameId },
                                               "jpeg", jpeg),
                camera_info = info,
                camera_pose = cameraPose,
            });
            FramesSent++;
            Status = $"Streaming: {FramesSent} frames sent ({w}x{h}, {jpeg.Length / 1024} kB).";
        }
        catch (Exception e)
        {
            Debug.LogException(e);
            Status = $"Frame not sent: {e.Message}";
        }
        finally
        {
            _inFlight = false;
        }
    }

    // Runs on a worker thread (ImageConversion is thread safe). The readback's row 0 is the
    // bottom of the picture, as for any Unity texture, and the encoder writes it last, like
    // Texture2D.EncodeToJPG: the JPEG comes out upright.
    static byte[] EncodeJpeg(byte[] rgba, int w, int h, bool flip, int quality)
    {
        if (flip)
        {
            int stride = w * 4;
            var row = new byte[stride];
            for (int top = 0, bottom = h - 1; top < bottom; top++, bottom--)
            {
                Buffer.BlockCopy(rgba, top * stride, row, 0, stride);
                Buffer.BlockCopy(rgba, bottom * stride, rgba, top * stride, stride);
                Buffer.BlockCopy(row, 0, rgba, bottom * stride, stride);
            }
        }
        // UNorm: the bytes are already display (sRGB) values; no conversion wanted.
        return ImageConversion.EncodeArrayToJPG(rgba, GraphicsFormat.R8G8B8A8_UNorm, (uint)w, (uint)h, 0, quality);
    }
}
