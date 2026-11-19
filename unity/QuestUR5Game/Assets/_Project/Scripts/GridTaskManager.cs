using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;
using Oculus.Interaction;

public class GridTaskManager : MonoBehaviour
{
    [Header("Scene references")]
    public Transform taskObjects;
    public BoxCollider finishMat;

    [Header("Placement rules")]
    [Min(0.1f)] public float settleSeconds = 0.5f;
    [Min(0.001f)] public float surfaceToleranceMeters = 0.01f;
    [Min(0.001f)] public float maxLinearSpeed = 0.05f;
    [Min(0.001f)] public float maxAngularSpeed = 0.5f;
    [Min(0.1f)] public float releaseGraceSeconds = 1.5f;

    [Header("Grid metadata - TaskObjects origin is the board center")]
    [Min(1)] public int gridRows = 10;
    [Min(1)] public int gridColumns = 10;
    [Min(0.001f)] public float cellSizeMeters = 0.1f;

    [Header("CSV")]
    public bool autoExportCsv = true;
    public string testDeviceLabel = "Meta XR Simulator";

    [Header("Runtime status - observe during Play")]
    [SerializeField] private int totalObjects;
    [SerializeField] private int completedCount;
    [SerializeField] private int totalPickCount;
    [SerializeField] private double elapsedSeconds;
    [SerializeField] private bool timerStarted;
    [SerializeField] private int totalInvalidReleases;
    [SerializeField] private string csvFolder;
    [SerializeField] private bool csvSaveSucceeded;

    public int TotalObjects => totalObjects;
    public int CompletedCount => completedCount;
    public int TotalPickCount => totalPickCount;
    public double ElapsedSeconds => elapsedSeconds;
    public bool TimerStarted => timerStarted;
    public int TotalInvalidReleases => totalInvalidReleases;
    public string CsvFolder => csvFolder;
    public bool CsvSaveSucceeded => csvSaveSucceeded;

    private sealed class BlockState
    {
        public string id;
        public Grabbable grab;
        public Rigidbody body;
        public BoxCollider box;
        public Vector3 startLocal;
        public Vector3 currentLocal;
        public int startRow;
        public int startColumn;
        public bool wasPicked;
        public bool heldLastFrame;
        public bool completed;
        public float stableTime;
        public int pickCount;
        public int releaseCount;
        public int validReleases;
        public int invalidReleases;
        public bool pendingRelease;
        public double releaseDeadline;
        public double firstPickSessionTime = -1d;
        public double firstValidSessionTime = -1d;
    }

    private readonly List<BlockState> blocks = new List<BlockState>();
    private bool initialized;
    private bool needsSave;
    private string sessionId;
    private string sceneName;
    private string deviceModel;
    private string runContext;
    private double sessionStartedAt;
    private double firstAllPlacedTaskTime = -1d;

    private void Start()
    {
        if (taskObjects == null || finishMat == null ||
            !finishMat.enabled || finishMat.isTrigger ||
            gridRows < 1 || gridColumns < 1 || cellSizeMeters <= 0f)
        {
            Debug.LogError("Check TaskObjects, solid FinishMat and grid dimensions.", this);
            enabled = false;
            return;
        }

        releaseGraceSeconds = Mathf.Max(releaseGraceSeconds, settleSeconds + 0.1f);
        blocks.Clear();
        var ids = new HashSet<string>();

        foreach (Transform child in taskObjects)
        {
            var grab = child.GetComponent<Grabbable>();
            var body = child.GetComponent<Rigidbody>();
            var box = child.GetComponent<BoxCollider>();
            if (!child.gameObject.activeInHierarchy || grab == null ||
                body == null || box == null || !box.enabled || box.isTrigger ||
                !ids.Add(child.name))
            {
                Debug.LogError("Each child needs a unique name, Grabbable, Rigidbody "
                    + "and solid BoxCollider: " + child.name, child);
                enabled = false;
                return;
            }

            Vector3 p = taskObjects.InverseTransformPoint(child.position);
            int column = Mathf.FloorToInt(
                (p.x + gridColumns * cellSizeMeters * 0.5f) / cellSizeMeters) + 1;
            int row = Mathf.FloorToInt(
                (p.z + gridRows * cellSizeMeters * 0.5f) / cellSizeMeters) + 1;
            if (column < 1 || column > gridColumns || row < 1 || row > gridRows)
            {
                row = column = 0;
                Debug.LogWarning(child.name + ": start is outside the configured grid; "
                    + "CSV row/column will be blank.", child);
            }

            blocks.Add(new BlockState {
                id = child.name, grab = grab, body = body, box = box,
                startLocal = p, currentLocal = p,
                startRow = row, startColumn = column
            });
        }

        totalObjects = blocks.Count;
        if (totalObjects == 0)
        {
            Debug.LogError("TaskObjects has no direct child blocks.", this);
            enabled = false;
            return;
        }

        completedCount = totalPickCount = totalInvalidReleases = 0;
        elapsedSeconds = 0d;
        timerStarted = false;
        csvSaveSucceeded = false;
        firstAllPlacedTaskTime = -1d;
        sessionStartedAt = Time.timeAsDouble;
        sessionId = DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fff",
            CultureInfo.InvariantCulture) + "_" + Guid.NewGuid().ToString("N").Substring(0, 8);
        sceneName = SceneManager.GetActiveScene().name;
        deviceModel = SystemInfo.deviceModel;
        runContext = Application.isEditor ? "editor" : Application.platform.ToString();
        csvFolder = Path.Combine(Application.persistentDataPath, "GridTaskResults", sessionId);
        initialized = true;
        Debug.Log("Grid task ready. CSV folder:\n" + csvFolder, this);
        ReportCount();
    }

    private void LateUpdate()
    {
        if (!initialized) return;
        if (timerStarted && completedCount < totalObjects)
            elapsedSeconds += Time.deltaTime;

        double now = Time.timeAsDouble;
        int count = 0;
        foreach (BlockState block in blocks)
        {
            if (block.grab == null || block.body == null || block.box == null ||
                !block.grab.gameObject.activeInHierarchy)
            {
                block.completed = false;
                block.stableTime = 0f;
                continue;
            }

            block.currentLocal = taskObjects.InverseTransformPoint(block.grab.transform.position);
            bool held = block.grab.GrabPoints.Count > 0;

            if (held && !block.heldLastFrame)
            {
                // An unresolved previous release becomes invalid exactly once.
                if (block.pendingRelease)
                    ResolveRelease(block, false, "picked up before valid placement");

                block.wasPicked = true;
                block.pickCount++;
                totalPickCount++;
                timerStarted = true;
                if (block.firstPickSessionTime < 0d)
                    block.firstPickSessionTime = now - sessionStartedAt;
            }
            else if (!held && block.heldLastFrame)
            {
                block.releaseCount++;
                block.pendingRelease = true;
                block.releaseDeadline = now + releaseGraceSeconds;
                needsSave = true;
            }
            block.heldLastFrame = held;

            bool eligible = block.wasPicked && !held &&
                block.box.enabled && !block.box.isTrigger &&
                !block.body.isKinematic && finishMat != null &&
                finishMat.enabled && !finishMat.isTrigger &&
                finishMat.gameObject.activeInHierarchy &&
                block.body.linearVelocity.sqrMagnitude <= maxLinearSpeed * maxLinearSpeed &&
                block.body.angularVelocity.sqrMagnitude <= maxAngularSpeed * maxAngularSpeed &&
                FitsOnMat(block.box);

            block.stableTime = eligible ? block.stableTime + Time.deltaTime : 0f;
            bool completeNow = eligible && block.stableTime >= settleSeconds;
            if (completeNow != block.completed)
            {
                block.completed = completeNow;
                needsSave = true;
                if (completeNow && block.firstValidSessionTime < 0d)
                    block.firstValidSessionTime = now - sessionStartedAt;
                Debug.Log(block.id + (completeNow
                    ? ": placement valid." : ": placement no longer valid."), this);
            }

            if (block.pendingRelease)
            {
                if (block.completed)
                    ResolveRelease(block, true, "stable on mat");
                else if (now >= block.releaseDeadline)
                    ResolveRelease(block, false, "placement window expired");
            }

            if (block.completed) count++;
        }

        if (count != completedCount)
        {
            completedCount = count;
            if (completedCount == totalObjects && firstAllPlacedTaskTime < 0d)
                firstAllPlacedTaskTime = elapsedSeconds;
            needsSave = true;
            ReportCount();
        }

        if (autoExportCsv && needsSave)
        {
            needsSave = false;
            SaveCsv("state_changed");
        }
    }

    private void ResolveRelease(BlockState block, bool valid, string reason)
    {
        block.pendingRelease = false;
        if (valid) block.validReleases++;
        else
        {
            block.invalidReleases++;
            totalInvalidReleases++;
            Debug.Log(block.id + ": invalid release (" + reason + "). Total: "
                + totalInvalidReleases, this);
        }
        needsSave = true;
    }

    private bool FitsOnMat(BoxCollider block)
    {
        Vector3 half = block.size * 0.5f;
        Vector3 matHalf = finishMat.size * 0.5f;
        Vector3 matTop = finishMat.transform.TransformPoint(
            finishMat.center + Vector3.up * matHalf.y);
        float lowestDistance = float.PositiveInfinity;

        for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
        for (int z = -1; z <= 1; z += 2)
        {
            Vector3 world = block.transform.TransformPoint(
                block.center + Vector3.Scale(half, new Vector3(x, y, z)));
            Vector3 local = finishMat.transform.InverseTransformPoint(world)
                - finishMat.center;
            if (Mathf.Abs(local.x) > matHalf.x || Mathf.Abs(local.z) > matHalf.z)
                return false;
            lowestDistance = Mathf.Min(lowestDistance,
                Vector3.Dot(world - matTop, finishMat.transform.up));
        }
        return Mathf.Abs(lowestDistance) <= surfaceToleranceMeters;
    }

    private void ReportCount()
    {
        Debug.Log("Completed: " + completedCount + "/" + totalObjects +
            (completedCount == totalObjects ? " - All blocks placed!" : ""), this);
    }

    [ContextMenu("Export CSV Snapshot (Play Mode)")]
    public void ExportCsvSnapshot()
    {
        if (!Application.isPlaying || !initialized)
        {
            Debug.LogWarning("Start Play mode before exporting a session.", this);
            return;
        }
        SaveCsv("manual_snapshot");
    }

    [ContextMenu("Show CSV Folder")]
    public void ShowCsvFolder()
    {
        if (string.IsNullOrEmpty(csvFolder))
        {
            Debug.LogWarning("Start a session first.", this);
            return;
        }
        Debug.Log("Grid CSV folder:\n" + csvFolder, this);
#if UNITY_EDITOR
        if (Directory.Exists(csvFolder))
            UnityEditor.EditorUtility.RevealInFinder(csvFolder);
#endif
    }

    private void SaveCsv(string reason)
    {
        if (!initialized) return;
        try
        {
            int releaseCount = 0, validCount = 0, pendingCount = 0, heldCount = 0;
            foreach (BlockState b in blocks)
            {
                releaseCount += b.releaseCount;
                validCount += b.validReleases;
                if (b.pendingRelease) pendingCount++;
                if (b.heldLastFrame) heldCount++;
            }

            string exportedUtc = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture);
            string status = !timerStarted ? "not_started" :
                completedCount == totalObjects ? "all_placed" : "incomplete";
            var summary = new StringBuilder();
            Row(summary, "session_id", "csv_schema_version", "exported_at_utc", "save_reason",
                "scene_name", "test_device_label", "run_context", "device_model",
                "unity_version", "session_status", "task_time_s", "session_game_time_s",
                "completed_count", "total_objects", "total_pick_count", "total_release_count",
                "valid_release_count", "invalid_release_count", "pending_release_count",
                "held_object_count", "first_all_placed_task_time_s", "robot_assistance_used",
                "grid_rows", "grid_columns", "cell_size_m", "release_grace_s",
                "settle_seconds", "surface_tolerance_m", "position_frame");
            Row(summary, sessionId, "1", exportedUtc, reason, sceneName, testDeviceLabel,
                runContext, deviceModel, Application.unityVersion, status, N(elapsedSeconds),
                N(Time.timeAsDouble - sessionStartedAt), I(completedCount), I(totalObjects),
                I(totalPickCount), I(releaseCount), I(validCount), I(totalInvalidReleases),
                I(pendingCount), I(heldCount), N(firstAllPlacedTaskTime), "not_recorded",
                I(gridRows), I(gridColumns), N(cellSizeMeters), N(releaseGraceSeconds),
                N(settleSeconds), N(surfaceToleranceMeters), "TaskObjects_local");

            var objects = new StringBuilder();
            Row(objects, "session_id", "object_id", "start_row", "start_column",
                "position_frame", "start_local_x_m", "start_local_y_m", "start_local_z_m",
                "current_local_x_m", "current_local_y_m", "current_local_z_m",
                "pick_count", "release_count", "valid_release_count", "invalid_release_count",
                "pending_release", "held_at_export", "completed_at_export",
                "first_pick_session_s", "first_valid_place_session_s");
            foreach (BlockState b in blocks)
                Row(objects, sessionId, b.id, b.startRow == 0 ? "" : I(b.startRow),
                    b.startColumn == 0 ? "" : I(b.startColumn), "TaskObjects_local",
                    Coordinate(b.startLocal.x), Coordinate(b.startLocal.y), Coordinate(b.startLocal.z),
                    Coordinate(b.currentLocal.x), Coordinate(b.currentLocal.y), Coordinate(b.currentLocal.z),
                    I(b.pickCount), I(b.releaseCount), I(b.validReleases), I(b.invalidReleases),
                    B(b.pendingRelease), B(b.heldLastFrame), B(b.completed),
                    N(b.firstPickSessionTime), N(b.firstValidSessionTime));

            Directory.CreateDirectory(csvFolder);
            var encoding = new UTF8Encoding(true);
            // Same session files are refreshed as snapshots, not appended.
            File.WriteAllText(Path.Combine(csvFolder, "objects.csv"), objects.ToString(), encoding);
            File.WriteAllText(Path.Combine(csvFolder, "summary.csv"), summary.ToString(), encoding);
            csvSaveSucceeded = true;
            Debug.Log("Grid CSV saved (" + reason + "):\n" + csvFolder, this);
        }
        catch (Exception e)
        {
            csvSaveSucceeded = false;
            Debug.LogError("Grid CSV save failed: " + e.Message +
                "\nClose CSV files in Excel and use Export CSV Snapshot to retry.\n"
                + csvFolder, this);
        }
    }

    private static string I(int value) => value.ToString(CultureInfo.InvariantCulture);
    private static string B(bool value) => value ? "true" : "false";
    private static string N(double value) =>
        value < 0d ? "" : value.ToString("F3", CultureInfo.InvariantCulture);
    private static string Coordinate(float value) =>
        value.ToString("F4", CultureInfo.InvariantCulture);

    private static void Row(StringBuilder output, params string[] cells)
    {
        for (int i = 0; i < cells.Length; i++)
        {
            if (i > 0) output.Append(',');
            output.Append('"').Append((cells[i] ?? "").Replace("\"", "\"\"")).Append('"');
        }
        output.Append("\r\n");
    }

    private void OnApplicationPause(bool paused)
    {
        if (paused && initialized && autoExportCsv) SaveCsv("app_paused");
    }

    private void OnApplicationQuit()
    {
        if (initialized && autoExportCsv) SaveCsv("application_quit");
    }
}

