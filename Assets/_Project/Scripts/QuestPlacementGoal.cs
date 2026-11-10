using UnityEngine;
using TMPro;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.Globalization;

// Lesson prototype: assisted rounds accept a release inside the target box;
// optional automatic rounds bypass human placement using a scripted plan.
// Transport is visual only; there is no perception or real robot control.
public class QuestPlacementGoal : MonoBehaviour
{
    [SerializeField] private Transform targetCube;
    [SerializeField] private BoxCollider targetZone;
    [SerializeField] private Renderer indicator;
    [SerializeField] private Color waitingColor = new Color(1f, 0.65f, 0.1f);
    [SerializeField] private Color successColor = new Color(0.15f, 0.85f, 0.25f);

    [Header("Practice loop")]
    [SerializeField] private TMP_Text statusText;
    [SerializeField] private bool autoReset = true;
    [SerializeField, Min(1f)] private float resetDelay = 3f;
    [SerializeField, Range(1, 10)] private int totalRounds = 3;

    [Header("Simulated robot assistance (optional)")]
    [SerializeField] private bool simulateRobotAssistance = false;
    [SerializeField, Min(0.5f)] private float robotCheckDelay = 2f;
    [SerializeField] private Color checkingColor = new Color(0.2f, 0.6f, 1f);

    [Header("Simulated transport (optional)")]
    [SerializeField] private bool simulateRobotTransport = false;
    [SerializeField] private Transform robotTool;
    [SerializeField] private Transform robotArm;
    [SerializeField] private Renderer robotColumn;
    [SerializeField] private Renderer collectionPad;
    [SerializeField, Min(0.05f)] private float transportSpeed = 0.35f;
    [SerializeField, Min(0.05f)] private float liftHeight = 0.2f;

    public enum RoundTask
    {
        AutoTransport,
        HumanAssistance
    }

    [Header("Mixed round plan (optional, scripted)")]
    [SerializeField] private bool useMixedRounds = false;
    [SerializeField] private RoundTask[] roundPlan =
    {
        RoundTask.AutoTransport,
        RoundTask.HumanAssistance,
        RoundTask.AutoTransport
    };
    private bool mixedMode;
    private RoundTask[] sessionPlan;
    private RoundTask currentRoundTask;
    private bool IsAutomaticRound => mixedMode && currentRoundTask == RoundTask.AutoTransport;

    public enum AssistanceState
    {
        PracticeOnly,
        Checking,
        WaitingForHuman,
        HelpReceived,
        RobotTransporting,
        DeliveryComplete
    }

    // Runtime display for learning; do not edit this field during Play.
    [SerializeField] private AssistanceState assistanceState;
    private bool assistanceMode;
    private float checkRemaining;

    [System.Serializable]
    public class RoundResult
    {
        public int round;
        public double seconds;
        public int outsideReleases;
        public int manualRecoveries;
        public bool transportComplete;
        public double transportSeconds;
        public bool humanRequired = true;
        public string roundType;
    }

    [Header("Results from this Play session")]
    [SerializeField] private List<RoundResult> results = new List<RoundResult>();

    private MaterialPropertyBlock colors;
    private int selectingCount;
    private bool completed;
    private bool ready;
    private int successCount;
    private float resetAt;
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private Rigidbody cubeBody;
    private string lastMessage;
    private bool sessionFinished;
    private bool timerStarted;
    private double roundStartedAt;
    private int outsideReleases;
    private int manualRecoveries;
    private int roundsThisSession;
    private string sessionId;
    private string csvPath;
    private bool csvSaveSucceeded;

    private bool transportMode;
    private bool transportPending;
    private bool transportBusy;
    private bool cubeHiddenForTransport;
    private MeshFilter cubeMesh;
    private MeshRenderer cubeRenderer;
    private Renderer toolRenderer;
    private GameObject transportVisual;
    private Vector3 toolHomePosition;
    private Quaternion toolHomeRotation;
    private Vector3 armHomePosition;
    private Quaternion armHomeRotation;
    private Vector3 armHomeScale;
    private Vector3 armAnchor;
    private Vector3 toolToCubeOffset;
    private float toolHalfHeight;

    private void Awake()
    {
        ready = targetCube != null && targetZone != null && indicator != null;
        if (!ready)
        {
            Debug.LogError("Placement: assign Target Cube, Target Zone and Indicator.", this);
            return;
        }
        assistanceMode = simulateRobotAssistance;
        transportMode = simulateRobotTransport;
        mixedMode = useMixedRounds;
        roundsThisSession = Mathf.Clamp(totalRounds, 1, 10);
        if (mixedMode)
        {
            if (!assistanceMode || !transportMode || roundPlan == null
                || roundPlan.Length != roundsThisSession)
            {
                Debug.LogError("Mixed rounds: enable assistance and transport, and make Round Plan size equal Total Rounds.", this);
                ready = false;
                return;
            }
            foreach (var task in roundPlan)
            {
                if (task != RoundTask.AutoTransport && task != RoundTask.HumanAssistance)
                {
                    Debug.LogError("Mixed rounds: each plan entry must be Auto Transport or Human Assistance.", this);
                    ready = false;
                    return;
                }
            }
            // Keep the plan fixed throughout this Play session.
            sessionPlan = (RoundTask[])roundPlan.Clone();
        }
        if (transportMode && !ConfigureTransport())
        {
            ready = false;
            return;
        }
        colors = new MaterialPropertyBlock();
        results.Clear();
        sessionId = System.DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fff", CultureInfo.InvariantCulture)
            + "_" + System.Guid.NewGuid().ToString("N").Substring(0, 8);
        csvPath = Path.Combine(Application.persistentDataPath, "PlacementResults", "placement_" + sessionId + ".csv");
        initialPosition = targetCube.position;
        initialRotation = targetCube.rotation;
        cubeBody = targetCube.GetComponent<Rigidbody>();
        BeginRoundPhase();
    }

    // Scripted scenario only: no object recognition, grasp assessment or robot API.
    private void BeginRoundPhase()
    {
        currentRoundTask = mixedMode ? sessionPlan[successCount] : RoundTask.HumanAssistance;
        assistanceState = assistanceMode
            ? AssistanceState.Checking : AssistanceState.PracticeOnly;
        checkRemaining = Mathf.Max(0.5f, robotCheckDelay);
        SetColor(assistanceMode ? checkingColor : waitingColor);
        ShowStatus(assistanceMode
            ? "Robot checking... (simulated)\nPlease wait."
            : "Grab the cube.\nRelease it inside the target.");
    }

    private void Update()
    {
        if (!ready || sessionFinished) return;
        if (transportBusy) return;
        if (transportPending)
        {
            // Start on Update, outside the SDK's Unselect callback.
            if (selectingCount > 0)
            {
                ShowStatus(IsAutomaticRound
                    ? "Automatic round (scripted).\nLet go so the robot can collect it."
                    : "Help received.\nLet go so the robot can collect it.");
                return;
            }
            transportPending = false;
            transportBusy = true;
            StartCoroutine(TransportToCollection());
            return;
        }
        if (assistanceState == AssistanceState.Checking)
        {
            checkRemaining = Mathf.Max(0f, checkRemaining - Time.deltaTime);
            if (selectingCount > 0)
            {
                ShowStatus("Robot checking... (simulated)\nLet go before the task starts.");
                return;
            }
            if (checkRemaining > 0f)
            {
                ShowStatus($"Robot checking... (simulated)\nPlease wait: {Mathf.CeilToInt(checkRemaining)}...");
                return;
            }
            if (IsAutomaticRound)
            {
                BeginAutomaticRound();
                return;
            }
            assistanceState = AssistanceState.WaitingForHuman;
            SetColor(waitingColor);
            ShowStatus("Robot needs help (simulated).\nMove the cube into the target.");
            Debug.Log("Assistance: scripted help request. Waiting for human placement.", this);
            return;
        }
        if (!completed)
        {
            if (timerStarted)
                ShowStatus(selectingCount > 0
                    ? (assistanceMode ? "Helping robot (simulated).\nMove into the target, then let go." : "Move into the target, then let go.")
                    : (assistanceMode ? "Robot still needs help.\nGrab the cube and try again." : "Grab the cube and try again."));
            return;
        }
        if (!autoReset) return;

        // Track selections even after success: never teleport a held cube.
        if (selectingCount > 0)
        {
            ShowStatus(assistanceMode
                ? "Help received (simulated).\nLet go to start the next round."
                : "Success!\nLet go to start the next round.");
            return;
        }

        float remaining = resetAt - Time.time;
        if (remaining > 0f)
        {
            ShowStatus(assistanceMode
                ? $"{(transportMode ? "Delivered (simulated)." : "Help received (simulated).")}\nNext round in {Mathf.CeilToInt(remaining)}..."
                : $"Success!\nNext round in {Mathf.CeilToInt(remaining)}...");
            return;
        }

        ResetCurrentRound();
    }

    // An automatic round creates a transport record without a human timer.
    // The round is NOT fully complete until TransportToCollection returns.
    private void BeginAutomaticRound()
    {
        if (!ready || !transportMode || !IsAutomaticRound || completed || selectingCount > 0) return;
        completed = true;
        successCount++;
        timerStarted = false;
        results.Add(new RoundResult
        {
            round = successCount,
            humanRequired = false,
            roundType = "auto_transport"
        });
        transportPending = true;
        ShowStatus("Automatic round (scripted).\nNo help needed. Please wait.");
        ExportResultsCsv();
        Debug.Log($"Round {successCount}: auto transport selected by scripted plan; no human placement required.", this);
    }

    // Wire Grabbable's PointableUnityEventWrapper: When Select.
    public void NotifyGrabbed()
    {
        if (!ready || !isActiveAndEnabled || transportBusy || cubeHiddenForTransport) return;
        selectingCount++;
        if (completed || sessionFinished) return;
        if (assistanceState == AssistanceState.Checking)
        {
            ShowStatus("Robot checking... (simulated)\nLet go before the task starts.");
            return;
        }
        if (IsAutomaticRound) return;
        // A retry belongs to the same round: do not restart its timer.
        if (!timerStarted)
        {
            timerStarted = true;
            roundStartedAt = Time.timeAsDouble;
        }
        ShowStatus(assistanceMode
            ? "Helping robot (simulated).\nMove into the target, then let go."
            : "Holding cube.\nMove it into the target, then let go.");
        Debug.Log("Placement: grab received.", this);
    }

    // Wire When Unselect, not When Release. Use the no-argument method.
    public void NotifyReleased()
    {
        if (!ready || !isActiveAndEnabled || transportBusy || cubeHiddenForTransport || selectingCount == 0) return;
        selectingCount--;
        if (completed || sessionFinished) return;
        if (assistanceState == AssistanceState.Checking || IsAutomaticRound) return;
        if (selectingCount > 0) return;

        Vector3 point = targetZone.transform.InverseTransformPoint(targetCube.position);
        Bounds localBox = new Bounds(targetZone.center, targetZone.size);
        if (!localBox.Contains(point))
        {
            outsideReleases++;
            ShowStatus("Outside target.\nGrab the cube and try again.");
            Debug.Log("Placement: released outside target; try again.", this);
            return;
        }

        completed = true;
        if (assistanceMode)
        {
            assistanceState = AssistanceState.HelpReceived;
            Debug.Log("Assistance: help received (simulated). No real robot command was sent.", this);
        }
        successCount++;
        var result = new RoundResult
        {
            round = successCount,
            seconds = System.Math.Max(0d, Time.timeAsDouble - roundStartedAt),
            outsideReleases = outsideReleases,
            manualRecoveries = manualRecoveries,
            humanRequired = true,
            roundType = assistanceMode ? "human_assistance" : "placement_practice"
        };
        results.Add(result);
        ExportResultsCsv();
        Debug.Log($"Placement result: round={result.round}, seconds={result.seconds:F2}, outside_releases={result.outsideReleases}, manual_recoveries={result.manualRecoveries}", this);
        SetColor(successColor);
        if (transportMode)
        {
            transportPending = true;
            ShowStatus("Help received (simulated).\nRobot will collect the cube.");
        }
        else
        {
            FinishSuccessfulRound();
        }
        Debug.Log("Placement: SUCCESS - cube released inside target.", this);
    }

    private void FinishSuccessfulRound()
    {
        resetAt = Time.time + Mathf.Max(1f, resetDelay);
        if (successCount >= roundsThisSession)
        {
            sessionFinished = true;
            ShowSummary();
            return;
        }
        ShowStatus(transportMode
            ? "Delivered (simulated).\nCube is in the collection area."
            : (assistanceMode ? "Help received (simulated).\nCube released inside the target."
                : "Success!\nCube released inside the target."));
    }

    // Also accessible through the component's three-dot menu in Play mode.
    // An unfinished round retains its timer and misses during manual recovery.
    [ContextMenu("Reset Current Round")]
    public void ResetCurrentRound()
    {
        if (!Application.isPlaying || !ready || sessionFinished || transportPending || transportBusy) return;
        if (assistanceState == AssistanceState.Checking) return;
        if (selectingCount > 0)
        {
            Debug.Log("Placement: release the cube before resetting.", this);
            return;
        }

        bool startNewRound = completed;
        if (completed)
        {
            timerStarted = false;
            outsideReleases = 0;
            manualRecoveries = 0;
        }
        else if (timerStarted)
        {
            manualRecoveries++;
        }

        if (cubeBody != null)
        {
            if (!cubeBody.isKinematic)
            {
                cubeBody.linearVelocity = Vector3.zero;
                cubeBody.angularVelocity = Vector3.zero;
            }
            cubeBody.position = initialPosition;
            cubeBody.rotation = initialRotation;
        }
        targetCube.SetPositionAndRotation(initialPosition, initialRotation);
        // Set the next task phase before re-enabling XR components.
        bool restoreInteractiveCube = cubeHiddenForTransport;
        if (restoreInteractiveCube)
        {
            RemoveTransportVisual();
            RestoreRobotPose();
            selectingCount = 0;
            completed = false;
            cubeHiddenForTransport = false;
        }
        completed = false;
        if (startNewRound)
        {
            BeginRoundPhase();
        }
        else
        {
            SetColor(waitingColor);
            ShowStatus(assistanceMode
                ? "Robot still needs help.\nMove the cube into the target."
                : "Grab the cube.\nRelease it inside the target.");
        }
        if (restoreInteractiveCube) targetCube.gameObject.SetActive(true);
        Debug.Log("Placement: round reset; ready to grab again.", this);
    }

    private void ShowStatus(string message)
    {
        double elapsed = timerStarted ? System.Math.Max(0d, Time.timeAsDouble - roundStartedAt) : 0d;
        if (completed && results.Count > 0) elapsed = results[results.Count - 1].seconds;
        int shownRound = completed ? successCount : successCount + 1;
        string taskLabel = mixedMode ? (IsAutomaticRound ? " | AUTO" : " | HELP") : "";
        string timing = IsAutomaticRound ? "\nHuman time: N/A"
            : $"\nTime: {elapsed:F1}s | Misses: {outsideReleases}";
        string text = $"Round {shownRound}/{roundsThisSession}{taskLabel}\n" + message + timing;
        SetText(text);
    }

    private void ShowSummary()
    {
        var summary = new StringBuilder(mixedMode ? "Mixed session complete\n"
            : (assistanceMode
                ? (transportMode ? "Transport session complete\n" : "Simulated assistance complete\n")
                : "Session complete\n"));
        double totalSeconds = 0d;
        int humanRounds = 0;
        int totalMisses = 0;
        int totalRecoveries = 0;
        foreach (var result in results)
        {
            if (!result.humanRequired)
            {
                summary.AppendLine($"{result.round}: AUTO | delivered");
                continue;
            }
            humanRounds++;
            totalSeconds += result.seconds;
            totalMisses += result.outsideReleases;
            totalRecoveries += result.manualRecoveries;
            summary.AppendLine($"{result.round}: {result.seconds:F1}s | misses: {result.outsideReleases}");
        }
        string averageLabel = mixedMode ? "Human average" : "Average";
        summary.AppendLine(humanRounds > 0
            ? $"{averageLabel}: {totalSeconds / humanRounds:F1}s"
            : $"{averageLabel}: N/A");
        summary.Append($"Misses: {totalMisses} | Recoveries: {totalRecoveries}");
        summary.Append(csvSaveSucceeded ? "\nCSV: saved" : "\nCSV: save failed - see Console");
        SetText(summary.ToString());
        Debug.Log(summary.ToString(), this);
    }

    private void SetText(string text)
    {
        if (statusText == null || text == lastMessage) return;
        statusText.text = text;
        lastMessage = text;
    }

    // Save when a round becomes ready for transport, then update after delivery.
    // An interrupted transport remains false with no duration. Unfinished human
    // placement is not exported. Each Play session gets a unique filename.
    [ContextMenu("Export Results CSV")]
    public void ExportResultsCsv()
    {
        if (!Application.isPlaying || !ready || results.Count == 0)
        {
            Debug.LogWarning("Task CSV: complete a human placement or begin an automatic transport in Play mode first.", this);
            return;
        }

        csvSaveSucceeded = false;
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(csvPath));
            var csv = new StringBuilder();
            csv.AppendLine("session_id,round,total_rounds,time_seconds,outside_releases,manual_recoveries,session_complete,time_basis,run_environment,task_mode,transport_complete,transport_seconds,round_type,human_required,decision_source");
            bool allComplete = results.Count >= roundsThisSession;
            if (transportMode)
                foreach (var recorded in results) allComplete &= recorded.transportComplete;
            string sessionComplete = allComplete ? "true" : "false";
            string environment = Application.isEditor ? "UnityEditor" : "Player";
            foreach (var result in results)
            {
                csv.Append(sessionId).Append(',')
                    .Append(result.round.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(roundsThisSession.ToString(CultureInfo.InvariantCulture)).Append(',')
                    .Append(result.humanRequired ? result.seconds.ToString("F3", CultureInfo.InvariantCulture) : "").Append(',')
                    .Append(result.humanRequired ? result.outsideReleases.ToString(CultureInfo.InvariantCulture) : "").Append(',')
                    .Append(result.humanRequired ? result.manualRecoveries.ToString(CultureInfo.InvariantCulture) : "").Append(',')
                    .Append(sessionComplete).Append(',')
                    .Append("unity_scaled_game_time").Append(',')
                    .Append(environment).Append(',')
                    .Append(mixedMode ? "mixed_simulated_cooperation"
                        : (transportMode ? "simulated_robot_assistance_with_transport"
                            : (assistanceMode ? "simulated_robot_assistance" : "placement_practice"))).Append(',')
                    .Append(transportMode ? (result.transportComplete ? "true" : "false") : "not_applicable").Append(',')
                    .Append(transportMode && result.transportComplete
                        ? result.transportSeconds.ToString("F3", CultureInfo.InvariantCulture) : "").Append(',')
                    .Append(result.roundType).Append(',')
                    .Append(result.humanRequired ? "true" : "false").Append(',')
                    .AppendLine(mixedMode ? "scripted_round_plan"
                        : (assistanceMode ? "scripted_help_request" : "placement_practice"));
            }
            // UTF-8 BOM supports opening the file directly in Excel.
            File.WriteAllText(csvPath, csv.ToString(), new UTF8Encoding(true));
            csvSaveSucceeded = true;
            Debug.Log($"Task CSV saved ({results.Count}/{roundsThisSession} round records):\n{csvPath}", this);
        }
        catch (System.Exception exception)
        {
            Debug.LogError($"Placement CSV save failed: {exception.Message}\nPath: {csvPath}\nClose the CSV in Excel if it is open, then use Export Results CSV to retry before leaving Play.", this);
        }

        if (sessionFinished) ShowSummary();
    }

    // Available after stopping Play too; the folder path does not depend on
    // the transient session fields. This opens files only in the Unity Editor.
    [ContextMenu("Open Results Folder")]
    public void OpenResultsFolder()
    {
        string directory = Path.Combine(Application.persistentDataPath, "PlacementResults");
        try
        {
            Directory.CreateDirectory(directory);
#if UNITY_EDITOR
            UnityEditor.EditorUtility.RevealInFinder(
                !string.IsNullOrEmpty(csvPath) && File.Exists(csvPath) ? csvPath : directory);
#else
            Debug.Log("Placement results folder: " + directory, this);
#endif
        }
        catch (System.Exception exception)
        {
            Debug.LogError("Cannot open results folder: " + exception.Message, this);
        }
    }


    // This visual mock uses the single-mesh Building Block cube and unit-scale
    // robot parent groups from the lesson. It performs no IK or collision planning.
    private bool ConfigureTransport()
    {
        if (!assistanceMode || robotTool == null || robotArm == null
            || robotColumn == null || collectionPad == null)
        {
            Debug.LogError("Transport: enable Simulate Robot Assistance and assign Robot Tool, Robot Arm, Robot Column and Collection Pad.", this);
            return false;
        }
        cubeMesh = targetCube.GetComponent<MeshFilter>();
        cubeRenderer = targetCube.GetComponent<MeshRenderer>();
        toolRenderer = robotTool.GetComponent<Renderer>();
        if (cubeMesh == null || cubeMesh.sharedMesh == null || cubeRenderer == null || toolRenderer == null)
        {
            Debug.LogError("Transport: Target Cube must be the visible Building Block cube root with MeshFilter and MeshRenderer. Robot Tool needs a Renderer.", this);
            return false;
        }
        if (robotTool.IsChildOf(robotArm) || transform.IsChildOf(targetCube)
            || (robotArm.parent != null
                && (robotArm.parent.lossyScale - Vector3.one).sqrMagnitude > 0.0001f))
        {
            Debug.LogError("Transport: keep Robot Tool and Robot Arm as siblings; keep parent scales at (1,1,1); keep this script on TargetZone.", this);
            return false;
        }
        toolHomePosition = robotTool.position;
        toolHomeRotation = robotTool.rotation;
        armHomePosition = robotArm.position;
        armHomeRotation = robotArm.rotation;
        armHomeScale = robotArm.localScale;
        Bounds columnBounds = robotColumn.bounds;
        armAnchor = new Vector3(columnBounds.center.x, columnBounds.max.y, columnBounds.center.z);
        toolHalfHeight = toolRenderer.bounds.extents.y;
        return true;
    }

    private IEnumerator TransportToCollection()
    {
        assistanceState = AssistanceState.RobotTransporting;
        double startedAt = Time.timeAsDouble;
        RoundResult result = results[results.Count - 1];

        // Copy ONLY the visible mesh, never the XR scripts or colliders.
        // This prevents SDK grabbing and animation from driving one object together.
        Bounds cubeBounds = cubeRenderer.bounds;
        Bounds padBounds = collectionPad.bounds;
        Vector3 pickupPosition = targetCube.position;
        float pivotToBottom = pickupPosition.y - cubeBounds.min.y;
        float pivotToTop = cubeBounds.max.y - pickupPosition.y;
        Vector3 dropPosition = new Vector3(padBounds.center.x,
            padBounds.max.y + pivotToBottom + 0.003f, padBounds.center.z);
        toolToCubeOffset = Vector3.up * (pivotToTop + toolHalfHeight + 0.01f);
        float cruiseY = Mathf.Max(pickupPosition.y, dropPosition.y) + Mathf.Max(0.05f, liftHeight);
        Vector3 pickupHover = new Vector3(pickupPosition.x, cruiseY, pickupPosition.z) + toolToCubeOffset;
        Vector3 dropHover = new Vector3(dropPosition.x, cruiseY, dropPosition.z) + toolToCubeOffset;

        transportVisual = new GameObject("TransportVisual_Runtime");
        transportVisual.layer = targetCube.gameObject.layer;
        transportVisual.transform.SetPositionAndRotation(pickupPosition, targetCube.rotation);
        transportVisual.transform.localScale = targetCube.lossyScale;
        transportVisual.AddComponent<MeshFilter>().sharedMesh = cubeMesh.sharedMesh;
        MeshRenderer visualRenderer = transportVisual.AddComponent<MeshRenderer>();
        visualRenderer.sharedMaterials = cubeRenderer.sharedMaterials;
        visualRenderer.shadowCastingMode = cubeRenderer.shadowCastingMode;
        visualRenderer.receiveShadows = cubeRenderer.receiveShadows;
        var appearance = new MaterialPropertyBlock();
        cubeRenderer.GetPropertyBlock(appearance);
        visualRenderer.SetPropertyBlock(appearance);

        cubeHiddenForTransport = true;
        targetCube.gameObject.SetActive(false);
        selectingCount = 0;
        ShowStatus(IsAutomaticRound
            ? "Automatic round (scripted).\nNo help needed. Please wait."
            : "Help received.\nRobot taking over (simulated).");
        yield return new WaitForSeconds(0.4f);
        SetColor(checkingColor);
        ShowStatus("Robot approaching (simulated).\nPlease wait.");
        yield return MoveTool(pickupHover, false);
        yield return MoveTool(pickupPosition + toolToCubeOffset, false);
        ShowStatus("Robot lifting (simulated).\nPlease wait.");
        yield return MoveTool(pickupHover, true);
        ShowStatus("Robot transporting (simulated).\nMoving to the purple area.");
        yield return MoveTool(dropHover, true);
        yield return MoveTool(dropPosition + toolToCubeOffset, true);
        transportVisual.transform.position = dropPosition;
        ShowStatus("Cube delivered (simulated).\nRobot returning.");
        yield return new WaitForSeconds(0.5f);
        yield return MoveTool(dropHover, false);
        Vector3 homeHover = new Vector3(toolHomePosition.x,
            Mathf.Max(dropHover.y, toolHomePosition.y), toolHomePosition.z);
        yield return MoveTool(homeHover, false);
        yield return MoveTool(toolHomePosition, false);
        RestoreRobotPose();

        result.transportComplete = true;
        result.transportSeconds = System.Math.Max(0d, Time.timeAsDouble - startedAt);
        assistanceState = AssistanceState.DeliveryComplete;
        transportBusy = false;
        SetColor(successColor);
        ExportResultsCsv();
        Debug.Log($"Transport complete: round={result.round}, animation_seconds={result.transportSeconds:F2}. Visual simulation only.", this);
        FinishSuccessfulRound();
    }

    private IEnumerator MoveTool(Vector3 destination, bool carryCube)
    {
        while ((robotTool.position - destination).sqrMagnitude > 0.000001f)
        {
            robotTool.position = Vector3.MoveTowards(robotTool.position, destination,
                Mathf.Max(0.05f, transportSpeed) * Time.deltaTime);
            UpdateArmVisual();
            if (carryCube) transportVisual.transform.position = robotTool.position - toolToCubeOffset;
            yield return null;
        }
        robotTool.position = destination;
        UpdateArmVisual();
        if (carryCube) transportVisual.transform.position = destination - toolToCubeOffset;
    }

    private void UpdateArmVisual()
    {
        // A stretching connector makes the simple mock visibly connected.
        // It is not a model of UR5 joints or reachability.
        Vector3 tip = robotTool.position + Vector3.up * (toolHalfHeight + armHomeScale.y * 0.5f);
        Vector3 direction = tip - armAnchor;
        if (direction.sqrMagnitude < 0.000001f) return;
        robotArm.position = (armAnchor + tip) * 0.5f;
        robotArm.rotation = Quaternion.FromToRotation(Vector3.right, direction.normalized);
        robotArm.localScale = new Vector3(direction.magnitude, armHomeScale.y, armHomeScale.z);
    }

    private void RestoreRobotPose()
    {
        if (!transportMode) return;
        if (robotTool != null) robotTool.SetPositionAndRotation(toolHomePosition, toolHomeRotation);
        if (robotArm != null)
        {
            robotArm.SetPositionAndRotation(armHomePosition, armHomeRotation);
            robotArm.localScale = armHomeScale;
        }
    }

    private void RemoveTransportVisual()
    {
        if (transportVisual == null) return;
        transportVisual.SetActive(false);
        Destroy(transportVisual);
        transportVisual = null;
    }

    private void OnDisable()
    {
        if (!ready) return;
        ready = false;
        StopAllCoroutines();
        transportPending = false;
        transportBusy = false;
        RemoveTransportVisual();
        RestoreRobotPose();
        if (cubeHiddenForTransport && targetCube != null)
        {
            targetCube.SetPositionAndRotation(initialPosition, initialRotation);
            targetCube.gameObject.SetActive(true);
        }
        cubeHiddenForTransport = false;
        // After disabling this component during Play, restart Play to initialize
        // a fresh session. An interrupted transport remains incomplete in CSV.
    }

    private void SetColor(Color color)
    {
        indicator.GetPropertyBlock(colors);
        colors.SetColor("_BaseColor", color); // URP Lit
        colors.SetColor("_Color", color);     // Built-in shaders
        indicator.SetPropertyBlock(colors);
    }
}
