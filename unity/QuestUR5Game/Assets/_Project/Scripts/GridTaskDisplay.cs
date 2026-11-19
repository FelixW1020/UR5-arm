using TMPro;
using UnityEngine;
using UnityEngine.UI;

// Displays the existing manager's live counts in world space.
// Read-only display; the manager owns task rules and statistics.
[DefaultExecutionOrder(100)]
public class GridTaskDisplay : MonoBehaviour
{
    public GridTaskManager taskManager;
    public Transform taskRoot;
    public Camera viewerCamera;
    public TMP_FontAsset fontAsset;

    [Header("Position relative to GridTaskRoot, in meters")]
    public Vector3 localPosition = new Vector3(0f, 0.55f, 0.35f);

    private GameObject board;
    private TextMeshProUGUI label;
    private int lastTotal = -1;
    private int lastCompleted = -1;
    private bool lastReady;
    private int lastPicks = -1;
    private int lastInvalid = -1;
    private int lastTenths = -1;

    private void Start()
    {
        if (taskManager == null || taskRoot == null)
        {
            Debug.LogError("GridTaskDisplay: assign Task Manager and Task Root.", this);
            enabled = false;
            return;
        }

        TMP_FontAsset font = fontAsset != null
            ? fontAsset : TMP_Settings.defaultFontAsset;

        if (font == null)
        {
            Debug.LogError(
                "GridTaskDisplay: import TMP Essential Resources or assign a TMP Font Asset.",
                this);
            enabled = false;
            return;
        }

        if (viewerCamera == null) viewerCamera = Camera.main;

        board = new GameObject("GridStatusBoard_Runtime",
            typeof(RectTransform), typeof(Canvas));
        RectTransform rect = board.GetComponent<RectTransform>();
        rect.SetParent(taskRoot, false);
        rect.localPosition = localPosition;
        rect.localRotation = Quaternion.identity;
        rect.localScale = Vector3.one * 0.001f;
        rect.sizeDelta = new Vector2(800f, 260f);

        Canvas canvas = board.GetComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.worldCamera = viewerCamera;

        Image background = board.AddComponent<Image>();
        background.color = new Color(0.035f, 0.055f, 0.09f, 0.95f);
        background.raycastTarget = false;

        GameObject textObject = new GameObject("StatusText",
            typeof(RectTransform));
        textObject.transform.SetParent(rect, false);
        label = textObject.AddComponent<TextMeshProUGUI>();
        label.font = font;
        label.fontSize = 36f;
        label.color = Color.white;
        label.alignment = TextAlignmentOptions.Center;
        label.raycastTarget = false;
        label.richText = true;

        RectTransform textRect = label.rectTransform;
        textRect.anchorMin = Vector2.zero;
        textRect.anchorMax = Vector2.one;
        textRect.offsetMin = new Vector2(18f, 12f);
        textRect.offsetMax = new Vector2(-18f, -12f);

        RefreshText();
    }

    private void LateUpdate()
    {
        if (board == null || label == null) return;

        board.transform.localPosition = localPosition;
        if (viewerCamera == null) viewerCamera = Camera.main;

        if (viewerCamera != null)
        {
            Vector3 awayFromViewer = board.transform.position
                - viewerCamera.transform.position;
            if (awayFromViewer.sqrMagnitude > 0.0001f)
                board.transform.rotation = Quaternion.LookRotation(
                    awayFromViewer, Vector3.up);
        }

        RefreshText();
    }

    private void RefreshText()
    {
        if (label == null || taskManager == null) return;

        int total = taskManager.TotalObjects;
        int completed = taskManager.CompletedCount;
        bool ready = taskManager.isActiveAndEnabled && total > 0;
        int picks = taskManager.TotalPickCount;
        int invalid = taskManager.TotalInvalidReleases;
        int tenths = Mathf.FloorToInt((float)(taskManager.ElapsedSeconds * 10d));

        if (total == lastTotal && completed == lastCompleted &&
            ready == lastReady && picks == lastPicks &&
            tenths == lastTenths && invalid == lastInvalid) return;

        lastTotal = total;
        lastCompleted = completed;
        lastReady = ready;
        lastPicks = picks;
        lastInvalid = invalid;
        lastTenths = tenths;

        if (!ready)
        {
            label.text = "GRID PICK & PLACE\nCheck TaskManager and Console";
            return;
        }

        string hint = !taskManager.TimerStarted
            ? "Grab a block to start"
            : completed == total
                ? "All blocks placed - timer paused"
                : "Release blocks fully on the green mat";
        label.text = "<size=27>GRID PICK & PLACE</size>\n"
            + "<size=46>Completed: " + completed + " / " + total + "</size>\n"
            + "<size=30>Time: " + taskManager.ElapsedSeconds.ToString("F1")
            + " s</size>\n"
            + "<size=30>Grabs: " + picks + "   |   Invalid releases: " + invalid + "</size>\n"
            + "<size=23>" + hint + "</size>";
    }

    private void OnEnable()
    {
        if (board != null) board.SetActive(true);
    }

    private void OnDisable()
    {
        if (board != null) board.SetActive(false);
    }

    private void OnDestroy()
    {
        if (board != null) Destroy(board);
    }
}

