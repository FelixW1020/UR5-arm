using System.Collections.Generic;
using RosMessageTypes.VrInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Draws where ROS thinks each ArUco tag is, in VR. With passthrough on, the virtual
/// squares should sit on the printed tags. This is the quickest check of the registration.
/// Hook Refresh() to the registration scripts' onRegistered event.
/// </summary>
public class TagMapVisualizer : MonoBehaviour
{
    public string serviceName = "/vr_registration/get_tag_map";
    public float tagSize = 0.10f;
    public Material material;

    readonly List<GameObject> _tags = new List<GameObject>();
    ROSConnection _ros;

    void Start() => Refresh();

    // onRegistered can call Refresh before Start (e.g. this GameObject starts inactive).
    void EnsureRos()
    {
        if (_ros != null) return;
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterRosService(serviceName, GetTagMapRequest.k_RosMessageName,
                                GetTagMapResponse.k_RosMessageName);
    }

    // EmptyRequest, not GetTagMapRequest: ROS-TCP-Connector serialises empty generated requests
    // in a form ROS 2 can't read, so the call would never be answered (see RosUtil.cs).
    public void Refresh()
    {
        EnsureRos();
        _ros.SendServiceMessage<GetTagMapResponse>(serviceName,
            new EmptyRequest(GetTagMapRequest.k_RosMessageName), OnTagMap);
    }

    void OnTagMap(GetTagMapResponse r)
    {
        foreach (var t in _tags) Destroy(t);
        _tags.Clear();
        if (!r.success || r.frame_id != "vr_world")
        {
            Debug.Log($"[TagMapVisualizer] Not drawing tags: {r.message}");
            return;
        }
        for (int i = 0; i < r.ids.Length; i++)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Destroy(go.GetComponent<Collider>());
            go.name = $"tag_{r.ids[i]}";
            RobotWorldFrame.Instance.Apply(r.poses[i], go.transform);
            // The tag's ROS z axis (out of the printed face) becomes local +Y in Unity.
            go.transform.localScale = new Vector3(tagSize, 0.002f, tagSize);
            if (material != null) go.GetComponent<Renderer>().material = material;
            _tags.Add(go);
        }
    }
}
