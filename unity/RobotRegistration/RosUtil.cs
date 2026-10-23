using System;
using System.Threading.Tasks;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

/// <summary>Small ROS helpers shared by the RobotRegistration scripts.</summary>
public static class RosUtil
{
    /// <summary>
    /// A UTC time as a ROS stamp. Stamps sent from the headset are informational only: the
    /// headset clock isn't synchronised with ROS, so the ROS side restamps on receipt.
    /// </summary>
    public static TimeMsg Stamp(DateTime utc)
    {
        long ticks = Math.Max(0L, (utc - DateTime.UnixEpoch).Ticks);   // 100 ns ticks
        long sec = ticks / TimeSpan.TicksPerSecond;
        uint nanosec = (uint)(ticks % TimeSpan.TicksPerSecond * 100);
#if ROS2
        return new TimeMsg((int)sec, nanosec);
#else
        return new TimeMsg((uint)sec, nanosec);
#endif
    }

    /// <summary>
    /// Call a ROS service and give up after timeoutS (returns null). ROS-TCP-Endpoint never
    /// answers a call to a service that isn't running, so a plain SendServiceMessage would wait
    /// forever and leave the caller's "busy" flag stuck. Call from the main thread.
    /// </summary>
    public static async Task<T> CallService<T>(this ROSConnection ros, string service,
                                               Message request, float timeoutS)
        where T : Message, new()
    {
        var call = ros.SendServiceMessage<T>(service, request);
        var first = await Task.WhenAny(call, Task.Delay(TimeSpan.FromSeconds(timeoutS)));
        return first == call ? await call : null;
    }
}

/// <summary>
/// Request of a service whose request has no fields (std_srvs/Trigger, vr_interfaces/GetTagMap).
/// ROS 2 serialises an empty message as one dummy byte, but ROS-TCP-Connector's generated empty
/// classes serialise as nothing: the endpoint fails to deserialize the request (Fast-DDS and
/// Cyclone alike) and never answers. Send this instead of the generated XxxRequest.
/// </summary>
public class EmptyRequest : Message
{
    readonly string _rosMessageName;

    /// <param name="rosServiceType">e.g. "std_srvs/Trigger" (the generated class's k_RosMessageName)</param>
    public EmptyRequest(string rosServiceType) => _rosMessageName = rosServiceType;

    public override string RosMessageName => _rosMessageName;

    public override void SerializeTo(MessageSerializer serializer)
    {
#if ROS2
        serializer.Write((byte)0);   // structure_needs_at_least_one_member
#endif
    }
}

/// <summary>std_srvs/Trigger response, hand-written so std_srvs doesn't need generating.</summary>
public class TriggerResult : Message
{
    public const string k_RosMessageName = "std_srvs/Trigger";
    public override string RosMessageName => k_RosMessageName;

    public bool success;
    public string message = "";

    static bool s_registered;

    public override void SerializeTo(MessageSerializer serializer)
    {
        serializer.Write(success);
        serializer.Write(message);
    }

    public static TriggerResult Deserialize(MessageDeserializer deserializer)
    {
        var r = new TriggerResult();
        deserializer.Read(out r.success);
        deserializer.Read(out r.message);
        return r;
    }

    /// <summary>Call before the first service call that returns a TriggerResult.</summary>
    public static void Register()
    {
        if (s_registered) return;
        s_registered = true;
        MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
    }
}
