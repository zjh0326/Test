using UnityEngine;
using TMPro;

public class MotorControllerUI : MonoBehaviour
{
    [SerializeField] private TMP_InputField portInput;
    [SerializeField] private TMP_InputField inputSpeed;
    [SerializeField] private TMP_InputField inputCurrentLimit;
    [SerializeField] private TMP_InputField inputPositionAngle;
    [SerializeField] private TMP_InputField inputPositionSpeedLimit;
    [SerializeField] private TMP_InputField inputCurrentModeCurrent;
    [Header("MIT Mode")]
    [SerializeField] private TMP_InputField inputMIT_Kp;
    [SerializeField] private TMP_InputField inputMIT_Kd;
    [SerializeField] private TMP_InputField inputMIT_Position;
    [SerializeField] private TMP_InputField inputMIT_Velocity;
    [SerializeField] private TMP_InputField inputMIT_Torque;
    [SerializeField] private int baudRate = 115200;

    private UartManager uart;
    private AtFrameDecoder decoder;

    public MotorFeedback LastFeedback { get; private set; }
    public bool MotorEnabled { get; private set; }
    public event System.Action<ushort, byte[]> OnGetParamResponse;

    void Start()
    {
        if (inputSpeed != null && string.IsNullOrEmpty(inputSpeed.text)) inputSpeed.text = "5";
        if (inputCurrentLimit != null && string.IsNullOrEmpty(inputCurrentLimit.text)) inputCurrentLimit.text = "10";
        if (inputPositionAngle != null && string.IsNullOrEmpty(inputPositionAngle.text)) inputPositionAngle.text = "3.14";
        if (inputPositionSpeedLimit != null && string.IsNullOrEmpty(inputPositionSpeedLimit.text)) inputPositionSpeedLimit.text = "2";
        if (inputCurrentModeCurrent != null && string.IsNullOrEmpty(inputCurrentModeCurrent.text)) inputCurrentModeCurrent.text = "1";
        if (inputMIT_Kp != null && string.IsNullOrEmpty(inputMIT_Kp.text)) inputMIT_Kp.text = "50";
        if (inputMIT_Kd != null && string.IsNullOrEmpty(inputMIT_Kd.text)) inputMIT_Kd.text = "1";
        if (inputMIT_Position != null && string.IsNullOrEmpty(inputMIT_Position.text)) inputMIT_Position.text = "0";
        if (inputMIT_Velocity != null && string.IsNullOrEmpty(inputMIT_Velocity.text)) inputMIT_Velocity.text = "2";
        if (inputMIT_Torque != null && string.IsNullOrEmpty(inputMIT_Torque.text)) inputMIT_Torque.text = "0";

        uart = gameObject.AddComponent<UartManager>();
        decoder = new AtFrameDecoder();
        decoder.OnFrameDecoded += frame =>
        {
            uint commType = (frame.canId >> 24) & 0xFF;
            if (commType == 0x02 || commType == 0x18)
            {
                LastFeedback = MotorFeedback.Parse(frame);
                Debug.Log($"[RX] {LastFeedback}");
            }
            else if (commType == 0x11 && frame.dataLen >= 8)
            {
                ushort idx = (ushort)(frame.data[0] | (frame.data[1] << 8));
                if (idx == 0x7005)
                    Debug.Log($"[RX] GetParam idx=0x{idx:X4} mode={frame.data[4]}");
                else if (idx == 0x7006)
                {
                    float val = System.BitConverter.ToSingle(frame.data, 4);
                    Debug.Log($"[RX] GetParam idx=0x{idx:X4} iq_ref={val:F3}A");
                }
                else
                    Debug.Log($"[RX] GetParam idx=0x{idx:X4}");
                OnGetParamResponse?.Invoke(idx, frame.data);
            }
        };
        uart.OnDataReceived += data => decoder.Feed(data);
    }

    public void OnOpenClick()
    {
        if (uart == null) return;
        if (portInput == null) { Debug.LogError("PortInput not assigned!"); return; }
        uart.portName = portInput.text;
        uart.baudRate = baudRate;
        uart.Open();
    }

    public void OnCloseClick()
    {
        uart?.Close();
    }

    public void OnEnableClick()
    {
        _currentModeActive = false;
        _mitModeActive = false;
        MotorEnabled = true;
        SendMotorCommand(0x03);
    }

    public void OnDisableClick()
    {
        _currentModeActive = false;
        _mitModeActive = false;
        MotorEnabled = false;
        SendMotorCommand(0x04, 1);
    }

    private void SendMotorCommand(uint commType, byte data0 = 0)
    {
        uint canId = (commType << 24) | (0x00 << 16) | (0xFD << 8) | 0x7F;
        byte[] data = new byte[8];
        data[0] = data0;
        byte[] frame = AtFrame.Encode(canId, data, 8);
        uart.Send(frame);
        Debug.Log($"[TX] Comm=0x{commType:X2} ID=0x{canId:X8}");
    }

    public void SendSetParam(ushort index, float value)
    {
        uint canId = (0x12u << 24) | (0x00u << 16) | (0xFD << 8) | 0x7Fu;
        byte[] data = new byte[8];
        data[0] = (byte)(index & 0xFF);
        data[1] = (byte)(index >> 8);
        System.BitConverter.GetBytes(value).CopyTo(data, 4);
        uart.Send(AtFrame.Encode(canId, data, 8));
        Debug.Log($"[TX] SetParam idx=0x{index:X4} val={value} (float)");
    }

    public void SendSetParamMode(ushort index, byte modeValue)
    {
        uint canId = (0x12u << 24) | (0x00u << 16) | (0xFD << 8) | 0x7Fu;
        byte[] data = new byte[8];
        data[0] = (byte)(index & 0xFF);
        data[1] = (byte)(index >> 8);
        data[4] = modeValue;
        uart.Send(AtFrame.Encode(canId, data, 8));
        Debug.Log($"[TX] SetParamMode idx=0x{index:X4} val={modeValue} (byte)");
    }

    public void SendGetParam(ushort index)
    {
        uint canId = (0x11u << 24) | (0x00u << 16) | (0xFD << 8) | 0x7Fu;
        byte[] data = new byte[8];
        data[0] = (byte)(index & 0xFF);
        data[1] = (byte)(index >> 8);
        uart.Send(AtFrame.Encode(canId, data, 8));
        Debug.Log($"[TX] GetParam idx=0x{index:X4}");
    }

    public void OnSpeedModeClick()
    {
        _currentModeActive = false;
        _mitModeActive = false;
        float speed = float.TryParse(inputSpeed?.text, out float s) ? s : 0f;
        float limit = float.TryParse(inputCurrentLimit?.text, out float l) ? l : 0f;
        StartSpeedMode(speed, limit);
    }

    public void StartSpeedMode(float speed, float currentLimit)
    {
        StartCoroutine(SpeedModeSequence(speed, currentLimit));
    }

    private System.Collections.IEnumerator SpeedModeSequence(float speed, float currentLimit)
    {
        SendMotorCommand(0x04, 1);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x7028, 0f);
        yield return new WaitForSeconds(0.15f);
        SendSetParamMode(0x7005, 2);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x7018, currentLimit);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x700A, 0f);
        yield return new WaitForSeconds(0.15f);
        SendMotorCommand(0x03);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x700A, speed);
        Debug.Log($"[TX] Speed mode entered: {speed} rad/s, {currentLimit} A");
    }

    public void OnPositionModeClick()
    {
        _currentModeActive = false;
        _mitModeActive = false;
        float angle = float.TryParse(inputPositionAngle?.text, out float a) ? a : 0f;
        float speedLimit = float.TryParse(inputPositionSpeedLimit?.text, out float s) ? s : 2f;
        StartPositionMode(angle, speedLimit);
    }

    public void StartPositionMode(float angle, float speedLimit)
    {
        StartCoroutine(PositionModeSequence(angle, speedLimit));
    }

    private System.Collections.IEnumerator PositionModeSequence(float angle, float speedLimit)
    {
        SendMotorCommand(0x04, 1);
        yield return new WaitForSeconds(0.15f);
        SendSetParamMode(0x7005, 1);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x7024, speedLimit);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x7025, 10f);
        yield return new WaitForSeconds(0.15f);
        SendMotorCommand(0x03);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x7016, angle);
        Debug.Log($"[TX] Position mode entered: angle={angle} rad, speedLimit={speedLimit} rad/s");
    }

    public void OnCurrentModeClick()
    {
        _mitModeActive = false;
        string rawText = inputCurrentModeCurrent?.text ?? "null";
        float current = float.TryParse(rawText, out float c) ? c : 0f;
        Debug.Log($"[DEBUG] OnCurrentModeClick: rawText='{rawText}' parsed={current}");
        StartCurrentMode(current);
    }

    public void StartCurrentMode(float current)
    {
        StartCoroutine(CurrentModeSequence(current));
    }

    private bool _currentModeActive;

    private System.Collections.IEnumerator CurrentModeSequence(float current)
    {
        _currentModeActive = false;
        SendMotorCommand(0x04, 1);
        yield return new WaitForSeconds(0.15f);
        SendSetParamMode(0x7005, 3);
        yield return new WaitForSeconds(0.15f);
        SendGetParam(0x7005);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x7018, 23f);
        yield return new WaitForSeconds(0.05f);
        SendMotorCommand(0x03);
        yield return new WaitForSeconds(0.15f);
        SendSetParam(0x7006, current);
        yield return new WaitForSeconds(0.05f);
        SendGetParam(0x7006);
        yield return new WaitForSeconds(0.05f);
        _currentModeActive = true;
        Debug.Log($"[TX] Current mode started: {current} A (continuous)");
        while (_currentModeActive)
        {
            SendSetParam(0x7006, current);
            yield return new WaitForSeconds(0.05f);
        }
        Debug.Log("[TX] Current mode stopped");
    }

    public void OnZeroSettingClick()
    {
        _currentModeActive = false;
        _mitModeActive = false;
        StartCoroutine(ZeroSettingSequence());
    }

    private System.Collections.IEnumerator ZeroSettingSequence()
    {
        SendMotorCommand(0x04, 0);
        yield return new WaitForSeconds(0.1f);
        SendMotorCommand(0x06, 1);
        yield return new WaitForSeconds(0.1f);
        SendMotorCommand(0x03);
        Debug.Log($"[TX] Zero setting done");
    }

    public void OnMITModeClick()
    {
        _currentModeActive = false;
        float kp = float.TryParse(inputMIT_Kp?.text, out float k) ? k : 50f;
        float kd = float.TryParse(inputMIT_Kd?.text, out float d) ? d : 1f;
        float pos = float.TryParse(inputMIT_Position?.text, out float p) ? p : 0f;
        float vel = float.TryParse(inputMIT_Velocity?.text, out float v) ? v : 2f;
        float tor = float.TryParse(inputMIT_Torque?.text, out float t) ? t : 0f;
        StartMITMode(kp, kd, pos, vel, tor);
    }

    public void StartMITMode(float kp, float kd, float position, float velocity, float torque)
    {
        StartCoroutine(MITModeSequence(kp, kd, position, velocity, torque));
    }

    private bool _mitModeActive;

    private System.Collections.IEnumerator MITModeSequence(float kp, float kd, float position, float velocity, float torque)
    {
        _mitModeActive = false;
        SendMotorCommand(0x04, 1);
        yield return new WaitForSeconds(0.15f);
        SendSetParamMode(0x7005, 0);
        yield return new WaitForSeconds(0.15f);
        SendMotorCommand(0x03);
        yield return new WaitForSeconds(0.15f);
        _mitModeActive = true;
        Debug.Log($"[TX] MIT mode started: Kp={kp} Kd={kd} Pos={position} Vel={velocity} Torq={torque}");
        while (_mitModeActive)
        {
            SendMITControl(position, velocity, kp, kd, torque);
            yield return new WaitForSeconds(0.02f);
        }
        Debug.Log("[TX] MIT mode stopped");
    }

    private void SendMITControl(float angle, float velocity, float kp, float kd, float torque)
    {
        ushort angleU16 = (ushort)FloatToUint(angle, -12.5f, 12.5f, 16);
        ushort velU16 = (ushort)FloatToUint(velocity, -44f, 44f, 16);
        ushort kpU16 = (ushort)FloatToUint(kp, 0f, 500f, 16);
        ushort kdU16 = (ushort)FloatToUint(kd, 0f, 5f, 16);
        ushort torU16 = (ushort)FloatToUint(torque, -17f, 17f, 16);

        uint canId = (1u << 24) | ((uint)torU16 << 8) | 0x7Fu;

        byte[] data = new byte[8];
        data[0] = (byte)(angleU16 >> 8);
        data[1] = (byte)(angleU16);
        data[2] = (byte)(velU16 >> 8);
        data[3] = (byte)(velU16);
        data[4] = (byte)(kpU16 >> 8);
        data[5] = (byte)(kpU16);
        data[6] = (byte)(kdU16 >> 8);
        data[7] = (byte)(kdU16);

        uart.Send(AtFrame.Encode(canId, data, 8));
    }

    private int FloatToUint(float x, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        if (x > x_max) x = x_max;
        else if (x < x_min) x = x_min;
        return (int)((x - x_min) * ((1 << bits) - 1) / span);
    }
}
