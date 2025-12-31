using System;
using System.Collections;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using NativeWebSocket;

/// <summary>
/// Quest 3 Controller Manager for CLOVER rover control
/// Sends joystick input to rover via WebSocket
/// </summary>
public class ControllerManager : MonoBehaviour
{
    [Header("Server Configuration")]
    [SerializeField] private string controlServerUrl = "ws://192.168.1.100:8081/ws";
    [SerializeField] private bool autoConnect = true;
    [SerializeField] private float sendRate = 20f; // Hz

    [Header("Input Settings")]
    [SerializeField] private float deadzone = 0.1f;
    [SerializeField] private bool invertY = false;

    [Header("Haptics")]
    [SerializeField] private bool enableHaptics = true;
    [SerializeField] private float hapticIntensity = 0.3f;

    // Events
    public event Action OnConnected;
    public event Action OnDisconnected;
    public event Action<RoverState> OnRoverStateReceived;
    public event Action<string> OnError;

    // WebSocket
    private WebSocket webSocket;
    private bool isConnected = false;

    // Controller state
    private ControllerState controllerState = new ControllerState();
    private float lastSendTime = 0f;

    // Input actions (from XR Input)
    private InputAction leftThumbstick;
    private InputAction rightThumbstick;
    private InputAction leftTrigger;
    private InputAction rightTrigger;
    private InputAction leftGrip;
    private InputAction rightGrip;
    private InputAction buttonA;
    private InputAction buttonB;
    private InputAction buttonX;
    private InputAction buttonY;

    private void Awake()
    {
        SetupInputActions();
    }

    private void Start()
    {
        if (autoConnect)
        {
            Connect();
        }
    }

    private void Update()
    {
        // Process WebSocket messages
        #if !UNITY_WEBGL || UNITY_EDITOR
        webSocket?.DispatchMessageQueue();
        #endif

        // Send controller state at fixed rate
        if (isConnected && Time.time - lastSendTime >= 1f / sendRate)
        {
            UpdateControllerState();
            SendControllerState();
            lastSendTime = Time.time;
        }
    }

    private void OnDestroy()
    {
        Disconnect();
    }

    private void SetupInputActions()
    {
        // Setup XR controller input actions
        // These bindings work with Quest controllers through Unity's XR Input System

        leftThumbstick = new InputAction("LeftThumbstick", InputActionType.Value);
        leftThumbstick.AddBinding("<XRController>{LeftHand}/thumbstick");
        leftThumbstick.Enable();

        rightThumbstick = new InputAction("RightThumbstick", InputActionType.Value);
        rightThumbstick.AddBinding("<XRController>{RightHand}/thumbstick");
        rightThumbstick.Enable();

        leftTrigger = new InputAction("LeftTrigger", InputActionType.Value);
        leftTrigger.AddBinding("<XRController>{LeftHand}/trigger");
        leftTrigger.Enable();

        rightTrigger = new InputAction("RightTrigger", InputActionType.Value);
        rightTrigger.AddBinding("<XRController>{RightHand}/trigger");
        rightTrigger.Enable();

        leftGrip = new InputAction("LeftGrip", InputActionType.Value);
        leftGrip.AddBinding("<XRController>{LeftHand}/grip");
        leftGrip.Enable();

        rightGrip = new InputAction("RightGrip", InputActionType.Value);
        rightGrip.AddBinding("<XRController>{RightHand}/grip");
        rightGrip.Enable();

        buttonA = new InputAction("ButtonA", InputActionType.Button);
        buttonA.AddBinding("<XRController>{RightHand}/primaryButton");
        buttonA.Enable();

        buttonB = new InputAction("ButtonB", InputActionType.Button);
        buttonB.AddBinding("<XRController>{RightHand}/secondaryButton");
        buttonB.Enable();

        buttonX = new InputAction("ButtonX", InputActionType.Button);
        buttonX.AddBinding("<XRController>{LeftHand}/primaryButton");
        buttonX.Enable();

        buttonY = new InputAction("ButtonY", InputActionType.Button);
        buttonY.AddBinding("<XRController>{LeftHand}/secondaryButton");
        buttonY.Enable();
    }

    /// <summary>
    /// Connect to CLOVER control server
    /// </summary>
    public async void Connect()
    {
        if (webSocket != null && webSocket.State == WebSocketState.Open)
        {
            Debug.LogWarning("Already connected");
            return;
        }

        Debug.Log($"Connecting to {controlServerUrl}...");

        webSocket = new WebSocket(controlServerUrl);

        webSocket.OnOpen += () =>
        {
            Debug.Log("Connected to CLOVER control server");
            isConnected = true;
            OnConnected?.Invoke();

            // Enable teleop mode
            SendMessage(new { type = "set_mode", mode = "teleop" });
        };

        webSocket.OnMessage += (bytes) =>
        {
            var message = Encoding.UTF8.GetString(bytes);
            ProcessMessage(message);
        };

        webSocket.OnError += (error) =>
        {
            Debug.LogError($"WebSocket error: {error}");
            OnError?.Invoke(error);
        };

        webSocket.OnClose += (code) =>
        {
            Debug.Log($"Disconnected from server (code: {code})");
            isConnected = false;
            OnDisconnected?.Invoke();

            // Auto reconnect
            if (autoConnect)
            {
                StartCoroutine(ReconnectCoroutine());
            }
        };

        await webSocket.Connect();
    }

    /// <summary>
    /// Disconnect from server
    /// </summary>
    public async void Disconnect()
    {
        if (webSocket != null)
        {
            // Disable teleop before disconnecting
            SendMessage(new { type = "set_mode", mode = "disabled" });

            await webSocket.Close();
            webSocket = null;
        }

        isConnected = false;
    }

    private IEnumerator ReconnectCoroutine()
    {
        yield return new WaitForSeconds(3f);

        if (!isConnected)
        {
            Debug.Log("Attempting to reconnect...");
            Connect();
        }
    }

    private void UpdateControllerState()
    {
        // Read thumbsticks
        Vector2 leftStick = leftThumbstick.ReadValue<Vector2>();
        Vector2 rightStick = rightThumbstick.ReadValue<Vector2>();

        // Apply deadzone
        leftStick = ApplyDeadzone(leftStick);
        rightStick = ApplyDeadzone(rightStick);

        // Update state
        controllerState.left_thumbstick_x = leftStick.x;
        controllerState.left_thumbstick_y = invertY ? -leftStick.y : leftStick.y;
        controllerState.right_thumbstick_x = rightStick.x;
        controllerState.right_thumbstick_y = invertY ? -rightStick.y : rightStick.y;

        controllerState.left_trigger = leftTrigger.ReadValue<float>();
        controllerState.right_trigger = rightTrigger.ReadValue<float>();
        controllerState.left_grip = leftGrip.ReadValue<float>();
        controllerState.right_grip = rightGrip.ReadValue<float>();

        controllerState.left_button_x = buttonX.IsPressed();
        controllerState.left_button_y = buttonY.IsPressed();
        controllerState.right_button_a = buttonA.IsPressed();
        controllerState.right_button_b = buttonB.IsPressed();

        // Head tracking
        var headPose = InputTracking.GetLocalPosition(XRNode.Head);
        var headRot = InputTracking.GetLocalRotation(XRNode.Head).eulerAngles;
        controllerState.head_pitch = headRot.x;
        controllerState.head_yaw = headRot.y;
        controllerState.head_roll = headRot.z;

        controllerState.timestamp = Time.time;
    }

    private Vector2 ApplyDeadzone(Vector2 input)
    {
        float magnitude = input.magnitude;

        if (magnitude < deadzone)
        {
            return Vector2.zero;
        }

        // Rescale to full range
        float scale = (magnitude - deadzone) / (1f - deadzone);
        return input.normalized * scale;
    }

    private void SendControllerState()
    {
        if (!isConnected) return;

        var message = new ControllerMessage
        {
            type = "controller_state",
            left_thumbstick_x = controllerState.left_thumbstick_x,
            left_thumbstick_y = controllerState.left_thumbstick_y,
            left_trigger = controllerState.left_trigger,
            left_grip = controllerState.left_grip,
            left_button_x = controllerState.left_button_x,
            left_button_y = controllerState.left_button_y,
            right_thumbstick_x = controllerState.right_thumbstick_x,
            right_thumbstick_y = controllerState.right_thumbstick_y,
            right_trigger = controllerState.right_trigger,
            right_grip = controllerState.right_grip,
            right_button_a = controllerState.right_button_a,
            right_button_b = controllerState.right_button_b,
            head_pitch = controllerState.head_pitch,
            head_yaw = controllerState.head_yaw,
            head_roll = controllerState.head_roll
        };

        SendMessage(message);
    }

    private async void SendMessage(object message)
    {
        if (webSocket?.State == WebSocketState.Open)
        {
            string json = JsonUtility.ToJson(message);
            await webSocket.SendText(json);
        }
    }

    private void ProcessMessage(string json)
    {
        try
        {
            // Check message type
            if (json.Contains("\"type\":\"rover_state\""))
            {
                var state = JsonUtility.FromJson<RoverState>(json);
                OnRoverStateReceived?.Invoke(state);

                // Haptic feedback based on speed
                if (enableHaptics)
                {
                    float speed = Mathf.Sqrt(state.vx * state.vx + state.vy * state.vy);
                    ApplyHapticFeedback(speed);
                }
            }
            else if (json.Contains("\"type\":\"emergency_stop\""))
            {
                Debug.LogWarning("Emergency stop triggered!");
                // Strong haptic feedback
                TriggerHapticPulse(1f, 0.5f);
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error processing message: {e.Message}");
        }
    }

    private void ApplyHapticFeedback(float speed)
    {
        // Subtle vibration proportional to speed
        float intensity = Mathf.Clamp01(speed * hapticIntensity);

        if (intensity > 0.05f)
        {
            // Apply to both controllers
            OVRInput.SetControllerVibration(intensity * 0.5f, intensity, OVRInput.Controller.LTouch);
            OVRInput.SetControllerVibration(intensity * 0.5f, intensity, OVRInput.Controller.RTouch);
        }
    }

    private void TriggerHapticPulse(float intensity, float duration)
    {
        StartCoroutine(HapticPulseCoroutine(intensity, duration));
    }

    private IEnumerator HapticPulseCoroutine(float intensity, float duration)
    {
        OVRInput.SetControllerVibration(1f, intensity, OVRInput.Controller.LTouch);
        OVRInput.SetControllerVibration(1f, intensity, OVRInput.Controller.RTouch);

        yield return new WaitForSeconds(duration);

        OVRInput.SetControllerVibration(0f, 0f, OVRInput.Controller.LTouch);
        OVRInput.SetControllerVibration(0f, 0f, OVRInput.Controller.RTouch);
    }

    /// <summary>
    /// Trigger emergency stop
    /// </summary>
    public void EmergencyStop()
    {
        SendMessage(new { type = "set_mode", mode = "disabled" });
        Debug.LogWarning("Emergency stop sent!");
    }

    /// <summary>
    /// Set control mode
    /// </summary>
    public void SetMode(string mode)
    {
        SendMessage(new { type = "set_mode", mode = mode });
    }

    public bool IsConnected => isConnected;

    public void SetServerUrl(string url)
    {
        controlServerUrl = url;
    }

    // Data classes
    [Serializable]
    private class ControllerState
    {
        public float left_thumbstick_x;
        public float left_thumbstick_y;
        public float left_trigger;
        public float left_grip;
        public bool left_button_x;
        public bool left_button_y;
        public bool left_thumbstick_click;

        public float right_thumbstick_x;
        public float right_thumbstick_y;
        public float right_trigger;
        public float right_grip;
        public bool right_button_a;
        public bool right_button_b;
        public bool right_thumbstick_click;

        public float head_pitch;
        public float head_yaw;
        public float head_roll;

        public float timestamp;
    }

    [Serializable]
    private class ControllerMessage
    {
        public string type;
        public float left_thumbstick_x;
        public float left_thumbstick_y;
        public float left_trigger;
        public float left_grip;
        public bool left_button_x;
        public bool left_button_y;
        public float right_thumbstick_x;
        public float right_thumbstick_y;
        public float right_trigger;
        public float right_grip;
        public bool right_button_a;
        public bool right_button_b;
        public float head_pitch;
        public float head_yaw;
        public float head_roll;
    }
}

[Serializable]
public class RoverState
{
    public string mode;
    public float vx;
    public float vy;
    public float wz;
    public float battery_voltage;
    public bool connected;
    public float timestamp;
}
