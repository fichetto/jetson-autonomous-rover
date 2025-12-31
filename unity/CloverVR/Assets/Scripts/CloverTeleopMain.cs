using System;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// Main controller for CLOVER VR Teleoperation on Quest 3
/// Coordinates video streaming, controller input, and UI
/// </summary>
public class CloverTeleopMain : MonoBehaviour
{
    [Header("Components")]
    [SerializeField] private WebRTCManager webRTCManager;
    [SerializeField] private ControllerManager controllerManager;
    [SerializeField] private StereoRenderer stereoRenderer;

    [Header("Configuration")]
    [SerializeField] private string roverIP = "192.168.1.100";
    [SerializeField] private int videoPort = 8080;
    [SerializeField] private int controlPort = 8081;

    [Header("UI Elements")]
    [SerializeField] private Canvas statusCanvas;
    [SerializeField] private TextMeshProUGUI statusText;
    [SerializeField] private TextMeshProUGUI speedText;
    [SerializeField] private TextMeshProUGUI batteryText;
    [SerializeField] private Image connectionIndicator;

    [Header("Colors")]
    [SerializeField] private Color connectedColor = Color.green;
    [SerializeField] private Color disconnectedColor = Color.red;
    [SerializeField] private Color connectingColor = Color.yellow;

    // State
    private ConnectionState videoState = ConnectionState.Disconnected;
    private ConnectionState controlState = ConnectionState.Disconnected;
    private RoverState currentRoverState;

    // Settings (persisted)
    private const string PREF_ROVER_IP = "CloverRoverIP";

    private void Awake()
    {
        // Load saved settings
        roverIP = PlayerPrefs.GetString(PREF_ROVER_IP, roverIP);
    }

    private void Start()
    {
        // Configure server URLs
        ConfigureServers();

        // Subscribe to events
        SubscribeToEvents();

        // Initial UI update
        UpdateUI();

        Debug.Log("CLOVER VR Teleop initialized");
        Debug.Log($"Rover IP: {roverIP}");
    }

    private void OnDestroy()
    {
        UnsubscribeFromEvents();
    }

    private void Update()
    {
        // Handle special input
        HandleInput();
    }

    private void ConfigureServers()
    {
        string videoUrl = $"http://{roverIP}:{videoPort}";
        string controlUrl = $"ws://{roverIP}:{controlPort}/ws";

        if (webRTCManager != null)
        {
            webRTCManager.SetServerUrl(videoUrl);
        }

        if (controllerManager != null)
        {
            controllerManager.SetServerUrl(controlUrl);
        }
    }

    private void SubscribeToEvents()
    {
        if (webRTCManager != null)
        {
            webRTCManager.OnConnected += OnVideoConnected;
            webRTCManager.OnDisconnected += OnVideoDisconnected;
            webRTCManager.OnError += OnVideoError;
        }

        if (controllerManager != null)
        {
            controllerManager.OnConnected += OnControlConnected;
            controllerManager.OnDisconnected += OnControlDisconnected;
            controllerManager.OnRoverStateReceived += OnRoverStateReceived;
            controllerManager.OnError += OnControlError;
        }
    }

    private void UnsubscribeFromEvents()
    {
        if (webRTCManager != null)
        {
            webRTCManager.OnConnected -= OnVideoConnected;
            webRTCManager.OnDisconnected -= OnVideoDisconnected;
            webRTCManager.OnError -= OnVideoError;
        }

        if (controllerManager != null)
        {
            controllerManager.OnConnected -= OnControlConnected;
            controllerManager.OnDisconnected -= OnControlDisconnected;
            controllerManager.OnRoverStateReceived -= OnRoverStateReceived;
            controllerManager.OnError -= OnControlError;
        }
    }

    private void HandleInput()
    {
        // B button = Emergency Stop
        if (OVRInput.GetDown(OVRInput.Button.Two)) // B button
        {
            EmergencyStop();
        }

        // Y button = Toggle UI visibility
        if (OVRInput.GetDown(OVRInput.Button.Four)) // Y button
        {
            ToggleUI();
        }

        // Thumbstick click (both) = Recenter view
        if (OVRInput.GetDown(OVRInput.Button.PrimaryThumbstick) &&
            OVRInput.GetDown(OVRInput.Button.SecondaryThumbstick))
        {
            RecenterView();
        }
    }

    // ==================== Connection Events ====================

    private void OnVideoConnected()
    {
        videoState = ConnectionState.Connected;
        Debug.Log("Video stream connected");
        UpdateUI();
    }

    private void OnVideoDisconnected()
    {
        videoState = ConnectionState.Disconnected;
        Debug.Log("Video stream disconnected");
        UpdateUI();
    }

    private void OnVideoError(string error)
    {
        Debug.LogError($"Video error: {error}");
        ShowStatus($"Video Error: {error}");
    }

    private void OnControlConnected()
    {
        controlState = ConnectionState.Connected;
        Debug.Log("Control connected");
        UpdateUI();
    }

    private void OnControlDisconnected()
    {
        controlState = ConnectionState.Disconnected;
        Debug.Log("Control disconnected");
        UpdateUI();
    }

    private void OnControlError(string error)
    {
        Debug.LogError($"Control error: {error}");
        ShowStatus($"Control Error: {error}");
    }

    private void OnRoverStateReceived(RoverState state)
    {
        currentRoverState = state;
        UpdateRoverDisplay();
    }

    // ==================== UI Updates ====================

    private void UpdateUI()
    {
        // Update connection indicator
        if (connectionIndicator != null)
        {
            if (videoState == ConnectionState.Connected && controlState == ConnectionState.Connected)
            {
                connectionIndicator.color = connectedColor;
            }
            else if (videoState == ConnectionState.Connecting || controlState == ConnectionState.Connecting)
            {
                connectionIndicator.color = connectingColor;
            }
            else
            {
                connectionIndicator.color = disconnectedColor;
            }
        }

        // Update status text
        if (statusText != null)
        {
            string videoStatus = videoState.ToString();
            string controlStatus = controlState.ToString();
            statusText.text = $"Video: {videoStatus}\nControl: {controlStatus}";
        }
    }

    private void UpdateRoverDisplay()
    {
        if (currentRoverState == null) return;

        // Update speed display
        if (speedText != null)
        {
            float speed = Mathf.Sqrt(
                currentRoverState.vx * currentRoverState.vx +
                currentRoverState.vy * currentRoverState.vy
            );
            speedText.text = $"Speed: {speed:F2} m/s";
        }

        // Update battery display
        if (batteryText != null)
        {
            float voltage = currentRoverState.battery_voltage / 1000f; // mV to V
            string batteryIcon = voltage > 11f ? "🔋" : voltage > 10f ? "🪫" : "⚠️";
            batteryText.text = $"{batteryIcon} {voltage:F1}V";
        }
    }

    private void ShowStatus(string message)
    {
        if (statusText != null)
        {
            statusText.text = message;
        }
        Debug.Log(message);
    }

    // ==================== Actions ====================

    /// <summary>
    /// Emergency stop - immediately halt the rover
    /// </summary>
    public void EmergencyStop()
    {
        if (controllerManager != null)
        {
            controllerManager.EmergencyStop();
        }

        ShowStatus("⚠️ EMERGENCY STOP");
        Debug.LogWarning("Emergency stop activated!");
    }

    /// <summary>
    /// Toggle status UI visibility
    /// </summary>
    public void ToggleUI()
    {
        if (statusCanvas != null)
        {
            statusCanvas.enabled = !statusCanvas.enabled;
        }
    }

    /// <summary>
    /// Recenter the VR view
    /// </summary>
    public void RecenterView()
    {
        OVRManager.display.RecenterPose();
        Debug.Log("View recentered");
    }

    /// <summary>
    /// Reconnect to rover
    /// </summary>
    public void Reconnect()
    {
        if (webRTCManager != null)
        {
            webRTCManager.Disconnect();
            webRTCManager.Connect();
        }

        if (controllerManager != null)
        {
            controllerManager.Disconnect();
            controllerManager.Connect();
        }
    }

    /// <summary>
    /// Update rover IP address
    /// </summary>
    public void SetRoverIP(string ip)
    {
        roverIP = ip;
        PlayerPrefs.SetString(PREF_ROVER_IP, ip);
        PlayerPrefs.Save();

        ConfigureServers();
        Reconnect();
    }

    /// <summary>
    /// Adjust stereo screen distance
    /// </summary>
    public void AdjustScreenDistance(float delta)
    {
        if (stereoRenderer != null)
        {
            float currentDistance = 2f; // Would need to expose this
            stereoRenderer.SetDisplayDistance(currentDistance + delta);
        }
    }

    /// <summary>
    /// Adjust stereo screen scale
    /// </summary>
    public void AdjustScreenScale(float delta)
    {
        if (stereoRenderer != null)
        {
            float currentScale = 1.6f; // Would need to expose this
            stereoRenderer.SetDisplayScale(currentScale + delta);
        }
    }
}

public enum ConnectionState
{
    Disconnected,
    Connecting,
    Connected
}
