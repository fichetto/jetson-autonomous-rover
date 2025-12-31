using System;
using System.Collections;
using UnityEngine;
using Unity.WebRTC;
using UnityEngine.Networking;

/// <summary>
/// WebRTC Manager for receiving stereo video stream from CLOVER rover
/// Handles connection establishment, ICE negotiation, and video track reception
/// </summary>
public class WebRTCManager : MonoBehaviour
{
    [Header("Server Configuration")]
    [SerializeField] private string serverUrl = "http://192.168.1.100:8080";
    [SerializeField] private bool autoConnect = true;
    [SerializeField] private float reconnectDelay = 3f;

    [Header("Video Output")]
    [SerializeField] private RenderTexture stereoTexture;
    [SerializeField] private Material leftEyeMaterial;
    [SerializeField] private Material rightEyeMaterial;

    // Events
    public event Action OnConnected;
    public event Action OnDisconnected;
    public event Action<string> OnError;
    public event Action<Texture> OnVideoReceived;

    // WebRTC components
    private RTCPeerConnection peerConnection;
    private MediaStream receiveStream;
    private VideoStreamTrack videoTrack;

    // State
    private bool isConnected = false;
    private bool isConnecting = false;
    private Texture2D videoTexture;

    // Video dimensions (stereo side-by-side)
    private int videoWidth = 2560;  // 1280 * 2
    private int videoHeight = 720;

    private void Awake()
    {
        // Initialize WebRTC
        WebRTC.Initialize(WebRTCSettings.LimitTextureSize);
    }

    private void Start()
    {
        if (autoConnect)
        {
            StartCoroutine(ConnectWithRetry());
        }
    }

    private void OnDestroy()
    {
        Disconnect();
        WebRTC.Dispose();
    }

    /// <summary>
    /// Connect to CLOVER teleop server
    /// </summary>
    public void Connect()
    {
        if (isConnecting || isConnected)
        {
            Debug.LogWarning("Already connected or connecting");
            return;
        }

        StartCoroutine(ConnectCoroutine());
    }

    /// <summary>
    /// Disconnect from server
    /// </summary>
    public void Disconnect()
    {
        isConnected = false;
        isConnecting = false;

        if (videoTrack != null)
        {
            videoTrack.Dispose();
            videoTrack = null;
        }

        if (receiveStream != null)
        {
            receiveStream.Dispose();
            receiveStream = null;
        }

        if (peerConnection != null)
        {
            peerConnection.Close();
            peerConnection.Dispose();
            peerConnection = null;
        }

        OnDisconnected?.Invoke();
        Debug.Log("Disconnected from CLOVER");
    }

    private IEnumerator ConnectWithRetry()
    {
        while (!isConnected)
        {
            yield return StartCoroutine(ConnectCoroutine());

            if (!isConnected)
            {
                Debug.Log($"Retrying connection in {reconnectDelay}s...");
                yield return new WaitForSeconds(reconnectDelay);
            }
        }
    }

    private IEnumerator ConnectCoroutine()
    {
        isConnecting = true;
        Debug.Log($"Connecting to {serverUrl}...");

        // Create peer connection
        var config = new RTCConfiguration
        {
            iceServers = new RTCIceServer[]
            {
                new RTCIceServer { urls = new string[] { "stun:stun.l.google.com:19302" } }
            }
        };

        peerConnection = new RTCPeerConnection(ref config);

        // Setup event handlers
        peerConnection.OnIceCandidate = candidate =>
        {
            Debug.Log($"ICE Candidate: {candidate.Candidate}");
        };

        peerConnection.OnIceConnectionChange = state =>
        {
            Debug.Log($"ICE Connection State: {state}");

            if (state == RTCIceConnectionState.Connected)
            {
                isConnected = true;
                isConnecting = false;
                OnConnected?.Invoke();
            }
            else if (state == RTCIceConnectionState.Disconnected ||
                     state == RTCIceConnectionState.Failed)
            {
                isConnected = false;
                OnDisconnected?.Invoke();
            }
        };

        peerConnection.OnTrack = e =>
        {
            Debug.Log($"Received track: {e.Track.Kind}");

            if (e.Track is VideoStreamTrack video)
            {
                videoTrack = video;
                videoTrack.OnVideoReceived += OnVideoFrameReceived;
                Debug.Log("Video track received and handler attached");
            }
        };

        // Add transceiver for receiving video
        var transceiver = peerConnection.AddTransceiver(TrackKind.Video);
        transceiver.Direction = RTCRtpTransceiverDirection.RecvOnly;

        // Create offer
        var offerOp = peerConnection.CreateOffer();
        yield return offerOp;

        if (offerOp.IsError)
        {
            OnError?.Invoke($"Create offer failed: {offerOp.Error.message}");
            isConnecting = false;
            yield break;
        }

        var offer = offerOp.Desc;

        // Set local description
        var setLocalOp = peerConnection.SetLocalDescription(ref offer);
        yield return setLocalOp;

        if (setLocalOp.IsError)
        {
            OnError?.Invoke($"Set local description failed: {setLocalOp.Error.message}");
            isConnecting = false;
            yield break;
        }

        // Wait for ICE gathering to complete
        yield return new WaitUntil(() =>
            peerConnection.GatheringState == RTCIceGatheringState.Complete);

        // Send offer to server
        var offerJson = JsonUtility.ToJson(new SDPMessage
        {
            type = "offer",
            sdp = peerConnection.LocalDescription.sdp
        });

        using (var request = new UnityWebRequest($"{serverUrl}/offer", "POST"))
        {
            byte[] bodyRaw = System.Text.Encoding.UTF8.GetBytes(offerJson);
            request.uploadHandler = new UploadHandlerRaw(bodyRaw);
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/json");

            yield return request.SendWebRequest();

            if (request.result != UnityWebRequest.Result.Success)
            {
                OnError?.Invoke($"Server request failed: {request.error}");
                isConnecting = false;
                yield break;
            }

            // Parse answer
            var answerJson = request.downloadHandler.text;
            var answer = JsonUtility.FromJson<SDPMessage>(answerJson);

            if (string.IsNullOrEmpty(answer.sdp))
            {
                OnError?.Invoke("Invalid answer from server");
                isConnecting = false;
                yield break;
            }

            // Set remote description
            var remoteDesc = new RTCSessionDescription
            {
                type = RTCSdpType.Answer,
                sdp = answer.sdp
            };

            var setRemoteOp = peerConnection.SetRemoteDescription(ref remoteDesc);
            yield return setRemoteOp;

            if (setRemoteOp.IsError)
            {
                OnError?.Invoke($"Set remote description failed: {setRemoteOp.Error.message}");
                isConnecting = false;
                yield break;
            }

            Debug.Log("WebRTC connection established!");
        }
    }

    private void OnVideoFrameReceived(Texture texture)
    {
        videoTexture = texture as Texture2D;

        if (videoTexture != null)
        {
            // Update stereo render texture
            if (stereoTexture != null)
            {
                Graphics.Blit(videoTexture, stereoTexture);
            }

            // Update eye materials if assigned
            UpdateEyeMaterials(videoTexture);

            OnVideoReceived?.Invoke(videoTexture);
        }
    }

    private void UpdateEyeMaterials(Texture2D stereoFrame)
    {
        // For side-by-side stereo:
        // Left eye: left half of texture
        // Right eye: right half of texture

        if (leftEyeMaterial != null)
        {
            leftEyeMaterial.mainTexture = stereoFrame;
            leftEyeMaterial.mainTextureScale = new Vector2(0.5f, 1f);
            leftEyeMaterial.mainTextureOffset = new Vector2(0f, 0f);
        }

        if (rightEyeMaterial != null)
        {
            rightEyeMaterial.mainTexture = stereoFrame;
            rightEyeMaterial.mainTextureScale = new Vector2(0.5f, 1f);
            rightEyeMaterial.mainTextureOffset = new Vector2(0.5f, 0f);
        }
    }

    /// <summary>
    /// Get current video texture
    /// </summary>
    public Texture2D GetVideoTexture()
    {
        return videoTexture;
    }

    /// <summary>
    /// Check if connected to server
    /// </summary>
    public bool IsConnected => isConnected;

    /// <summary>
    /// Set server URL
    /// </summary>
    public void SetServerUrl(string url)
    {
        serverUrl = url;
    }

    [Serializable]
    private class SDPMessage
    {
        public string type;
        public string sdp;
    }
}
