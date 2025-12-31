using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// Stereo Renderer for displaying side-by-side video on Quest 3
/// Maps left/right halves of stereo texture to respective eyes
/// </summary>
public class StereoRenderer : MonoBehaviour
{
    [Header("Video Source")]
    [SerializeField] private WebRTCManager webRTCManager;

    [Header("Display Configuration")]
    [SerializeField] private Camera leftEyeCamera;
    [SerializeField] private Camera rightEyeCamera;
    [SerializeField] private float displayDistance = 2f;
    [SerializeField] private float displayScale = 1.6f;
    [SerializeField] private float aspectRatio = 16f / 9f;

    [Header("Screen Quads")]
    [SerializeField] private GameObject leftScreenQuad;
    [SerializeField] private GameObject rightScreenQuad;

    [Header("Materials")]
    [SerializeField] private Material leftEyeMaterial;
    [SerializeField] private Material rightEyeMaterial;

    // Screen dimensions
    private float screenWidth;
    private float screenHeight;

    // State
    private bool isInitialized = false;

    private void Start()
    {
        // Calculate screen dimensions
        screenHeight = displayScale;
        screenWidth = screenHeight * aspectRatio;

        // Create or configure screen quads
        if (leftScreenQuad == null || rightScreenQuad == null)
        {
            CreateScreenQuads();
        }
        else
        {
            ConfigureScreenQuads();
        }

        // Subscribe to video events
        if (webRTCManager != null)
        {
            webRTCManager.OnVideoReceived += OnVideoReceived;
            webRTCManager.OnConnected += OnConnected;
            webRTCManager.OnDisconnected += OnDisconnected;
        }

        isInitialized = true;
        Debug.Log("StereoRenderer initialized");
    }

    private void OnDestroy()
    {
        if (webRTCManager != null)
        {
            webRTCManager.OnVideoReceived -= OnVideoReceived;
            webRTCManager.OnConnected -= OnConnected;
            webRTCManager.OnDisconnected -= OnDisconnected;
        }
    }

    private void CreateScreenQuads()
    {
        // Create parent container
        GameObject container = new GameObject("StereoScreen");
        container.transform.SetParent(transform);
        container.transform.localPosition = new Vector3(0, 0, displayDistance);
        container.transform.localRotation = Quaternion.identity;

        // Create left eye quad
        leftScreenQuad = CreateQuad("LeftScreen", container.transform);
        leftScreenQuad.layer = LayerMask.NameToLayer("LeftEye");

        // Create right eye quad
        rightScreenQuad = CreateQuad("RightScreen", container.transform);
        rightScreenQuad.layer = LayerMask.NameToLayer("RightEye");

        // Create materials if not assigned
        if (leftEyeMaterial == null)
        {
            leftEyeMaterial = new Material(Shader.Find("Unlit/Texture"));
            leftEyeMaterial.name = "LeftEyeMaterial";
        }

        if (rightEyeMaterial == null)
        {
            rightEyeMaterial = new Material(Shader.Find("Unlit/Texture"));
            rightEyeMaterial.name = "RightEyeMaterial";
        }

        // Apply materials
        leftScreenQuad.GetComponent<MeshRenderer>().material = leftEyeMaterial;
        rightScreenQuad.GetComponent<MeshRenderer>().material = rightEyeMaterial;

        // Configure UVs for stereo
        ConfigureStereoUVs();
    }

    private GameObject CreateQuad(string name, Transform parent)
    {
        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = name;
        quad.transform.SetParent(parent);
        quad.transform.localPosition = Vector3.zero;
        quad.transform.localRotation = Quaternion.identity;
        quad.transform.localScale = new Vector3(screenWidth, screenHeight, 1f);

        // Remove collider
        Destroy(quad.GetComponent<Collider>());

        return quad;
    }

    private void ConfigureScreenQuads()
    {
        // Scale existing quads
        leftScreenQuad.transform.localScale = new Vector3(screenWidth, screenHeight, 1f);
        rightScreenQuad.transform.localScale = new Vector3(screenWidth, screenHeight, 1f);

        // Position at display distance
        leftScreenQuad.transform.localPosition = new Vector3(0, 0, displayDistance);
        rightScreenQuad.transform.localPosition = new Vector3(0, 0, displayDistance);

        ConfigureStereoUVs();
    }

    private void ConfigureStereoUVs()
    {
        // For side-by-side stereo video:
        // Left eye shows left half (0.0 - 0.5 U)
        // Right eye shows right half (0.5 - 1.0 U)

        // Left eye UV (left half of texture)
        if (leftEyeMaterial != null)
        {
            leftEyeMaterial.mainTextureScale = new Vector2(0.5f, 1f);
            leftEyeMaterial.mainTextureOffset = new Vector2(0f, 0f);
        }

        // Right eye UV (right half of texture)
        if (rightEyeMaterial != null)
        {
            rightEyeMaterial.mainTextureScale = new Vector2(0.5f, 1f);
            rightEyeMaterial.mainTextureOffset = new Vector2(0.5f, 0f);
        }
    }

    private void OnVideoReceived(Texture texture)
    {
        // Update materials with new texture
        if (leftEyeMaterial != null)
        {
            leftEyeMaterial.mainTexture = texture;
        }

        if (rightEyeMaterial != null)
        {
            rightEyeMaterial.mainTexture = texture;
        }
    }

    private void OnConnected()
    {
        Debug.Log("StereoRenderer: Video stream connected");
        SetScreenVisibility(true);
    }

    private void OnDisconnected()
    {
        Debug.Log("StereoRenderer: Video stream disconnected");
        // Optionally show "disconnected" message or pattern
    }

    private void SetScreenVisibility(bool visible)
    {
        if (leftScreenQuad != null)
            leftScreenQuad.SetActive(visible);

        if (rightScreenQuad != null)
            rightScreenQuad.SetActive(visible);
    }

    /// <summary>
    /// Adjust display distance (how far the virtual screen appears)
    /// </summary>
    public void SetDisplayDistance(float distance)
    {
        displayDistance = Mathf.Clamp(distance, 0.5f, 10f);

        if (leftScreenQuad != null)
            leftScreenQuad.transform.localPosition = new Vector3(0, 0, displayDistance);

        if (rightScreenQuad != null)
            rightScreenQuad.transform.localPosition = new Vector3(0, 0, displayDistance);
    }

    /// <summary>
    /// Adjust display scale (size of virtual screen)
    /// </summary>
    public void SetDisplayScale(float scale)
    {
        displayScale = Mathf.Clamp(scale, 0.5f, 5f);
        screenHeight = displayScale;
        screenWidth = screenHeight * aspectRatio;

        if (leftScreenQuad != null)
            leftScreenQuad.transform.localScale = new Vector3(screenWidth, screenHeight, 1f);

        if (rightScreenQuad != null)
            rightScreenQuad.transform.localScale = new Vector3(screenWidth, screenHeight, 1f);
    }

    /// <summary>
    /// Switch between stereo modes
    /// </summary>
    public void SetStereoMode(StereoMode mode)
    {
        switch (mode)
        {
            case StereoMode.SideBySide:
                // Left half for left eye, right half for right eye
                leftEyeMaterial.mainTextureScale = new Vector2(0.5f, 1f);
                leftEyeMaterial.mainTextureOffset = new Vector2(0f, 0f);
                rightEyeMaterial.mainTextureScale = new Vector2(0.5f, 1f);
                rightEyeMaterial.mainTextureOffset = new Vector2(0.5f, 0f);
                break;

            case StereoMode.OverUnder:
                // Top half for left eye, bottom half for right eye
                leftEyeMaterial.mainTextureScale = new Vector2(1f, 0.5f);
                leftEyeMaterial.mainTextureOffset = new Vector2(0f, 0.5f);
                rightEyeMaterial.mainTextureScale = new Vector2(1f, 0.5f);
                rightEyeMaterial.mainTextureOffset = new Vector2(0f, 0f);
                break;

            case StereoMode.Mono:
                // Full texture for both eyes (2D mode)
                leftEyeMaterial.mainTextureScale = new Vector2(1f, 1f);
                leftEyeMaterial.mainTextureOffset = new Vector2(0f, 0f);
                rightEyeMaterial.mainTextureScale = new Vector2(1f, 1f);
                rightEyeMaterial.mainTextureOffset = new Vector2(0f, 0f);
                break;
        }
    }
}

public enum StereoMode
{
    SideBySide,  // Left/Right halves
    OverUnder,   // Top/Bottom halves
    Mono         // 2D (same image both eyes)
}
