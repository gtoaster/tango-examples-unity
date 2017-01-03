using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Tango;


public class PointCloudParticles : MonoBehaviour, ITangoPointCloud
{

    public float OccSizeMin = 0.01f;
    public float OccSizeMax = 0.02f;
    public float DistanceNear = 0.5f;
    public float DistanceFar = 3.0f;
    public float TimeToUpdatePS = 0.5f;

    private ParticleSystem cloudParticles;  
    private ParticleSystem.Particle[] parts;
    private float timeSinceUpdatePS = 0.0f;
    private bool shouldEmitPS = false;
    private float DistanceNearSq = 0.25f;
    private float DistanceFarSq = 9.0f;

    /// <summary>
    /// If set, the point cloud will be transformed to be in the Area 
    /// Description frame.
    /// </summary>
    private bool m_useAreaDescriptionPose;  

    /// <summary>
    /// The points of the point cloud, in world space.
    /// 
    /// Note that not every member of this array will be filled out. See
    /// m_pointsCount.
    /// </summary>
    [HideInInspector]
    public Vector3[] m_points;
    
    /// <summary>
    /// The number of points in m_points.
    /// </summary>
    [HideInInspector]
    public int m_pointsCount = 0;
    
    /// <summary>
    /// The average depth (relative to the depth camera).
    /// </summary>
    [HideInInspector]
    public float m_overallZ = 0.0f;
    
    /// <summary>
    /// Time between the last two depth events.
    /// </summary>
    [HideInInspector]
    public float m_depthDeltaTime = 0.0f;
    
    /// <summary>
    /// The maximum points displayed.  Just some constant value.
    /// </summary>
    private const int MAX_POINT_COUNT = 61440;


    private TangoApplication m_tangoApplication;
    
    // Matrices for transforming pointcloud to world coordinates.
    // This equation will take account of the camera sensors extrinsic.
    // Full equation is:
    // Matrix4x4 unityWorldTDepthCamera = 
    // m_unityWorldTStartService * startServiceTDevice * Matrix4x4.Inverse(m_imuTDevice) * m_imuTDepthCamera;
    private Matrix4x4 m_unityWorldTStartService;
    private Matrix4x4 m_imuTDevice;
    private Matrix4x4 m_imuTDepthCamera;
    
    // Matrix for transforming the Unity camera space to the color camera space.
    private Matrix4x4 m_colorCameraTUnityCamera;
    
    /// <summary>
    /// Color camera intrinsics.
    /// </summary>
    private TangoCameraIntrinsics m_colorCameraIntrinsics;
    
    /// <summary>
    /// If the camera data has already been set up.
    /// </summary>
    private bool m_cameraDataSetUp;
    
    /// <summary>
    /// The Tango timestamp from the last update of m_points.
    /// </summary>
    private double m_depthTimestamp;
    

    // Pose controller from which the offset is queried.
    private TangoDeltaPoseController m_tangoDeltaPoseController;

    /// @cond
    /// <summary>
    /// Use this for initialization.
    /// </summary>
    public void Start() 
    {
        m_tangoApplication = FindObjectOfType<TangoApplication>();
        m_tangoApplication.Register(this);
        m_tangoDeltaPoseController = FindObjectOfType<TangoDeltaPoseController>();
        m_unityWorldTStartService.SetColumn(0, new Vector4(1.0f, 0.0f, 0.0f, 0.0f));
        m_unityWorldTStartService.SetColumn(1, new Vector4(0.0f, 0.0f, 1.0f, 0.0f));
        m_unityWorldTStartService.SetColumn(2, new Vector4(0.0f, 1.0f, 0.0f, 0.0f));
        m_unityWorldTStartService.SetColumn(3, new Vector4(0.0f, 0.0f, 0.0f, 1.0f));
        
        // Constant matrix converting Unity world frame frame to device frame.
        m_colorCameraTUnityCamera.SetColumn(0, new Vector4(1.0f, 0.0f, 0.0f, 0.0f));
        m_colorCameraTUnityCamera.SetColumn(1, new Vector4(0.0f, -1.0f, 0.0f, 0.0f));
        m_colorCameraTUnityCamera.SetColumn(2, new Vector4(0.0f, 0.0f, 1.0f, 0.0f));
        m_colorCameraTUnityCamera.SetColumn(3, new Vector4(0.0f, 0.0f, 0.0f, 1.0f));
        
        // Assign triangles, note: this is just for visualizing point in the mesh data.
        m_points = new Vector3[MAX_POINT_COUNT];

        m_useAreaDescriptionPose = m_tangoApplication.m_enableAreaDescriptions;
        cloudParticles = GetComponent<ParticleSystem>();

        DistanceNearSq = DistanceNear * DistanceNear;
        DistanceFarSq = DistanceFar * DistanceFar;
    }
    
    /// <summary>
    /// Unity callback when the component gets destroyed.
    /// </summary>
    public void OnDestroy()
    {
        m_tangoApplication.Unregister(this);
    }
    
    /// <summary>
    /// Callback that gets called when depth is available from the Tango Service.
    /// </summary>
    /// <param name="pointCloud">Depth information from Tango.</param>
    
    public void OnTangoPointCloudAvailable(TangoPointCloudData pointCloud)
    {
        if (!shouldEmitPS) {
            return;
        }
        
        // Calculate the time since the last successful depth data
        // collection.
        if (m_depthTimestamp != 0.0)
        {
            m_depthDeltaTime = (float)((pointCloud.m_timestamp - m_depthTimestamp) * 1000.0);
        }
        
        // Fill in the data to draw the point cloud.
        m_pointsCount = pointCloud.m_numPoints;
        if (m_pointsCount > 0)
        {
            _SetUpCameraData();
            TangoCoordinateFramePair pair;
            TangoPoseData poseData = new TangoPoseData();
            
            // Query pose to transform point cloud to world coordinates, here we are using the timestamp
            // that we get from depth.
            if (m_useAreaDescriptionPose)
            {
                pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_AREA_DESCRIPTION;
                pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_DEVICE;
            }
            else
            {
                pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_START_OF_SERVICE;
                pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_DEVICE;
            }
            
            PoseProvider.GetPoseAtTime(poseData, pointCloud.m_timestamp, pair);
            if (poseData.status_code != TangoEnums.TangoPoseStatusType.TANGO_POSE_VALID)
            {
                return;
            }
            
            Matrix4x4 startServiceTDevice = poseData.ToMatrix4x4();
            
            // The transformation matrix that represents the pointcloud's pose. 
            // Explanation: 
            // The pointcloud which is in Depth camera's frame, is put in unity world's 
            // coordinate system(wrt unity world).
            // Then we are extracting the position and rotation from uwTuc matrix and applying it to 
            // the PointCloud's transform.
            Matrix4x4 unityWorldTDepthCamera = m_unityWorldTStartService * startServiceTDevice * Matrix4x4.Inverse(m_imuTDevice) * m_imuTDepthCamera;
            transform.position = Vector3.zero;
            transform.rotation = Quaternion.identity;
            
            // Add offset to the pointcloud depending on the offset from TangoDeltaPoseController
            Matrix4x4 unityWorldOffsetTDepthCamera;
            if (m_tangoDeltaPoseController != null)
            {
                unityWorldOffsetTDepthCamera = m_tangoDeltaPoseController.UnityWorldOffset * unityWorldTDepthCamera;
            }
            else
            {
                unityWorldOffsetTDepthCamera = unityWorldTDepthCamera;
            }
            
            // Converting points array to world space.
            int numDepthPoints = 0;
            m_overallZ = 0;
            for (int i = 0; i < m_pointsCount; ++i)
            {
                Vector3 point = pointCloud[i];
                m_points[i] = unityWorldOffsetTDepthCamera.MultiplyPoint3x4(point);
                m_overallZ += point.z;
                numDepthPoints += 1;
            }
            
            m_overallZ = m_overallZ / m_pointsCount;
            m_depthTimestamp = pointCloud.m_timestamp;
            
            // Generate particles based on world space point cloud.
            // Filter by distance and randomize size based on min/max settings.
            Vector3 camPos = Camera.main.transform.position;
            int processed = 0;
            bool depthok = false;
            if (shouldEmitPS) {
                if (cloudParticles != null) {
                    cloudParticles.Clear();

                    int maxPoints = Mathf.Min(30000, numDepthPoints);                   
                    for (int i=0; i < numDepthPoints; i++) {
                        // camera to depth point sample
                        Vector3 cam2PointVec = m_points[i] - camPos;
                        float sqrDist = cam2PointVec.sqrMagnitude;
                        depthok = sqrDist < DistanceFarSq && sqrDist > DistanceNearSq;

                        if (depthok) {
                            float rOccSize = Random.Range(OccSizeMin, OccSizeMax);
                            cloudParticles.Emit(m_points[i], Vector3.zero, rOccSize, 5.0f, Color.gray);
                            processed += 1;
                        }
                    }
                }
                shouldEmitPS = false;
                timeSinceUpdatePS = 0.0f;
                Debug.Log(string.Format("Particle count for occlusion: {0}", processed));
            }           
        }
        else
        {
            m_overallZ = 0;
        }
        
    }
    
    void Update() {
        if (!shouldEmitPS) {
            timeSinceUpdatePS += Time.deltaTime;
            if (timeSinceUpdatePS >= TimeToUpdatePS) {
                shouldEmitPS = true;
            }
        }
    }
    


    /// <summary>
    /// Sets up extrinsic matrixes and camera intrinsics for this hardware.
    /// </summary>
    private void _SetUpCameraData()
    {
        if (m_cameraDataSetUp)
        {
            return;
        }
        
        double timestamp = 0.0;
        TangoCoordinateFramePair pair;
        TangoPoseData poseData = new TangoPoseData();
        
        // Query the extrinsics between IMU and device frame.
        pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_IMU;
        pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_DEVICE;
        PoseProvider.GetPoseAtTime(poseData, timestamp, pair);
        m_imuTDevice = poseData.ToMatrix4x4();
        
        // Query the extrinsics between IMU and depth camera frame.
        pair.baseFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_IMU;
        pair.targetFrame = TangoEnums.TangoCoordinateFrameType.TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
        PoseProvider.GetPoseAtTime(poseData, timestamp, pair);
        m_imuTDepthCamera = poseData.ToMatrix4x4();
        
        // Also get the camera intrinsics
        m_colorCameraIntrinsics = new TangoCameraIntrinsics();
        VideoOverlayProvider.GetIntrinsics(TangoEnums.TangoCameraId.TANGO_CAMERA_COLOR, m_colorCameraIntrinsics);
        
        m_cameraDataSetUp = true;
    }
}
