//#define HUNT_DEPTH_PIXEL_GRID
#define USE_CENTRE_DEPTH_IMAGE
using UnityEngine.XR.WSA;
using System;
using System.Linq;
using UnityEngine;
using System.Threading;

#if ENABLE_WINMD_SUPPORT
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using Windows.Foundation;
using System.Threading.Tasks;
using Windows.Perception.Spatial;
using System.Runtime.InteropServices;
using Windows.Media.FaceAnalysis;
using Windows.Graphics.Imaging;
using uVector3 = UnityEngine.Vector3;
using wVector3 = System.Numerics.Vector3;
using wVector4 = System.Numerics.Vector4;
using wMatrix4x4 = System.Numerics.Matrix4x4;

[ComImport]
[Guid("5B0D3235-4DBA-4D44-865E-8F1D0E4FD04D")]
[InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
unsafe interface IMemoryBufferByteAccess
{
    void GetBuffer(out byte* buffer, out uint capacity);
}

#endif // ENABLE_WINMD_SUPPORT

public class Placeholder : MonoBehaviour
{
    // A Unity text mesh that I can print some diagnostics to.
    public TextMesh textMesh;

    // A Unity game object (small sphere e.g.) that I can use to mark the position of one face.
    public GameObject faceMarker;

    void Start()
    {
#if ENABLE_WINMD_SUPPORT

        // Not awaiting this...let it go.
        this.ProcessingLoopAsync();

#endif // ENABLE_WINMD_SUPPORT
    }

#if ENABLE_WINMD_SUPPORT
    /// <summary>
    /// This is just one big lump of code right now which should be factored out into some kind of
    /// 'frame reader' class which can then be subclassed for depth frame and video frame but
    /// it was handy to have it like this while I experimented with it - the intention was
    /// to tidy it up if I could get it doing more or less what I wanted :-)
    /// </summary>
    async Task ProcessingLoopAsync()
    {
        var depthMediaCapture = await this.GetMediaCaptureForDescriptionAsync(
            MediaFrameSourceKind.Depth, 448, 450, 15);

        var depthFrameReader = await depthMediaCapture.Item1.CreateFrameReaderAsync(depthMediaCapture.Item2);

        depthFrameReader.AcquisitionMode = MediaFrameReaderAcquisitionMode.Realtime;

        MediaFrameReference lastDepthFrame = null;

        long depthFrameCount = 0;
        float centrePointDepthInMetres = 0.0f;

        // Expecting this to run at 1fps although the API (seems to) reports that it runs at 15fps
        TypedEventHandler<MediaFrameReader, MediaFrameArrivedEventArgs> depthFrameHandler =
            (sender, args) =>
            {
                using (var depthFrame = sender.TryAcquireLatestFrame())
                {
                    if ((depthFrame != null) && (depthFrame != lastDepthFrame))
                    {
                        lastDepthFrame = depthFrame;

                        Interlocked.Increment(ref depthFrameCount);

                        // Always try to grab the depth value although, clearly, this is subject
                        // to a bunch of race conditions etc. as other thread access it.
                        centrePointDepthInMetres =
                            GetDepthValueAtCoordinate(depthFrame,
                                (int)(depthFrame.Format.VideoFormat.Width * MAGIC_DEPTH_FRAME_WIDTH_RATIO_CENTRE),
                                (int)(depthFrame.Format.VideoFormat.Height * MAGIC_DEPTH_FRAME_HEIGHT_RATIO_CENTRE)) ?? 0.0f;

                    }
                }
            };

        long rgbProcessedCount = 0;
        long facesPresentCount = 0;
        long rgbDroppedCount = 0;

        MediaFrameReference lastRgbFrame = null;

        var faceBitmapFormats = FaceTracker.GetSupportedBitmapPixelFormats().Select(
            format => format.ToString().ToLower()).ToArray();

        var faceTracker = await FaceTracker.CreateAsync();

        var rgbMediaCapture = await this.GetMediaCaptureForDescriptionAsync(
            MediaFrameSourceKind.Color, 1280, 720, 30, faceBitmapFormats);

        var rgbFrameReader = await rgbMediaCapture.Item1.CreateFrameReaderAsync(rgbMediaCapture.Item2);

        rgbFrameReader.AcquisitionMode = MediaFrameReaderAcquisitionMode.Realtime;

        int busyProcessingRgbFrame = 0;

        var unityWorldCoordinateSystem =
            Marshal.GetObjectForIUnknown(WorldManager.GetNativeISpatialCoordinateSystemPtr()) as SpatialCoordinateSystem;
        
        // Expecting this to run at 30fps.
        TypedEventHandler<MediaFrameReader, MediaFrameArrivedEventArgs> rgbFrameHandler =
           (sender, args) =>
           {
               // Only proceed if we're not already 'busy' - i.e. we'
               if (Interlocked.CompareExchange(ref busyProcessingRgbFrame, 1, 0) == 0)
               {
                   Task.Run(
                       async () =>
                       {
                           using (var rgbFrame = rgbFrameReader.TryAcquireLatestFrame())
                           {
                               if ((rgbFrame != null) && (rgbFrame != lastRgbFrame))
                               {
                                   ++rgbProcessedCount;

                                   lastRgbFrame = rgbFrame;
                                   var facePosition = uVector3.zero;

                                   using (var videoFrame = rgbFrame.VideoMediaFrame.GetVideoFrame())
                                   {
                                       var faces = await faceTracker.ProcessNextFrameAsync(videoFrame);
                                       var firstFace = faces.FirstOrDefault();

                                       if (firstFace != null)
                                       {
                                           ++facesPresentCount;

                                           // Take the first face and the centre point of that face to try
                                           // and simplify things for my limited brain.
                                           var faceCentrePointInImageCoords =
                                              new Point(
                                                  firstFace.FaceBox.X + (firstFace.FaceBox.Width / 2.0f),
                                                  firstFace.FaceBox.Y + (firstFace.FaceBox.Height / 2.0f));

                                           wMatrix4x4 projectionTransform = wMatrix4x4.Identity;
                                           wMatrix4x4 viewTransform = wMatrix4x4.Identity;
                                           wMatrix4x4 invertedViewTransform = wMatrix4x4.Identity;

                                           var rgbCoordinateSystem = GetRgbFrameProjectionAndCoordinateSystemDetails(
                                               rgbFrame, out projectionTransform, out invertedViewTransform);

                                           // Scale the RGB point (1280x720)
                                           var faceCentrePointUnitScaleRGB = ScalePointMinusOneToOne(faceCentrePointInImageCoords, rgbFrame);

                                           // Unproject the RGB point back at unit depth as per the locatable camera
                                           // document.
                                           var unprojectedFaceCentrePointRGB = UnProjectVector(
                                                  new wVector3(
                                                      (float)faceCentrePointUnitScaleRGB.X,
                                                      (float)faceCentrePointUnitScaleRGB.Y,
                                                      1.0f),
                                                  projectionTransform);

                                           // Transfrom this back by the inverted view matrix in order to put this into
                                           // the RGB camera coordinate system
                                           var faceCentrePointCameraCoordsRGB =
                                                  wVector3.Transform(unprojectedFaceCentrePointRGB, invertedViewTransform);

                                           // Get the transform from the camera coordinate system to the Unity world
                                           // coordinate system, could probably cache this?
                                           var cameraRGBToWorldTransform =
                                                  rgbCoordinateSystem.TryGetTransformTo(unityWorldCoordinateSystem);

                                           if (cameraRGBToWorldTransform.HasValue)
                                           {
                                               // Transform to world coordinates
                                               var faceCentrePointWorldCoords = wVector4.Transform(
                                                      new wVector4(
                                                          faceCentrePointCameraCoordsRGB.X,
                                                          faceCentrePointCameraCoordsRGB.Y,
                                                          faceCentrePointCameraCoordsRGB.Z, 1),
                                                      cameraRGBToWorldTransform.Value);

                                               // Where's the camera in world coordinates?
                                               var cameraOriginWorldCoords = wVector4.Transform(
                                                      new wVector4(0, 0, 0, 1),
                                                      cameraRGBToWorldTransform.Value);

                                               // Multiply Z by -1 for Unity
                                               var cameraPoint = new uVector3(
                                                    cameraOriginWorldCoords.X,
                                                    cameraOriginWorldCoords.Y,
                                                    -1.0f * cameraOriginWorldCoords.Z);

                                               // Multiply Z by -1 for Unity
                                               var facePoint = new uVector3(
                                                      faceCentrePointWorldCoords.X,
                                                      faceCentrePointWorldCoords.Y,
                                                      -1.0f * faceCentrePointWorldCoords.Z);

                                               facePosition = 
                                                   cameraPoint + 
                                                   (facePoint - cameraPoint).normalized * centrePointDepthInMetres;
                                           }
                                       }
                                   }
                                   if (facePosition != uVector3.zero)
                                   {
                                       UnityEngine.WSA.Application.InvokeOnAppThread(
                                           () =>
                                           {
                                               this.faceMarker.transform.position = facePosition;
                                           },
                                           false
                                        );
                                   }
                               }
                           }
                           Interlocked.Exchange(ref busyProcessingRgbFrame, 0);
                       }
                   );
               }
               else
               {
                   Interlocked.Increment(ref rgbDroppedCount);
               }
               // NB: this is a bit naughty as I am accessing these counters across a few threads so
               // accuracy might suffer here.
               UnityEngine.WSA.Application.InvokeOnAppThread(
                   () =>
                   {
                       this.textMesh.text =
                           $"{depthFrameCount} depth,{rgbProcessedCount} rgb done, {rgbDroppedCount} rgb drop," +
                           $"{facesPresentCount} faces, ({centrePointDepthInMetres:N2})";
                   },
                   false);
           };

        depthFrameReader.FrameArrived += depthFrameHandler;
        rgbFrameReader.FrameArrived += rgbFrameHandler;

        await depthFrameReader.StartAsync();
        await rgbFrameReader.StartAsync();

        // Wait forever then dispose...just doing this to keep track of what needs disposing.
        await Task.Delay(-1);

        depthFrameReader.FrameArrived -= depthFrameHandler;
        rgbFrameReader.FrameArrived -= rgbFrameHandler;

        rgbFrameReader.Dispose();
        depthFrameReader.Dispose();

        rgbMediaCapture.Item1.Dispose();
        depthMediaCapture.Item1.Dispose();

        Marshal.ReleaseComObject(unityWorldCoordinateSystem);
    }


    static SpatialCoordinateSystem GetRgbFrameProjectionAndCoordinateSystemDetails(
        MediaFrameReference rgbFrame,
        out wMatrix4x4 projectionTransform,
        out wMatrix4x4 invertedViewTransform)
    {
        SpatialCoordinateSystem rgbCoordinateSystem = null;
        wMatrix4x4 viewTransform = wMatrix4x4.Identity;
        projectionTransform = wMatrix4x4.Identity;
        invertedViewTransform = wMatrix4x4.Identity;

        object value;

        if (rgbFrame.Properties.TryGetValue(MFSampleExtension_Spatial_CameraCoordinateSystem, out value))
        {
            // I'm not sure that this coordinate system changes per-frame so I could maybe do this once?
            rgbCoordinateSystem = value as SpatialCoordinateSystem;
        }
        if (rgbFrame.Properties.TryGetValue(MFSampleExtension_Spatial_CameraProjectionTransform, out value))
        {
            // I don't think that this transform changes per-frame so I could maybe do this once?
            projectionTransform = ByteArrayToMatrix(value as byte[]);
        }
        if (rgbFrame.Properties.TryGetValue(MFSampleExtension_Spatial_CameraViewTransform, out value))
        {
            // I think this transform changes per frame.
            viewTransform = ByteArrayToMatrix(value as byte[]);
            wMatrix4x4.Invert(viewTransform, out invertedViewTransform);
        }
        return (rgbCoordinateSystem);
    }
    /// <summary>
    /// Not using this right now as I don't *know* how to scale an RGB point to a depth point
    /// given that the depth frame seems to have a central 'hot spot' that's circular.
    /// </summary>
    static Point ScaleRgbPointToDepthPoint(Point rgbPoint, MediaFrameReference rgbFrame,
        MediaFrameReference depthFrame)
    {
        return (new Point(
            rgbPoint.X / rgbFrame.Format.VideoFormat.Width * depthFrame.Format.VideoFormat.Width,
            rgbPoint.Y / rgbFrame.Format.VideoFormat.Height * depthFrame.Format.VideoFormat.Height));
    }
    /// <summary>
    /// This code taken fairly literally from this doc
    /// https://docs.microsoft.com/en-us/windows/mixed-reality/locatable-camera#pixel-to-application-specified-coordinate-system
    /// and hopefully without me breaking it too badly.
    /// </summary>
    static Point ScalePointMinusOneToOne(Point point, MediaFrameReference frameRef)
    {
        var scaledPoint = new Point(
            (2.0f * (float)point.X / frameRef.Format.VideoFormat.Width) - 1.0f,
            (2.0f * (1.0f - (float)point.Y / frameRef.Format.VideoFormat.Height)) - 1.0f);

        return (scaledPoint);
    }
    /// <summary>
    /// This code taken fairly literally from this doc
    /// https://docs.microsoft.com/en-us/windows/mixed-reality/locatable-camera#pixel-to-application-specified-coordinate-system
    /// but if it's got messed up in the translation then that's definitely my fault :-)
    /// </summary>
    static wVector3 UnProjectVector(wVector3 from, wMatrix4x4 cameraProjection)
    {
        var to = new wVector3(0, 0, 0);

        var axsX = new wVector3(cameraProjection.M11, cameraProjection.M12, cameraProjection.M13);

        var axsY = new wVector3(cameraProjection.M21, cameraProjection.M22, cameraProjection.M23);

        var axsZ = new wVector3(cameraProjection.M31, cameraProjection.M32, cameraProjection.M33);

        to.Z = from.Z / axsZ.Z;
        to.Y = (from.Y - (to.Z * axsY.Z)) / axsY.Y;
        to.X = (from.X - (to.Z * axsX.Z)) / axsX.X;

        return to;
    }
    unsafe static float? GetDepthValueAtCoordinate(MediaFrameReference frame, int x, int y)
    {
        float? depthValue = null;

        var bitmap = frame.VideoMediaFrame.SoftwareBitmap;

        using (var buffer = bitmap.LockBuffer(BitmapBufferAccessMode.Read))
        using (var reference = buffer.CreateReference())
        {
            var description = buffer.GetPlaneDescription(0);

            byte* pBits;
            uint size;
            var byteAccess = reference as IMemoryBufferByteAccess;

            byteAccess.GetBuffer(out pBits, out size);

            // Try the pixel value itself and see if we get anything there.
            depthValue = GetDepthValueFromBufferAtXY(
                pBits, x, y, description, (float)frame.VideoMediaFrame.DepthMediaFrame.DepthFormat.DepthScaleInMeters);

#if HUNT_DEPTH_PIXEL_GRID
            if (depthValue == null)
            {
                // If we don't have a value, look for one in the surrounding space (the sub-function copes
                // with us using bad values of x,y).
                var minDistance = double.MaxValue;

                for (int i = 0 - DEPTH_SEARCH_GRID_SIZE; i < DEPTH_SEARCH_GRID_SIZE; i++)
                {
                    for (int j = 0 - DEPTH_SEARCH_GRID_SIZE; j < DEPTH_SEARCH_GRID_SIZE; j++)
                    {
                        var newX = x + i;
                        var newY = y + j;

                        var testValue = GetDepthValueFromBufferAtXY(
                            pBits,
                            newX,
                            newY,
                            description,
                            (float)frame.VideoMediaFrame.DepthMediaFrame.DepthFormat.DepthScaleInMeters);

                        if (testValue != null)
                        {
                            var distance =
                                Math.Sqrt(Math.Pow(newX - x, 2.0) + Math.Pow(newY - y, 2.0));

                            if (distance < minDistance)
                            {
                                depthValue = testValue;
                                minDistance = distance;
                            }
                        }
                    }
                }
            }
#endif // HUNT_DEPTH_PIXEL_GRID
        }
        return (depthValue);
    }
    unsafe static float? GetDepthValueFromBufferAtXY(byte* pBits, int x, int y, BitmapPlaneDescription desc,
        float scaleInMeters)
    {
        float? depthValue = null;

        var bytesPerPixel = desc.Stride / desc.Width;
        Debug.Assert(bytesPerPixel == Marshal.SizeOf<UInt16>());

        int offset = (desc.StartIndex + desc.Stride * y) + (x * bytesPerPixel);

        if ((offset > 0) && (offset < ((long)pBits + (desc.Height * desc.Stride))))
        {
            depthValue = *(UInt16*)(pBits + offset) * scaleInMeters;

            if (!IsValidDepthDistance((float)depthValue))
            {
                depthValue = null;
            }
        }
        return (depthValue);
    }
    static bool IsValidDepthDistance(float depthDistance)
    {
        // If that depth value is > 4.0m then we discard it because it seems like 
        // 4.**m (4.09?) comes back from the sensor when it hasn't really got a value
        return ((depthDistance > 0.5f) && (depthDistance <= 4.0f));
    }
    // Used an explicit tuple here as I'm in C# 6.0
    async Task<Tuple<MediaCapture, MediaFrameSource>> GetMediaCaptureForDescriptionAsync(
        MediaFrameSourceKind sourceKind,
        int width,
        int height,
        int frameRate,
        string[] bitmapFormats = null)
    {
        MediaCapture mediaCapture = null;
        MediaFrameSource frameSource = null;

        var allSources = await MediaFrameSourceGroup.FindAllAsync();

        // Ignore frame rate here on the description as both depth streams seem to tell me they are
        // 30fps whereas I don't think they are (from the docs) so I leave that to query later on.
        // NB: LastOrDefault here is a NASTY, NASTY hack - just my way of getting hold of the 
        // *LAST* depth stream rather than the *FIRST* because I'm assuming that the *LAST*
        // one is the longer distance stream rather than the short distance stream.
        // I should fix this and find a better way of choosing the right depth stream rather
        // than relying on some ordering that's not likely to always work!
        var sourceInfo =
            allSources.SelectMany(group => group.SourceInfos)
            .LastOrDefault(
                si =>
                    (si.MediaStreamType == MediaStreamType.VideoRecord) &&
                    (si.SourceKind == sourceKind) &&
                    (si.VideoProfileMediaDescription.Any(
                        desc =>
                            desc.Width == width &&
                            desc.Height == height &&
                            desc.FrameRate == frameRate)));

        if (sourceInfo != null)
        {
            var sourceGroup = sourceInfo.SourceGroup;

            mediaCapture = new MediaCapture();

            await mediaCapture.InitializeAsync(
               new MediaCaptureInitializationSettings()
               {
               // I want software bitmaps
               MemoryPreference = MediaCaptureMemoryPreference.Cpu,
                   SourceGroup = sourceGroup,
                   StreamingCaptureMode = StreamingCaptureMode.Video
               }
            );
            frameSource = mediaCapture.FrameSources[sourceInfo.Id];

            var selectedFormat = frameSource.SupportedFormats.First(
                format =>
                    format.VideoFormat.Width == width && format.VideoFormat.Height == height &&
                    format.FrameRate.Numerator / format.FrameRate.Denominator == frameRate &&
                    ((bitmapFormats == null) || (bitmapFormats.Contains(format.Subtype.ToLower()))));

            await frameSource.SetFormatAsync(selectedFormat);
        }
        return (Tuple.Create(mediaCapture, frameSource));
    }
    static wMatrix4x4 ByteArrayToMatrix(byte[] bits)
    {
        var matrix = wMatrix4x4.Identity;

        var handle = GCHandle.Alloc(bits, GCHandleType.Pinned);
        matrix = Marshal.PtrToStructure<wMatrix4x4>(handle.AddrOfPinnedObject());
        handle.Free();

        return (matrix);
    }
#if HUNT_DEPTH_PIXEL_GRID

    static readonly int DEPTH_SEARCH_GRID_SIZE = 32;

#endif // HUNT_DEPTH_PIXEL_GRID

    static readonly float MAGIC_DEPTH_FRAME_HEIGHT_RATIO_CENTRE = 0.25f;
    static readonly float MAGIC_DEPTH_FRAME_WIDTH_RATIO_CENTRE = 0.5f;
    static readonly Guid MFSampleExtension_Spatial_CameraCoordinateSystem = new Guid("9D13C82F-2199-4E67-91CD-D1A4181F2534");
    static readonly Guid MFSampleExtension_Spatial_CameraProjectionTransform = new Guid("47F9FCB5-2A02-4F26-A477-792FDF95886A");
    static readonly Guid MFSampleExtension_Spatial_CameraViewTransform = new Guid("4E251FA4-830F-4770-859A-4B8D99AA809B");

#endif // ENABLE_WINMD_SUPPORT
}