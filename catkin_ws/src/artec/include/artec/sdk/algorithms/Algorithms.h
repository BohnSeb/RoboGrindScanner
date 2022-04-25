/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Algorithms creator declarations
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ALGORITHMS_H_
#define _ALGORITHMS_H_

#include <artec/sdk/algorithms/AlgorithmSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IProgressInfo.h>
#include <artec/sdk/base/Point.h>
#include <artec/sdk/base/ScannerType.h>

namespace artec { namespace sdk { namespace base
{
	class  ICompositeMesh;
	class  IScan;
	class  ICompositeContainer;
	class  IModel;
	class  ICancellationToken;
	class  IProgressInfo;
	struct AlgorithmWorkset;
} } }

namespace artec { namespace sdk { namespace algorithms
{
using namespace artec::sdk::base::errors;

class IAlgorithm;

/// Enum for Poisson Fusion and Texturization algorithms.
enum InputFilter
{
    /// Algorithm will use only frames with set texture key-frame attribute.
    /// This type is used by default.
	InputFilter_UseTextureKeyFrames = 0,

    /// Algorithm considers all frames that do not have empty textures.
	InputFilter_UseAllTextures,
};

//////////////////////////////////////////////////////////////////////////
/// Parameters for Fast fusion algorithm (algorithm quickly creates a simple model)
struct FastFusionSettings
{
    /// Scanner type
    base::ScannerType	scannerType;

    /// Mesh resolution (in millimeters) is a step of grid which is used to reconstruct a polygonal model.
    /// Parameter defines the minimal space unit where the polygon of the model will be built.
    /// Resolution for data from EVA should be no less than 0.5
    /// Resolution for data from Spider should be no less than 0.1
    /// Resolution for data from L2 should be no less than 0.5
    float				resolution;
	
    /// Radius(for Fast Fusion only) is a factor to define vicinity
    /// that algorithm takes into consideration when adding each new minimal space unit.
    int					radius;

    /// If true, then calculate normals for created mesh
    bool				generateNormals;
};

///Fusion type 
enum PoissonFusionType
{
	PoissonFusionType_Sharp, ///< Sharp fusion
	PoissonFusionType_Smooth, ///< Smooth fusion

	PoissonFusionType_ForceDword = 0x7fffffff			///< Force 32-bit size enum 
};

/// What kind of holes to fill
enum FillHolesType
{
    /// Automatically fills in all holes in the mesh
    FillHolesType_All,

    /// All holes having radius no greater than the specified in "maxHoleRadius" value will be filled in;
    FillHolesType_ByRadius,

    FillHolesType_ForceDword = 0x7fffffff				///< Force 32-bit size enum 
};
/// Parameters for Sharp/Smooth fusion algorithms
struct PoissonFusionSettings
{
    /// Scanner type
    base::ScannerType		scannerType;

    /// Fusion type
    PoissonFusionType		fusionType;

    /// Settle the algorithm to fill holes in the mesh being reconstructed (for PoissonFusion).
    FillHolesType			fillType;

    /// Mesh resolution (in millimeters) is a step of grid which is used to reconstruct a polygonal model.
    /// Parameter defines the minimal space unit where the polygon of the model will be built.
    float					resolution;

    /// Maximum hole radius in millimeters
    float					maxHoleRadius;

    /// If it is true, then erase small embossments from surfaces on which targets are placed.
    bool					removeTargets;

    /// Round type target - black circle diameter (in millimeters)
	float					targetInnerSize;

    /// Round type target - white circle diameter (in millimeters)
    float					targetOuterSize;

    /// If true, then calculate normals for created mesh
    bool					generateNormals;

    /// Type of frames for algorithm to use.
    InputFilter				inputFilterType;
};

///Serial registration type
enum SerialRegistrationType
{
	SerialRegistrationType_Rough,						///< Register surfaces by geometry.
                                                        ///< Use this registration type only if no frames were registered during scanning.
                                                        ///< It is suited for any data types. It is fast, but less accurate.
	SerialRegistrationType_RoughTextured,				///< Register surfaces by both texture and geometry (hybrid registration).
                                                        ///< Use this registration type only if no frames were registered during scanning.
                                                        ///< It is suited for all frames that have texture. It is fast, but less accurate.
	SerialRegistrationType_Fine,						///< Register surfaces by geometry.
                                                        ///< Use this registration type for any data types. 
	SerialRegistrationType_FineTextured,				///< Register surfaces by both texture and geometry (hybrid registration).
                                                        ///< Use this registration type only if all frames have texture.

	SerialRegistrationType_ForceDword = 0x7fffffff		///< Force 32-bit size enum 
};

/// Parameters for Fine registration algorithm
struct SerialRegistrationSettings
{
    /// Scanner type
    base::ScannerType		scannerType;

    /// Serial registration type
    SerialRegistrationType	registrationType;
};

/// Parameters for Auto-alignment algorithm (Algorithm that assembles scans)
struct AutoAlignSettings
{
    /// Scanner type
    base::ScannerType		scannerType;
};


/// Global registration type
enum GlobalRegistrationType
{
	GlobalRegistrationType_Geometry,					///< Use this registration type for any data types 
	GlobalRegistrationType_GeometryAndTexture,			///< Use this registration type only if all frames have texture

	GlobalRegistrationType_ForceDword = 0x7fffffff		///< Force 32-bit size enum 
};
/// Parameters for Global registration algorithm (algorithm optimizes frames within scans, launch it for a pre-aligned batch of scans or for a single scan)
struct GlobalRegistrationSettings
{
    /// Scanner type
    base::ScannerType		scannerType;

    /// Global registration type
    GlobalRegistrationType	registrationType;
};

/// Texturing algorithm type
enum TexturizeType
{   
    TexturizeType_Advanced = 0,             ///< Each vertex gets multiple coordinates for each of the adjacent triangles,
                                            ///< final texture coordinates are continuous only for a given triangle area.

    TexturizeType_Atlas,                    ///< Unfold mesh to UV plane, then generate the texture image

    TexturizeType_KeepAtlas,                ///< Keep UV coordinates and regenerate the texture image

    TexturizeType_VertexColorToAtlas,       ///< Not implemented yet. Intended to texturize based on the vertex color data.

	TexturizeType_ForceDword = 0x7fffffff	///< Force 32-bit size enum 
};

/// Resolution of texture output file (in pixels)
enum TexturizeResolution
{
	TexturizeResolution_512x512 = 0,   ///< 0.5K texture file resolution 
	TexturizeResolution_1024x1024,     ///< 1K texture file resolution
	TexturizeResolution_2048x2048,     ///< 2K texture file resolution
	TexturizeResolution_4096x4096,     ///< 4K texture file resolution
	TexturizeResolution_8192x8192,     ///< 8K texture file resolution
	TexturizeResolution_16384x16384,   ///< 16K texture file resolution

	TexturizeResolution_ForceDword = 0x7fffffff			///< Force 32-bit size enum 
};

/// Parameters for Texturing algorithm
struct TexturizationSettings
{
    /// Scanner type
    base::ScannerType	scannerType;

    /// Texturing type to use
    TexturizeType		texturizeType;

    /// Resolution of texturing output (in pixels)
    TexturizeResolution	texturizeResolution;

    /// Enable background segmentation for input texture frames
    bool				enableBackgroundSegmentation;

    /// Enable compensation of ambient light
    bool				enableAmbientLightingCompensation;
    
    /// Set the limit for number of polygons for mesh unfolding:
    /// 0 - disable simplification (unfold original mesh)
    /// N - reduce number of polygons to N, then make unfolding and reproject atlas to the original mesh
    int					atlasUnfoldingPolygonLimit;

    /// Interpolate colors for empty texture areas
    bool				enableTextureInpainting;

    /// Correct texture brightness for Artec scanners
    bool				useTextureNormalization;
    
    /// Type of frames for algorithm to use
    InputFilter			inputFilterType;
};

/// What size of object to erase?
enum SmallObjectsFilterType
{
	SmallObjectsFilterType_LeaveBiggestObject = 0,		///< Erase all objects except the one with the most polygons
	SmallObjectsFilterType_FilterByThreshold  = 1,		///< Erases all objects whose number of polygons is less than 
														///< the amount specified in filterThreshold.
													
	SmallObjectsFilterType_ForceDword = 0x7fffffff		///< Force 32-bit size enum 
};

/**
* Parameters for Small-object filter.
* Algorithm erases small extraneous surfaces. Launch if after Fusion.
* 
*/
struct SmallObjectsFilterSettings
{
    /// Scanner type
    base::ScannerType		scannerType;

    /// Small-object filter type
    SmallObjectsFilterType	filterType;

    /// Threshold is the maximum number of triangles for FilterByThreshold. 
    int						filterThreshold;
};

/// Simplification-algorithm target
enum SimplifyType
{
	SimplifyType_TriangleQuantity,						///< Reduce triangle count to the given number
	SimplifyType_Accuracy,								///< Keep accuracy of the mesh being simplified
	SimplifyType_Remesh,								///< Remove triangles whose edge lengths are less than the remeshEdgeThreshold value
	SimplifyType_TriangleQuantityFast,					///< Quickly reduce triangle count to the given number (Fast mesh simplification)

	SimplifyType_ForceDword = 0x7fffffff				///< Force 32-bit size enum 
};

/// Simplification method. What exactly algorithm does to meet the target.

enum SimplifyMetric
{
	SimplifyMetric_EdgeLength,							///< Ensure that the new edges are less than the specified length (error)
	SimplifyMetric_EdgeLengthAndAngle,					///< Ensure that the new edges are less than the specified length (error)
                                                        ///< and angles between normals to the new neighboring faces are less than the specified value (angleThreshold)
	SimplifyMetric_DistanceToSurface,					///< Ensure that the distance to the new faces are less than the specified value (error)
	SimplifyMetric_DistanceToSurfaceIterative,			///< Ensure that the distance to the new faces are less than the specified value (error)

	SimplifyMetric_ForceDword = 0x7fffffff				///< Force 32-bit size enum 
};
/// Parameters for Mesh simplification algorithm
struct MeshSimplificationSettings
{
    /// Scanner type
    base::ScannerType	scannerType;

    /// Simplification target: TriangleQuantity, Accuracy, Remesh, TriangleQuantityFast
    SimplifyType		simplifyType;

    /// Simplification method: EdgeLength, DistanceToSurface etc. Not supported by Fast simplification.
    SimplifyMetric		simplifyMetrics;

    /// Triangle count after simplification
    int					triangleNumber;

    /// Maintains boundary of the surface
    bool				keepBoundary;

    /// Maximum angle between neighboring normals (in degrees). Not supported by Fast simplification.
    float				angleThreshold;

    /// Maximum edge length (in millimeters) after the Remesh operation. Not supported by Fast simplification.
    float				remeshEdgeThreshold;

    /// Maximum error or length (in millimeters). Not supported by Fast simplification.
    float				error;
};
/// Parameters for Fast mesh simplification algorithm
struct FastMeshSimplificationSettings
{
    /// Scanner type
    base::ScannerType	scannerType;

    /// Triangle number to target
    int					triangleNumber;

    /// Maintains boundary of the surface.
    bool				keepBoundary;

    /// Simplification stops if either of the thresholding criteria below isn't satisfied.
    bool				enableAdditionalCriteria;

    /// Limits maximum distance between the original and simplified meshes.
    bool				enableDistanceThreshold;

    /// Max error (in units^2).
    float				distanceThreshold;

    /// Limit the maximum normals deviation.
	bool				enableAngleThreshold;

    /// Maximum angle between normals to the neighboring faces (in degrees).
    float				angleThreshold;

    /// Limits edges length ratio.
    bool				enableAspectRatioThreshold;

    /// Maximum available ratio of the edges in triangle.
	float				aspectRatioThreshold;
};

/**
* Parameters for Loop closure algorithm designed
* to compensate for accumulated error during scanning.
* No need to utilize this algorithm. Starting from SDK ver. 2.0, you can skip it and run Global registration.
*/

struct LoopClosureSettings
{
    /// Scanner type
    base::ScannerType	scannerType;
};

/**
* Parameters for Outlier removal algorithm designed
* to erases small extraneous surfaces.
* Use either this algorithm or the Small-object filter. Also watch the order.
*/
struct OutliersRemovalSettings
{
    /// Scanner type
    base::ScannerType	scannerType;

    /// Standard deviation multiplier.
    float				standardDeviationMultiplier;

    /// Marching cube resolution (in millimeters).
    float				resolution;
};

//////////////////////////////////////////////////////////////////////////

extern "C"
{
	/// Check whether the algorithms are available on this machine
	///  
	/// @retval true if algorithm creation is permitted
	/// @retval false if algorithm creation is forbidden
	bool AALGORITHMSDK_LINK_SPEC
		checkAlgorithmsPermission();

	/// Initializes Fast Fusion algorithm descriptor with scanner related defaults
	ErrorCode AALGORITHMSDK_LINK_SPEC
		initializeFastFusionSettings( FastFusionSettings* desc, base::ScannerType scannerType );

	/// Initializes Sharp/Smooth fusion algorithm descriptor with scanner related defaults
	ErrorCode AALGORITHMSDK_LINK_SPEC
		initializePoissonFusionSettings( PoissonFusionSettings* desc, base::ScannerType scannerType );

	/// Initialize Texturing algorithm descriptor with scanner related defaults
	ErrorCode AALGORITHMSDK_LINK_SPEC
		initializeTexturizationSettings( TexturizationSettings* desc, base::ScannerType scannerType );

	/// Initializes Small-object filter descriptor with scanner related defaults
	ErrorCode AALGORITHMSDK_LINK_SPEC
		initializeSmallObjectsFilterSettings( SmallObjectsFilterSettings* desc, base::ScannerType scannerType );

	/// Initializes Mesh simplification descriptor with scanner related defaults
	ErrorCode AALGORITHMSDK_LINK_SPEC
		initializeMeshSimplificationSettings( MeshSimplificationSettings* desc, base::ScannerType scannerType, SimplifyType simplifyType );

	/// Initializes Fast mesh simplification descriptor with scanner related defaults
	ErrorCode AALGORITHMSDK_LINK_SPEC
		initializeFastMeshSimplificationSettings( FastMeshSimplificationSettings* desc, base::ScannerType scannerType);

	/// Initializes Outliers removal descriptor with scanner related defaults
	ErrorCode AALGORITHMSDK_LINK_SPEC
		initializeOutliersRemovalSettings( OutliersRemovalSettings* desc, base::ScannerType scannerType );
	

	/// Creates Fast fusion algorithm
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createFastFusionAlgorithm( IAlgorithm** fusion, const FastFusionSettings* desc );

	/// Creates Sharp/Smooth fusion algorithm
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createPoissonFusionAlgorithm( IAlgorithm** fusion, const PoissonFusionSettings* desc );

	/// Creates Fine/Rough serial registration algorithm
	ErrorCode AALGORITHMSDK_LINK_SPEC
        createSerialRegistrationAlgorithm( IAlgorithm** serialRegistration, const SerialRegistrationSettings* desc );

    /// Creates Auto-align algorithm
    ErrorCode AALGORITHMSDK_LINK_SPEC
         createAutoalignAlgorithm( IAlgorithm** autoalign, const AutoAlignSettings* desc);

	/// Creates Global registration algorithm
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createGlobalRegistrationAlgorithm( IAlgorithm** globalRegistration, const GlobalRegistrationSettings* desc );

	/// Creates texturing algorithm
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createTexturizationAlgorithm( IAlgorithm** texturizer, const TexturizationSettings* desc);

	/// Creates Small-object filter
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createSmallObjectsFilterAlgorithm( IAlgorithm** smallObjectsFilter, const SmallObjectsFilterSettings* desc );

	/// Creates Mesh simplification algorithm
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createMeshSimplificationAlgorithm( IAlgorithm** meshSimplification, const MeshSimplificationSettings* desc );

	/// Creates Fast mesh simplification algorithm
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createFastMeshSimplificationAlgorithm( IAlgorithm** meshSimplification, const FastMeshSimplificationSettings* desc );

	/// Creates Loop closure algorithm.
    /// No need to utilize this algorithm. Starting from SDK ver. 2.0, you can skip it and run Global registration instead.
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createLoopClosureAlgorithm( IAlgorithm** loopClosure, const LoopClosureSettings* desc );

	/// Creates Outlier removal algorithm
	ErrorCode AALGORITHMSDK_LINK_SPEC
		createOutliersRemovalAlgorithm( IAlgorithm** outliesRemoval, const OutliersRemovalSettings* desc );
}

} } } // namespace artec::sdk::algorithms

#endif // _ALGORITHMS_H_
