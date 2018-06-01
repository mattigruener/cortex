//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2018, Image Engine Design Inc. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are
//  met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of Image Engine Design nor the names of any
//       other contributors to this software may be used to endorse or
//       promote products derived from this software without specific prior
//       written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
//  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////
#include "maya/MFnDependencyNode.h"
#include "maya/MHWGeometry.h"
#include "maya/MFnDagNode.h"
#include "maya/MShaderManager.h"
#include <maya/MSelectionList.h>
#include <maya/MItSelectionList.h>
#include <maya/MGlobal.h>
#include <maya/MFnTransform.h>
#include <maya/MFnMatrixData.h>
#include <maya/MObjectArray.h>
#include <maya/MPlugArray.h>
#include <maya/MUserData.h>
#include <maya/MFnInstancer.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MPxComponentConverter.h>
#include <maya/MDrawRegistry.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MHWGeometryUtilities.h>

#include "OpenEXR/ImathVec.h"

#include "IECore/Timer.h"
#include "IECore/Exception.h"
#include "IECore/Object.h"
#include "IECore/MessageHandler.h"

#include "IECore/LRUCache.h"
#include "IECoreMaya/SceneShapeSubSceneOverride.h"
#include "IECoreMaya/Convert.h"
#include "IECoreMaya/MayaTypeIds.h"
#include "IECoreScene/TypeIds.h"
#include "IECoreScene/TriangulateOp.h"
#include "IECoreScene/MeshNormalsOp.h"
#include "IECoreScene/SampledSceneInterface.h"
#include "IECoreScene/PointsPrimitive.h"
#include "IECore/NullObject.h"
#include "IECore/Object.h"

#include <algorithm>
#include <math.h>

using namespace IECore;
using namespace IECoreScene;
using namespace IECoreMaya;
using namespace MHWRender;

namespace
{

struct GeometryData
{
	public :
		GeometryData()
			: vertexBufferArray( new MHWRender::MVertexBufferArray )
		{
		}

		~GeometryData()
		{
			for( unsigned int i = 0; i < vertexBufferArray->count(); ++i )
			{
				delete vertexBufferArray->getBuffer( i );
			}
			vertexBufferArray->clear();
		}

		// Indices can be shared between GeometryData objects. While not being
		// implemented currently, we should share topology buffers between
		// GeometryData instances that represent an animated/deformed mesh.
		using IndexBufferPtr = std::shared_ptr<MHWRender::MIndexBuffer> ;

		// While we could share actual buffers (if we add Pref at some point, for
		// example), the array itself that we pass to Maya is unique.
		using VertexBufferArrayPtr = std::unique_ptr<MHWRender::MVertexBufferArray>;

		VertexBufferArrayPtr vertexBufferArray;
		IndexBufferPtr indexBuffer;
		IndexBufferPtr wireframeIndexBuffer;
};

using GeometryDataPtr = std::shared_ptr<GeometryData>;

// Wrap Maya's MRenderItem so that we get a shareable pointer that we can return
// from a cache. This is intended to only ever hold MRenderItem's that Maya are
// not going to see as those need separate cleaning up.
// \todo is this specifier needed?
// class IECOREMAYA_API RenderItemWrapper : public IECore::RefCounted
// {
// 	public :

// 		RenderItemWrapper( MHWRender::MRenderItem *renderItem, GeometryDataPtr geometryData )
// 			: m_renderItem( renderItem ), m_geometryData( geometryData )
// 		{
// 		}

// 		~RenderItemWrapper()
// 		{
// 			MRenderItem::Destroy( m_renderItem );
// 		}

// 		MHWRender::MRenderItem* get() { return m_renderItem; }

// 	private :

// 		MHWRender::MRenderItem *m_renderItem;

// 		// Holding on to this in order to ensure the geometry data we've passed
// 		// to the render item is kept alive for as long as someone might take
// 		// copies of this render item.
// 		GeometryDataPtr m_geometryData;
// };

// IE_CORE_DECLAREPTR( RenderItemWrapper );

class RenderItemUserData : public MUserData
{
	public :

		RenderItemUserData( int componentIndex )
			: MUserData( true ),  // delete when render item is deleted
			  componentIndex( componentIndex )
		{
		}

		virtual ~RenderItemUserData()
		{
		}

		int componentIndex;
};

// As we render a render item per component, all we need to know here is the
// component index per render item.
class ComponentConverter : public MPxComponentConverter
{
	public :

		static MPxComponentConverter *creator()
		{
			return new ComponentConverter();
		}

		ComponentConverter()
			: MHWRender::MPxComponentConverter()
		{
		}

		~ComponentConverter() override
		{
		}

		void initialize( const MRenderItem &renderItem ) override
		{
			RenderItemUserData* userData = dynamic_cast<RenderItemUserData*>(renderItem.customData());
			m_idx = userData->componentIndex;

			m_object = m_component.create( MFn::kMeshPolygonComponent );
		}

		void addIntersection( MIntersection &intersection ) override
		{
			m_component.addElement( m_idx );
		}

		MObject component()
		{
			return m_object;
		}

		MSelectionMask 	selectionMask () const override
		{
			return MSelectionMask::kSelectMeshFaces;
		}

	private :

		MFnSingleIndexedComponent m_component;
		MObject m_object;
		int m_idx;
};

bool objectCanBeRendered( IECore::ConstObjectPtr object )
{
	switch( static_cast<IECoreScene::TypeId>( object->typeId() ) )
	{
		case IECoreScene::TypeId::MeshPrimitiveTypeId :
		case IECoreScene::TypeId::PointsPrimitiveTypeId :
		case IECoreScene::TypeId::CurvesPrimitiveTypeId :
		{
			return true;
		}
		default :
			return false;
	}
}

RenderStyle& operator++( RenderStyle& style )
{
	switch( style )
	{

	case RenderStyle::BoundingBox:
		return style = RenderStyle::Wireframe;

	case RenderStyle::Wireframe:
		return style = RenderStyle::Solid;

	case RenderStyle::Solid:
		return style = RenderStyle::Textured;

	case RenderStyle::Textured:
		return style = RenderStyle::Last;

	case RenderStyle::Last:
		return style = RenderStyle::BoundingBox;

	default:
		return style;

	}
}

struct BoundDataCacheGetterKey
{
	BoundDataCacheGetterKey( const Imath::Box3f b )
		:	m_bounds( b )
	{
		m_hash.append( m_bounds );
	}

	operator const IECore::MurmurHash & () const
	{
		return m_hash;
	}

	const Imath::Box3f m_bounds;
	IECore::MurmurHash m_hash;
};

struct GeometryDataCacheGetterKey
{
	GeometryDataCacheGetterKey( const IECoreScene::ConstPrimitivePtr primitive, const MVertexBufferDescriptorList &descriptorList )
		:	m_primitive( primitive ), m_descriptorList( descriptorList )
	{
		m_primitive->hash( m_hash );
	}

	operator const IECore::MurmurHash & () const
	{
		return m_hash;
	}

	const IECoreScene::ConstPrimitivePtr m_primitive;
	const MVertexBufferDescriptorList &m_descriptorList;
	IECore::MurmurHash m_hash;
};

// struct RenderItemWrapperCacheGetterKey
// {
// 	RenderItemWrapperCacheGetterKey( SceneShapeSubSceneOverride *sceneShape,
// 									 const std::string &name,
// 									 ConstObjectPtr object,
// 									 const RenderStyle style,
// 									 MSubSceneContainer &container,
// 									 MShaderInstance *shader,
// 									 const Imath::Box3f boundingBox,
// 									 bool hashName = true)
// 		:	m_object( object ), m_name( name ), m_style( style ), m_sceneShape( sceneShape ), m_container( container ), m_shader( shader ), m_boundingBox( boundingBox )
// 	{
// 		m_object->hash( m_hash );
// 		m_hash.append( (int)m_style );
// 		m_hash.append( boundingBox );

// 		if( hashName )
// 		{
// 			m_hash.append( name );
// 		}
// 	}

// 	operator const IECore::MurmurHash & () const
// 	{
// 		return m_hash;
// 	}

// 	// most of these members aren't needed to compute the hash, but to then
// 	// construct an MRenderItem
// 	ConstObjectPtr m_object;
// 	const std::string m_name;
// 	const RenderStyle m_style;
// 	SceneShapeSubSceneOverride *m_sceneShape;
// 	MSubSceneContainer &m_container;
// 	MShaderInstance *m_shader;
// 	const Imath::Box3f m_boundingBox;

// 	IECore::MurmurHash m_hash;
// };

void ensureFaceVaryingData( IECore::ConstIntVectorDataPtr &i, IECore::ConstV3fVectorDataPtr &p, IECore::ConstV3fVectorDataPtr &n, std::vector<int> &additionalIndices, bool interpolationIsLinear )
{
	const std::vector<int> &indicesReadable = i->readable();
	const std::vector<Imath::V3f> &positionsReadable = p->readable();
	const std::vector<Imath::V3f> &normalsReadable = n->readable();

	IECore::IntVectorDataPtr newIndices = new IECore::IntVectorData();
	IECore::V3fVectorDataPtr newPositions = new IECore::V3fVectorData();
	IECore::V3fVectorDataPtr newNormals = new IECore::V3fVectorData();

	std::vector<int> &newIndicesWritable = newIndices->writable();
	std::vector<Imath::V3f> &newPositionsWritable = newPositions->writable();
	std::vector<Imath::V3f> &newNormalsWritable = newNormals->writable();

	int numFaceVertices = indicesReadable.size();
	newIndicesWritable.reserve( numFaceVertices );
	newPositionsWritable.reserve( numFaceVertices );
	newNormalsWritable.reserve( numFaceVertices );

	std::vector<int> indexMapping;
	indexMapping.resize( *std::max_element( indicesReadable.begin(), indicesReadable.end() ) + 1 );
	for( int i = 0; i < numFaceVertices; ++i )
	{
		int oldIndex = indicesReadable[i];

		int newIndex;
		newPositionsWritable.push_back( positionsReadable[oldIndex] );
		if( interpolationIsLinear )
		{
			// we know that we operate on triangulated meshes.
			newNormalsWritable.push_back( normalsReadable[i/3] );
		}
		else
		{
			newNormalsWritable.push_back( normalsReadable[oldIndex] );
		}

		newIndex = newPositionsWritable.size() - 1;
		newIndicesWritable.push_back( newIndex );
		indexMapping[oldIndex] = newIndex;
	}

	// still need to remap additional indices that the above will have messed up
	for( int &index : additionalIndices )
	{
		index = indexMapping[index];
	}

	i = newIndices;
	p = newPositions;
	n = newNormals;
}

void fillBoundData( const Imath::Box3f &bounds, GeometryDataPtr boundingBoxData )
{
	MVertexBufferDescriptor positionDescriptor( "", MGeometry::kPosition, MGeometry::kFloat, 3 );
	MVertexBuffer *positionBuffer = new MVertexBuffer( positionDescriptor );
	void* positionData = positionBuffer->acquire( 8, true );
	if( positionData )
	{
		float* tmp = (float*) positionData;

		tmp[0*3 + 0] = bounds.min.x; // Indices:
		tmp[0*3 + 1] = bounds.min.y; //								  7------6
		tmp[0*3 + 2] = bounds.min.z; //								 /|     /|
		tmp[1*3 + 0] = bounds.max.x; //								3------2 |
		tmp[1*3 + 1] = bounds.min.y; //								| 4----|-5
		tmp[1*3 + 2] = bounds.min.z; //								|/     |/
		tmp[2*3 + 0] = bounds.max.x; //								0------1
		tmp[2*3 + 1] = bounds.max.y;
		tmp[2*3 + 2] = bounds.min.z;
		tmp[3*3 + 0] = bounds.min.x;
		tmp[3*3 + 1] = bounds.max.y;
		tmp[3*3 + 2] = bounds.min.z;
		tmp[4*3 + 0] = bounds.min.x;
		tmp[4*3 + 1] = bounds.min.y;
		tmp[4*3 + 2] = bounds.max.z;
		tmp[5*3 + 0] = bounds.max.x;
		tmp[5*3 + 1] = bounds.min.y;
		tmp[5*3 + 2] = bounds.max.z;
		tmp[6*3 + 0] = bounds.max.x;
		tmp[6*3 + 1] = bounds.max.y;
		tmp[6*3 + 2] = bounds.max.z;
		tmp[7*3 + 0] = bounds.min.x;
		tmp[7*3 + 1] = bounds.max.y;
		tmp[7*3 + 2] = bounds.max.z;

		positionBuffer->commit( positionData );
		boundingBoxData->vertexBufferArray->addBuffer( "positions", positionBuffer );
	}

	// \todo: For completeness we should probably create an index buffer for
	// triangles, as well - in case we ever want to render a shaded bounding box.

	// Create index buffer for lines that create a wireframe box.
	boundingBoxData->wireframeIndexBuffer.reset( new MIndexBuffer( MGeometry::kUnsignedInt32 ) );
	void* indexData = boundingBoxData->wireframeIndexBuffer->acquire( 24, true );

	if( indexData )
	{
		unsigned int* tmp = (unsigned int*) indexData;

		tmp[0] = 0; tmp[1] = 1;
		tmp[2] = 1; tmp[3] = 2;
		tmp[4] = 2; tmp[5] = 3;
		tmp[6] = 3; tmp[7] = 0;

		tmp[8]  = 4; tmp[9]  = 5;
		tmp[10] = 5; tmp[11] = 6;
		tmp[12] = 6; tmp[13] = 7;
		tmp[14] = 7; tmp[15] = 4;

		tmp[16] = 7; tmp[17] = 3;
		tmp[18] = 6; tmp[19] = 2;
		tmp[20] = 4; tmp[21] = 0;
		tmp[22] = 5; tmp[23] = 1;

		boundingBoxData->wireframeIndexBuffer->commit( indexData );
	}
}

void fillPointsData( ConstPrimitivePtr primitive, GeometryDataPtr geometryData )
{
	IECoreScene::ConstPointsPrimitivePtr pointsPrimitive = IECore::runTimeCast<const IECoreScene::PointsPrimitive>( primitive );

	IECore::ConstV3fVectorDataPtr p = pointsPrimitive->variableData<IECore::V3fVectorData>( "P" );
	const std::vector<Imath::V3f> &positionsReadable = p->readable();

	if( p )
	{
		MVertexBufferDescriptor positionDescriptor( "", MGeometry::kPosition, MGeometry::kFloat, 3 );
		MVertexBuffer *positionBuffer = new MVertexBuffer( positionDescriptor );
		void *positionData = positionBuffer->acquire( positionsReadable.size(), true );

		if( positionData && positionBuffer )
		{
			memcpy( positionData, positionsReadable.data(), sizeof( float ) * 3 * positionsReadable.size() );
			positionBuffer->commit( positionData );
			geometryData->vertexBufferArray->addBuffer( "positions", positionBuffer );
		}
	}

	// these can be empty for simple points data, but they have to exist.
	geometryData->indexBuffer.reset( new MIndexBuffer(MGeometry::kUnsignedInt32) );
	geometryData->wireframeIndexBuffer.reset( new MIndexBuffer(MGeometry::kUnsignedInt32) );
}

void fillCurvesData( ConstPrimitivePtr primitive, GeometryDataPtr geometryData )
{
	// \todo: currently a curve is built from linear segments. Needs proper interpolation at some point.

	IECoreScene::ConstCurvesPrimitivePtr curvesPrimitive = IECore::runTimeCast<const IECoreScene::CurvesPrimitive>( primitive );

	// provide positions
	IECore::ConstV3fVectorDataPtr p = curvesPrimitive->variableData<IECore::V3fVectorData>( "P" );
	const std::vector<Imath::V3f> &positionsReadable = p->readable();

	MVertexBufferDescriptor positionDescriptor( "", MGeometry::kPosition, MGeometry::kFloat, 3 );
	MVertexBuffer *positionBuffer = new MVertexBuffer( positionDescriptor );
	void *positionData = positionBuffer->acquire( positionsReadable.size(), true );

	if( positionData && positionBuffer )
	{
		memcpy( positionData, positionsReadable.data(), sizeof( float ) * 3 * positionsReadable.size() );
		positionBuffer->commit( positionData );
		geometryData->vertexBufferArray->addBuffer( "positions", positionBuffer );
	}

	// provide indices
	const IntVectorData *verticesPerCurve = curvesPrimitive->verticesPerCurve();
	const std::vector<int> &verticesPerCurveReadable = verticesPerCurve->readable();

	geometryData->indexBuffer.reset( new MIndexBuffer(MGeometry::kUnsignedInt32) );
	geometryData->wireframeIndexBuffer.reset( new MIndexBuffer(MGeometry::kUnsignedInt32) );

	int numElementsInBuffer = 0;
	for( unsigned int i = 0; i < curvesPrimitive->numCurves(); ++i )
	{
		numElementsInBuffer += curvesPrimitive->numSegments( i ) * 2;
	}

	void* wireframeIndexData = geometryData->wireframeIndexBuffer->acquire( numElementsInBuffer, true );
	unsigned int* tmp = (unsigned int*) wireframeIndexData;

	int positionBufferOffset = 0;
	int indexBufferOffset = 0;
	for( unsigned int i = 0; i < curvesPrimitive->numCurves(); ++i )
	{
		int numSegments = curvesPrimitive->numSegments( i );
		int endPointDuplication = ( verticesPerCurveReadable[i] - ( numSegments + 1 ) ) / 2;
		int segmentStart = positionBufferOffset + endPointDuplication;
		for( int j = 0; j < numSegments; ++j )
		{
			tmp[indexBufferOffset*2 + 0] = segmentStart;
			tmp[indexBufferOffset*2 + 1] = segmentStart + 1;

			indexBufferOffset++;
			segmentStart++;
		}
		positionBufferOffset += verticesPerCurveReadable[i];
	}

	// use the same buffer for both wireframe and shaded rendering
	geometryData->wireframeIndexBuffer->commit( wireframeIndexData );
	geometryData->indexBuffer.reset( geometryData->wireframeIndexBuffer.get() );
}

void fillMeshData( ConstPrimitivePtr primitive, const MVertexBufferDescriptorList &descriptorList, GeometryDataPtr geometryData )
{
	IECoreScene::ConstMeshPrimitivePtr meshPrimitive = IECore::runTimeCast<const IECoreScene::MeshPrimitive>( primitive );

	// \todo: Ideally we wouldn't copy a whole lot of data here, but just get an index buffer for tris.
	IECoreScene::MeshPrimitivePtr meshPrimitiveTriangulated = IECore::runTimeCast<IECoreScene::MeshPrimitive>( meshPrimitive->copy() );
	IECoreScene::TriangulateOpPtr op( new IECoreScene::TriangulateOp() );
	op->inputParameter()->setValue( meshPrimitiveTriangulated );
	op->throwExceptionsParameter()->setTypedValue( false ); // it's better to see something than nothing
	op->copyParameter()->setTypedValue( false );
	op->operate();

	// Get handles to relevant primitive variables.
	IECore::ConstV3fVectorDataPtr p = meshPrimitiveTriangulated->variableData<IECore::V3fVectorData>( "P" );
	IECore::ConstIntVectorDataPtr indicesTriangulated = meshPrimitiveTriangulated->vertexIds();
	IECore::ConstIntVectorDataPtr wireframeIndicesOriginal = meshPrimitive->vertexIds();

	// \todo: Room for optimization - we potentially already have normals in the data.
	IECoreScene::MeshNormalsOpPtr normalOp = new IECoreScene::MeshNormalsOp();
	normalOp->inputParameter()->setValue( meshPrimitiveTriangulated );
	normalOp->copyParameter()->setTypedValue( false );
	bool linearInterpolation = meshPrimitive->interpolation() == "linear";
	normalOp->interpolationParameter()->setNumericValue( linearInterpolation ? IECoreScene::PrimitiveVariable::Uniform : IECoreScene::PrimitiveVariable::Vertex );
	normalOp->operate();

	IECore::ConstV3fVectorDataPtr n = meshPrimitiveTriangulated->variableData<IECore::V3fVectorData>( "N" );
	IECore::ConstV2fVectorDataPtr uv = meshPrimitiveTriangulated->expandedVariableData<IECore::V2fVectorData>( "uv", IECoreScene::PrimitiveVariable::FaceVarying );

	// We need topology information to render wireframes. When changing the
	// data in `ensureFaceVaryingData`, we invalidate these indices, but the
	// function allows passing a list of indices to be updated along with the
	// data.
	std::vector<int> wireframeIndicesReadable;
	if( wireframeIndicesOriginal )
	{
		wireframeIndicesReadable = std::vector<int>( wireframeIndicesOriginal->readable() );
	}

 	// Expand out positions and normals so that they line up with UVs
	ensureFaceVaryingData( indicesTriangulated, p, n, wireframeIndicesReadable, linearInterpolation );

	// When preparing that data in the following, we try to respect the
	// descriptors that maya gives us. This is mostly to provide correct UVs
	// because it seems that the required semantic name and the buffer name
	// sometimes change.
	if( p )
	{
		void* positionData = nullptr;
		MVertexBuffer *positionBuffer = nullptr;
		MVertexBufferDescriptor positionDescriptor( "", MGeometry::kPosition, MGeometry::kFloat, 3 );

		const std::vector<Imath::V3f> &positionsReadable = p->readable();
		for( int i = 0; i < descriptorList.length(); ++i )
		{
			MVertexBufferDescriptor descriptor;
			descriptorList.getDescriptor( i, descriptor );
			if( descriptor.semantic() == MGeometry::kPosition )
			{
				positionDescriptor = descriptor;
				break;
			}
		}

		positionBuffer = new MVertexBuffer( positionDescriptor );
		positionData = positionBuffer->acquire( positionsReadable.size(), true );

		if( positionData && positionBuffer )
		{
			memcpy( positionData, positionsReadable.data(), sizeof( float ) * 3 * positionsReadable.size() );
			positionBuffer->commit( positionData );
			geometryData->vertexBufferArray->addBuffer( "positions", positionBuffer );
		}
	}

	if( n )
	{
		void* normalData = nullptr;
		MVertexBuffer *normalBuffer = nullptr;
		MVertexBufferDescriptor normalDescriptor( "", MGeometry::kNormal, MGeometry::kFloat, 3 );

		const std::vector<Imath::V3f> &normalsReadable = n->readable();
		for( int i = 0; i < descriptorList.length(); ++ i )
		{
			MVertexBufferDescriptor descriptor;
			descriptorList.getDescriptor( i, descriptor );
			if( descriptor.semantic() == MGeometry::kNormal )
			{
				normalDescriptor = descriptor;
				break;
			}
		}

		normalBuffer = new MVertexBuffer( normalDescriptor );
		normalData = normalBuffer->acquire( normalsReadable.size(), true );

		if( normalData && normalBuffer )
		{
			memcpy( normalData, normalsReadable.data(), sizeof( float ) * 3 * normalsReadable.size() );
			normalBuffer->commit( normalData );
			geometryData->vertexBufferArray->addBuffer( "normals", normalBuffer );
		}
	}

	if( uv )
	{
		void* uvData = nullptr;
		MVertexBuffer *uvBuffer = nullptr;
		const std::vector<Imath::V2f> &uvReadable = uv->readable();

		// Default descriptor. Maya sometimes requires the name to be 'uvCoord'
		// and the semantic name to be 'mayauvcoordsemantic'. By respecting the
		// given descriptor below, we should supply data that maya can work with.
		MVertexBufferDescriptor uvDescriptor( "", MGeometry::kTexture, MGeometry::kFloat, 2 );

		for( int i = 0; i < descriptorList.length(); ++ i )
		{
			MVertexBufferDescriptor descriptor;
			descriptorList.getDescriptor( i, descriptor );
			if( descriptor.semantic() == MGeometry::kTexture )
			{
				uvDescriptor = descriptor;
				break;
			}
		}

		uvBuffer = new MVertexBuffer( uvDescriptor );
		uvData = uvBuffer->acquire( uvReadable.size(), true );

		if( uvData && uvBuffer )
		{
			memcpy( uvData, uvReadable.data(), sizeof( float ) * 2 * uvReadable.size() );
			uvBuffer->commit( uvData );
			geometryData->vertexBufferArray->addBuffer( "uvs", uvBuffer );
		}
	}

	// NOTE: Once Maya supports UnsignedInt16 for MIndexBuffers, we should
	// consider switching the data type here based on the number of vertices in
	// our mesh.
	geometryData->indexBuffer.reset( new MIndexBuffer(MGeometry::kUnsignedInt32) );
	geometryData->wireframeIndexBuffer.reset( new MIndexBuffer(MGeometry::kUnsignedInt32) );

	if( !indicesTriangulated )
	{
		return;
	}

	// Prepare the index buffer for rendering a solid mesh

	const std::vector<int> &indexReadable = indicesTriangulated->readable();
	void* indexData = geometryData->indexBuffer->acquire( indexReadable.size(), true );
	if( indexData )
	{
		memcpy( indexData, indexReadable.data(), sizeof(int) * indexReadable.size() );
		geometryData->indexBuffer->commit( indexData );
	}

	// Prepare the index buffer for rendering mesh as wireframe.
	// NOTE: Currently every edge is pushed through TWICE. Initially, we had an
	// implementation that made edges unique by using STL containers, but it turns
	// out it's MUCH faster to just hand off both edges. Until we run into issues
	// where the amount of data is the bottleneck, we should probably keep things
	// the way they are now.
	const IECore::IntVectorData *numVerticesPerFace = meshPrimitive->verticesPerFace();
	if( numVerticesPerFace )
	{
		const std::vector<int> &numVerticesPerFaceReadable = numVerticesPerFace->readable();

		int totalNumEdgeIndices = std::accumulate( numVerticesPerFaceReadable.begin(), numVerticesPerFaceReadable.end(), 0 ) * 2;
		void* wireframeIndexData = geometryData->wireframeIndexBuffer->acquire( totalNumEdgeIndices, true );
		unsigned int* tmp = (unsigned int*) wireframeIndexData;

		int offset = 0;
		int indexBufferOffset = 0;
		for( size_t j = 0; j < numVerticesPerFaceReadable.size(); ++j )
		{
			int numVerts = numVerticesPerFaceReadable[j];
			for( int k = 0; k < numVerts; ++k )
			{
				tmp[indexBufferOffset*2 + 0] = wireframeIndicesReadable[offset + k];
				tmp[indexBufferOffset*2 + 1] = wireframeIndicesReadable[offset + (k+1)%numVerts];

				indexBufferOffset++;
			}
			offset += numVerts;
		}

		geometryData->wireframeIndexBuffer->commit( wireframeIndexData );
	}
}

GeometryDataPtr boundGetter( const BoundDataCacheGetterKey &key, size_t &cost )
{
	cost = 1;

	GeometryDataPtr boundingBoxData( new GeometryData() );
	fillBoundData( key.m_bounds, boundingBoxData );
	return boundingBoxData;
}

// \todo: should compute an estimate of the cost in terms of GPU memory so we can limit how much we're using.
//        Maya lets us query the available GPU memory via MRenderer.
GeometryDataPtr geometryGetter( const GeometryDataCacheGetterKey &key, size_t &cost )
{
	cost = 1;

	GeometryDataPtr geometryData( new GeometryData() );

	switch( static_cast<IECoreScene::TypeId>( key.m_primitive->typeId() ) )
	{
	case IECoreScene::TypeId::MeshPrimitiveTypeId :
		fillMeshData( key.m_primitive, key.m_descriptorList, geometryData );
		break;
	case IECoreScene::TypeId::PointsPrimitiveTypeId :
		fillPointsData( key.m_primitive, geometryData );
		break;
	case IECoreScene::TypeId::CurvesPrimitiveTypeId :
		fillCurvesData( key.m_primitive, geometryData );
		break;
	default :
		// we should never get here as only the above types are supported, see `objectCanBeRendered`.
		break;
	}

	return geometryData;
}

// Cache for BoundData
using BoundDataCache = IECore::LRUCache<IECore::MurmurHash, GeometryDataPtr, IECore::LRUCachePolicy::Parallel, BoundDataCacheGetterKey> ;
BoundDataCache g_boundDataCache(boundGetter, 100);

// Cache for mesh data
using GeometryDataCache = IECore::LRUCache<IECore::MurmurHash, GeometryDataPtr, IECore::LRUCachePolicy::Parallel, GeometryDataCacheGetterKey>;
GeometryDataCache g_geometryDataCache(geometryGetter, 100);

MString renderItemName( const std::string &baseName, RenderStyle style )
{
	switch( style )
	{
	case RenderStyle::BoundingBox :
		return MString( (baseName + "_bb").c_str() );
	case RenderStyle::Wireframe :
		return MString( (baseName + "_wf").c_str() );
	case RenderStyle::Solid :
		return MString( (baseName + "_s").c_str() );
	case RenderStyle::Textured :
		return MString( (baseName + "_t").c_str() );
	default :
		return MString();
	}
}

MRenderItem *getRenderItem( MSubSceneContainer &container, ConstObjectPtr object, const MString &name, const Imath::Box3f &boundingBox, RenderStyle style, bool &isEmpty )
{
	MRenderItem *renderItem = container.find( name );
	if( renderItem )
	{
		renderItem->enable( true );
		isEmpty = false; // this comes from the container as was assigned some geo before.
		return renderItem;
	}

	// If the container does not have an appropriate MRenderItem yet, we'll construct an empty one.
	isEmpty = true;

	// GeometryDataPtr geometryData;
	switch( style )
	{
	case RenderStyle::BoundingBox :
	{
		renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kLines);
		renderItem->setDrawMode( MGeometry::kAll );
		renderItem->castsShadows( false );
		renderItem->receivesShadows( false );
		renderItem->setExcludedFromPostEffects( true );
		break;
	}
	case RenderStyle::Wireframe :
	{
		switch( static_cast<IECoreScene::TypeId>( object->typeId() ) )
		{
		case IECoreScene::TypeId::PointsPrimitiveTypeId :
			renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kPoints );
			break;
		case IECoreScene::TypeId::CurvesPrimitiveTypeId :
		case IECoreScene::TypeId::MeshPrimitiveTypeId :
			renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kLines );
			break;
		default :
			return nullptr;
		}
		renderItem->setDrawMode( MGeometry::kAll );
		renderItem->castsShadows( false );
		renderItem->receivesShadows( false );
		renderItem->setExcludedFromPostEffects( true );
		renderItem->depthPriority( MRenderItem::sActiveWireDepthPriority );
		break;
	}
	case RenderStyle::Solid :
	case RenderStyle::Textured :
	{
		switch( static_cast<IECoreScene::TypeId>( object->typeId() ) )
		{
			case IECoreScene::TypeId::PointsPrimitiveTypeId :
			{
				renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kPoints );
				break;
			}
			case IECoreScene::TypeId::MeshPrimitiveTypeId :
			{
				renderItem = MRenderItem::Create( name, MRenderItem::MaterialSceneItem, MGeometry::kTriangles );
				break;
			}
			case IECoreScene::TypeId::CurvesPrimitiveTypeId :
			{
				renderItem = MRenderItem::Create( name, MRenderItem::MaterialSceneItem, MGeometry::kLines );
				break;
			}
		default :
			break;

		}

		renderItem->setDrawMode( ( style == RenderStyle::Solid ) ? MGeometry::kShaded : MGeometry::kTextured );
		renderItem->castsShadows( true );
		renderItem->receivesShadows( true );
		renderItem->setExcludedFromPostEffects( false );
	}
	default :
		break;
	}

	// if( !renderItem )
	// {
	// 	throw( IECore::Exception( boost::str( boost::format( "Could not create MRenderItem %1%." ) % name.asChar() ) ) );
	// }

	return renderItem;
}

// RenderItemWrapperPtr renderItemGetter( const RenderItemWrapperCacheGetterKey &key, size_t &cost )
// {
// 	cost = 1; // \todo: should do a better job at estimating the cost

// 	const MBoundingBox mayaBoundingBox = IECore::convert<MBoundingBox>( key.m_boundingBox );
// 	MString name = renderItemName( key.m_name, key.m_style );

// 	// \note: data can only be added to an MRenderItem if it has been added to
// 	// the container and has a shader assigned that determines the geometry
// 	// requirements.
// 	MRenderItem *renderItem = nullptr;
// 	GeometryDataPtr geometryData;
// 	switch( key.m_style )
// 	{
// 	case RenderStyle::BoundingBox :
// 	{
// 		renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kLines);

// 		renderItem->setDrawMode( MGeometry::kAll );
// 		renderItem->castsShadows( false );
// 		renderItem->receivesShadows( false );
// 		renderItem->setExcludedFromPostEffects( true );

// 		renderItem->setShader( key.m_shader );
// 		key.m_container.add( renderItem );

// 		geometryData = g_boundDataCache.get( BoundDataCacheGetterKey( key.m_boundingBox ) );
// 		key.m_sceneShape->setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->wireframeIndexBuffer), &mayaBoundingBox );
// 		break;
// 	}
// 	case RenderStyle::Wireframe :
// 	{
// 		ConstPrimitivePtr primitive = IECore::runTimeCast<const Primitive>( key.m_object );
// 		if( !primitive )
// 		{
// 			break;
// 		}

// 		switch( static_cast<IECoreScene::TypeId>( primitive->typeId() ) )
// 		{
// 		case IECoreScene::TypeId::PointsPrimitiveTypeId :
// 			renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kPoints );
// 			break;
// 		case IECoreScene::TypeId::CurvesPrimitiveTypeId :
// 		case IECoreScene::TypeId::MeshPrimitiveTypeId :
// 			renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kLines );
// 			break;
// 		default :
// 			return nullptr;
// 		}

// 		geometryData = g_geometryDataCache.get( GeometryDataCacheGetterKey( primitive, renderItem->requiredVertexBuffers() ) );

// 		renderItem->setDrawMode( MGeometry::kAll );
// 		renderItem->castsShadows( false );
// 		renderItem->receivesShadows( false );
// 		renderItem->setExcludedFromPostEffects( true );
// 		renderItem->depthPriority( MRenderItem::sActiveWireDepthPriority );

// 		renderItem->setShader( key.m_shader );
// 		key.m_container.add( renderItem );

// 		key.m_sceneShape->setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->wireframeIndexBuffer), &mayaBoundingBox );
// 		break;
// 	}
// 	case RenderStyle::Solid :
// 	case RenderStyle::Textured :
// 	{

// 		ConstPrimitivePtr primitive = IECore::runTimeCast<const Primitive>( key.m_object );
// 		if( !primitive )
// 		{
// 			break;
// 		}

// 		switch( static_cast<IECoreScene::TypeId>( primitive->typeId() ) )
// 		{
// 			case IECoreScene::TypeId::PointsPrimitiveTypeId :
// 			{
// 				renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kPoints );
// 				break;
// 			}
// 			case IECoreScene::TypeId::MeshPrimitiveTypeId :
// 			{
// 				renderItem = MRenderItem::Create( name, MRenderItem::MaterialSceneItem, MGeometry::kTriangles );
// 				break;
// 			}
// 			case IECoreScene::TypeId::CurvesPrimitiveTypeId :
// 			{
// 				renderItem = MRenderItem::Create( name, MRenderItem::MaterialSceneItem, MGeometry::kLines );
// 				break;
// 			}
// 		default :
// 			break;

// 		}

// 		geometryData = g_geometryDataCache.get( GeometryDataCacheGetterKey( primitive, renderItem->requiredVertexBuffers() ) );

// 		renderItem->setDrawMode( (key.m_style == RenderStyle::Solid) ? MGeometry::kShaded : MGeometry::kTextured );
// 		renderItem->castsShadows( true );
// 		renderItem->receivesShadows( true );
// 		renderItem->setExcludedFromPostEffects( false );

// 		renderItem->setShader( key.m_shader );
// 		key.m_container.add( renderItem );

// 		key.m_sceneShape->setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->indexBuffer), &mayaBoundingBox );
// 	}
// 	default :
// 		break;
// 	}

// 	if( !renderItem )
// 	{
// 		throw( IECore::Exception( boost::str( boost::format( "Could not create MRenderItem %1%." ) % key.m_name ) ) );
// 	}

// 	// Return a copy that can be handed to Maya. We will not give up our
// 	// internal copy as it's too easy to accidentally get it cleaned up by Maya.
// 	RenderItemWrapperPtr visibleCopy( new RenderItemWrapper( MRenderItem::Create( *renderItem ), geometryData ) );

// 	return visibleCopy;
// }

// using RenderItemCache = IECore::LRUCache<IECore::MurmurHash, RenderItemWrapperPtr, IECore::LRUCachePolicy::Parallel, RenderItemWrapperCacheGetterKey>;
// RenderItemCache g_renderItemCache(renderItemGetter, 10000);

// For the given node return the out plug on the surface shader that is assigned
MPlug getShaderOutPlug( const MObject &sceneShapeNode )
{
	MPlug result = MPlug();

	MObjectArray sets, components;
	MFnDagNode node( sceneShapeNode );

	if( !node.getConnectedSetsAndMembers( 0, sets, components, true ) )
	{
		return result;
	}

	for (unsigned int i = 0; i < sets.length(); i++)
	{
		MStatus status;
		MFnDependencyNode fnSet( sets[i], &status );

		if( !status )
		{
			return result;
		}

		MPlug shaderPlug = fnSet.findPlug( "surfaceShader" );
		if( shaderPlug.isNull() )
		{
			return result;
		}

		MPlugArray connectedPlugs;
		shaderPlug.connectedTo( connectedPlugs, true, false );

		if( connectedPlugs.length() >= 1 )
		{
			return connectedPlugs[0];
		}
	}

	return result;
}

bool isPathSelected( const MSelectionList &selectionList, const MDagPath &path )
{
    MStatus status = MStatus::kSuccess;
	MDagPath pathCopy = path;

	while( status )
	{
		if( selectionList.hasItem( pathCopy ) )
		{
			return true;
		}

		status = pathCopy.pop();
	}

	return false;
}

struct SelectionStateShaders
{
	SelectionStateShaders( MHWRender::MShaderInstance *unselected, MHWRender::MShaderInstance *selected )
		: unselected( unselected ), selected( selected )
	{
	}

	MHWRender::MShaderInstance *unselected;
	MHWRender::MShaderInstance *selected;
};

struct TextureStateShaders
{
	TextureStateShaders( MHWRender::MShaderInstance *untextured, MHWRender::MShaderInstance *textured )
		: untextured( untextured ), textured( textured )
	{
	}

	MHWRender::MShaderInstance *untextured;
	MHWRender::MShaderInstance *textured;
};

SelectionStateShaders getWireShaders()
{
	static SelectionStateShaders shaders( nullptr, nullptr );

	if( !shaders.unselected || !shaders.selected )
	{
		MRenderer* renderer = MRenderer::theRenderer();
		if( !renderer )
		{
			return shaders;
		}

		const MShaderManager* shaderManager = renderer->getShaderManager();
		if( !shaderManager )
		{
			return shaders;
		}

		shaders.unselected = shaderManager->getStockShader( MShaderManager::k3dSolidShader );
		shaders.selected = shaders.unselected->clone();

		MDoubleArray a, b;
		MGlobal::executeCommand( "colorIndex -q 19", a );
		MGlobal::executeCommand( "colorIndex -q 5", b );

		const float highlightedWireColor[] = {(float)a[0], (float)a[1], (float)a[2], 1.0f};
		const float unhighlightedWireColor[] = {(float)b[0], (float)b[1], (float)b[2], 1.0f};

		shaders.unselected->setParameter("solidColor", unhighlightedWireColor );
		shaders.selected->setParameter("solidColor", highlightedWireColor );
	}

	return shaders;
}

SelectionStateShaders getComponentWireShaders()
{
	static SelectionStateShaders shaders( nullptr, nullptr );

	if( !shaders.unselected || !shaders.selected )
	{
		MRenderer* renderer = MRenderer::theRenderer();
		if( !renderer )
		{
			return shaders;
		}

		const MShaderManager* shaderManager = renderer->getShaderManager();
		if( !shaderManager )
		{
			return shaders;
		}

		MDoubleArray a, b;
		MGlobal::executeCommand( "colorIndex -q 21", a );
		MGlobal::executeCommand( "colorIndex -q 18", b );

		const float highlightedWireColor[] = {(float)a[0], (float)a[1], (float)a[2], 1.0f};
		const float unhighlightedWireColor[] = {(float)b[0], (float)b[1], (float)b[2], 1.0f};

		shaders.unselected = shaderManager->getStockShader( MShaderManager::k3dSolidShader );
		shaders.selected = shaders.unselected->clone();

		shaders.unselected->setParameter("solidColor", unhighlightedWireColor );
		shaders.selected->setParameter("solidColor", highlightedWireColor );
	}

	return shaders;
}

TextureStateShaders getAssignedSurfaceShaders( const MObject &object )
{
	TextureStateShaders shaders( nullptr, nullptr );

	MRenderer* renderer = MRenderer::theRenderer();
	if( !renderer )
	{
		return shaders;
	}

	const MShaderManager* shaderManager = renderer->getShaderManager();
	if( !shaderManager )
	{
		return shaders;
	}

	MStatus s;
	MFnDagNode node( object, &s );
	if( !s )
	{
		return shaders;
	}

	MDagPathArray instances;
	node.getAllPaths( instances );

	MPlug shaderOutPlug = getShaderOutPlug( object );

	if( !shaderOutPlug.isNull() )
	{
		shaders.untextured = shaderManager->getShaderFromNode( shaderOutPlug.node(), instances[0], 0, 0, 0, 0, /* nonTextured = */ true );
		shaders.textured = shaderManager->getShaderFromNode( shaderOutPlug.node(), instances[0], 0, 0, 0, 0, /* nonTextured = */ false );
	}
	else
	{
		shaders.untextured = shaderManager->getStockShader( MShaderManager::MStockShader::k3dDefaultMaterialShader );
		shaders.textured = shaders.untextured;
	}

	return shaders;
}

MShaderInstance* getShader( const MObject &object, RenderStyle style, bool componentMode, bool isSelected )
{
	switch( style )
		{
		case RenderStyle::BoundingBox :
		case RenderStyle::Wireframe :
		{
			if( componentMode )
			{
				SelectionStateShaders shaders = getComponentWireShaders();
				return isSelected ? shaders.selected : shaders.unselected;
			}
			else
			{
				SelectionStateShaders shaders = getWireShaders();
				return isSelected ? shaders.selected : shaders.unselected;
			}
		}
		case RenderStyle::Solid :
		case RenderStyle::Textured :
			{
				TextureStateShaders shaders = getAssignedSurfaceShaders( object );
				return style == RenderStyle::Solid ? shaders.untextured : shaders.textured;
			}
		default :
			return nullptr;
		}
}

bool componentsSelectable( const MDagPath &path )
{
	MHWRender::DisplayStatus displayStatus = MHWRender::MGeometryUtilities::displayStatus( path );
	bool selectable = displayStatus == MHWRender::kHilite;
	return selectable;
}

// Convenience function that should eventually help with also rendering all instances generated by instancers.
// Currently not in use, but will come in handy when we get to look into MASH support.
MObjectArray getConnectedInstancers( MObject object )
{
	MObjectArray result;

	MStatus status;
	MObject current;

	MDagPathArray paths;
	MMatrixArray matrices;
	MIntArray pathStartIndices;
	MIntArray pathIndices;

	MItDependencyNodes it( MFn::kInstancer );
	for( ; !it.isDone(); it.next() )
	{
		MObject currentInstancer = it.item();
		MFnInstancer instFn( currentInstancer );

		MStatus status;
		MPlug inputsPlug = instFn.findPlug( "inputHierarchy", &status );
		if( !status )
		{
			continue;
		}

		// We will need to figure out if our SceneShape feeds into this instancer.
		for( unsigned int i = 0; i < inputsPlug.numElements(); ++i )
		{
			MPlug element = inputsPlug.elementByLogicalIndex( i );
			MPlugArray srcPlugs;
			element.connectedTo( srcPlugs, true, false );

			bool found = false;
			for(unsigned int i=0; i < srcPlugs.length(); ++i)
			{
				MFnDagNode connectedNode(srcPlugs[i].node());
				if( connectedNode.object() == object || connectedNode.hasChild( object ) )
				{
					result.append( currentInstancer );
					found = true;
					break;
				}
			}

			if( found )
			{
				break;
			}
		}
	}

	return result;
}

Imath::M44d worldTransform( const IECoreScene::SceneInterface *scene, double time )
{
	IECoreScene::SceneInterface::Path p;
	scene->path( p );

	IECoreScene::ConstSceneInterfacePtr tmpScene = scene->scene( IECoreScene::SceneInterface::rootPath );
	Imath::M44d result;

	for ( IECoreScene::SceneInterface::Path::const_iterator it = p.begin(); tmpScene && it != p.end(); ++it )
	{
		tmpScene = tmpScene->child( *it, IECoreScene::SceneInterface::NullIfMissing );
		if ( !tmpScene )
		{
			break;
		}

		result = tmpScene->readTransformAsMatrix( time ) * result;
	}

	return result;
}

bool sceneIsAnimated( ConstSceneInterfacePtr sceneInterface )
{
	ConstSampledSceneInterfacePtr scene = IECore::runTimeCast< const SampledSceneInterface >( sceneInterface );
	return ( !scene || scene->numBoundSamples() > 1 );
}

void appendMatrixToRenderItem( RenderItemMap &renderItems, const std::string &name, MRenderItem *renderItem, MMatrix instanceMatrix )
{
	auto it = renderItems.find( name );
	if( it == renderItems.end() )
		{
			MMatrixArray array = MMatrixArray();
			it = renderItems.emplace( name, std::make_pair( renderItem, array ) ).first;
		}

	it->second.second.append( instanceMatrix );
}

void selectedComponentIndices( const MObject &object, IndexMap &indexMap )
{
	// IECore::ScopedTimer foo( "selectedComponentIndices" );

	// printf( "called\n" );
	MStatus s;

	MSelectionList selectionList;
	MGlobal::getActiveSelectionList( selectionList );
	MItSelectionList selectionIter( selectionList );

	// printf( "length: %i\n", (int)selectionList.length() );

	MFnDagNode dagNode( object );
	MDagPathArray dagPaths;
	dagNode.getAllPaths(dagPaths);

	// Initialize map with empty sets
	for( int i = 0; i < (int)dagPaths.length(); ++i )
	{
		std::string keyPath = dagPaths[i].fullPathName().asChar();
		indexMap[keyPath] = std::set<int>();
	}

	for( ; !selectionIter.isDone(); selectionIter.next() )
	{
		MDagPath selectedPath; // path to shape
		MObject comp;
		selectionIter.getDagPath( selectedPath, comp );

		if( comp.isNull() )
		{
			continue;
		}

		MFnSingleIndexedComponent compFn( comp, &s );
		if( !s )
		{
			continue;
		}

		MIntArray componentIndices;
		compFn.getElements( componentIndices );

		std::string key = selectedPath.fullPathName().asChar();
		for( unsigned int i = 0; i < componentIndices.length(); ++i )
		{
			indexMap[key].insert( componentIndices[i] );
		}
	}
}

} // namespace

MString& SceneShapeSubSceneOverride::drawDbClassification()
{
	static MString classification{ "drawdb/subscene/ieSceneShape" };
	return classification;
}

MString& SceneShapeSubSceneOverride::drawDbId()
{
	static MString id{ "SceneShapeSubSceneOverride" };
	return id;
}

MPxSubSceneOverride* SceneShapeSubSceneOverride::Creator( const MObject& obj )
{
	return new SceneShapeSubSceneOverride( obj );
}

SceneShapeSubSceneOverride::~SceneShapeSubSceneOverride()
{
}

bool SceneShapeSubSceneOverride::requiresUpdate(const MSubSceneContainer& container, const MFrameContext& frameContext) const
{
	IECore::ScopedTimer foo( "reqUpdate" );

	// TIME UPDATED?
	if( m_sceneShape->time() != m_time && sceneIsAnimated( m_sceneInterface ) )
	{
		return true;
	}

	// SCENE UPDATED?
	if( m_sceneInterface != m_sceneShape->getSceneInterface() )
	{
		return true;
	}

	// DRAW GEOMETRY SETTINGS UPDATED?
	const unsigned int displayStyle = frameContext.getDisplayStyle();
	bool renderBounds = renderAllBounds( displayStyle );
	bool renderWireframes = renderAllWireframes( displayStyle );
	bool renderShaded = renderAllShaded( displayStyle );

	// Determine if display settings need to trigger update
	if( renderBounds != m_styleMask.test( (int)RenderStyle::BoundingBox ) )
	{
		return true;
	}

	if( renderWireframes != m_styleMask.test( (int)RenderStyle::Wireframe ) )
	{
		return true;
	}

	if( renderShaded != m_styleMask.test( (int)RenderStyle::Solid ) )
	{
		return true;
	}

	// ROOT BOUNDING BOX SETTING UPDATED?
	bool tmpDrawRootBound;
	MPlug drawRootBoundsPlug( m_sceneShape->thisMObject(), SceneShape::aDrawRootBound );
	drawRootBoundsPlug.getValue( tmpDrawRootBound );
	if( tmpDrawRootBound != m_drawRootBounds )
	{
		return true;
	}

	// TAGS FILTER UPDATED?
	MString tmpTagsFilter;
	MPlug drawTagsFilterPlug( m_sceneShape->thisMObject(), SceneShape::aDrawTagsFilter );
	drawTagsFilterPlug.getValue( tmpTagsFilter );
	if( tmpTagsFilter.asChar() != m_drawTagsFilter )
	{
		return true;
	}

	// \todo: This should scale better. It's depending on both the selection and the instances at the moment.
	// COMPONENT SELECTION UPDATED?
	IndexMap selectedComponents;
	selectedComponentIndices( m_sceneShape->thisMObject(), selectedComponents );
	if( selectedComponents != m_selectedComponents )
	{
		return true;
	}

	// SHADER UPDATED?
	if( getShaderOutPlug( m_sceneShape->thisMObject() ) != m_shaderOutPlug )
	{
		return true;
	}

	Instances currentInstances;
	gatherInstanceInformation( currentInstances, renderWireframes );

	if( currentInstances.size() != m_instances.size() )
	{
		return true; // instance was added or removed
	}

	for( const auto &instance : currentInstances )
	{
		// \todo: doesn't properly account for duplicates in the containers, but is probably good enough for now.
		if( std::find( m_instances.begin(), m_instances.end(), instance ) == m_instances.end() )
		{
			return true;
		}
	}

	return false;
}

void SceneShapeSubSceneOverride::update( MSubSceneContainer& container, const MFrameContext& frameContext )
{
	// \todo: if the scene was changed - we need to empty the container completely.

	// We'll set internal state based on settings in maya and then perform
	// updates by walking the tree.

	m_time = m_sceneShape->time();
	m_sceneInterface = m_sceneShape->getSceneInterface();

	// STYLE
	const unsigned int displayStyle = frameContext.getDisplayStyle();

	bool renderBounds( renderAllBounds( displayStyle ) );
	bool renderWireframes( renderAllWireframes( displayStyle ) );
	bool renderShaded( renderAllShaded( displayStyle ) );

	renderWireframes ? m_styleMask.set( (int)RenderStyle::Wireframe ) : m_styleMask.reset( (int)RenderStyle::Wireframe );
	renderShaded     ? m_styleMask.set( (int)RenderStyle::Solid ) : m_styleMask.reset( (int)RenderStyle::Solid );
	renderBounds     ? m_styleMask.set( (int)RenderStyle::BoundingBox ) : m_styleMask.reset( (int)RenderStyle::BoundingBox );

	// DRAWING ROOTS
	MPlug drawRootBoundsPlug( m_sceneShape->thisMObject(), SceneShape::aDrawRootBound );
	drawRootBoundsPlug.getValue( m_drawRootBounds );

	// TAGS
	MString tmpTagsFilter;
	MPlug drawTagsFilterPlug( m_sceneShape->thisMObject(), SceneShape::aDrawTagsFilter );
	drawTagsFilterPlug.getValue( tmpTagsFilter );
	if( tmpTagsFilter.asChar() != m_drawTagsFilter )
	{
		m_drawTagsFilter = tmpTagsFilter.asChar();
	}

	// COMPONENT SELECTION
	m_selectedComponents.clear();
	selectedComponentIndices( m_sceneShape->thisMObject(), m_selectedComponents );

	// INSTANCES - sort out transformation and selection
	m_instances.clear();
	gatherInstanceInformation( m_instances, renderWireframes );

	// SHADING
	MPlug currentShaderOutPlug = getShaderOutPlug( m_sceneShape->thisMObject() );
	if( currentShaderOutPlug.name() != m_shaderOutPlug.name() )
	{
		m_shaderOutPlug = currentShaderOutPlug;
		m_materialIsDirty = true;
	}

	// Perform update
	m_renderItemNameToDagPath.clear();
	// container.clear(); // Clear out previous render items so that they don't interfere with new ones.

	// Disable all MRenderItems before traversing the scene
	// \todo: performance improvement -> we don't have to do this is all that changed is a shader (selection state) for example.
	MSubSceneContainer::Iterator *it = container.getIterator();
	MRenderItem *renderItem = nullptr;
	while( (renderItem = it->next()) != nullptr )
	{
		renderItem->enable( false );
	}
	it->destroy();

	// Store MRenderItems to be added while walking the tree.
	RenderItemMap renderItems;
	visitSceneLocations( m_sceneInterface, renderItems, container, Imath::M44d(), /* isRoot = */ true );

	// container.clear(); // MRenderItems might have been added temporarily so that Maya lets us fill them

	// // Actually add all gathered MRenderItems while using instancing where possible
	// for( auto namedRenderItem : renderItems )
	// {
	// 	auto itemAndMatrices = namedRenderItem.second;
	// 	MRenderItem *renderItem = itemAndMatrices.first;

	// 	if( !renderItem )
	// 	{
	// 		continue;
	// 	}

	// 	// \todo
	// 	container.add( renderItem );

	// 	// note: the following is true for all entries if
	// 	// !m_instancedRendering because we're creating a separate
	// 	// render item per instance
	// 	if( itemAndMatrices.second.length() == 1 )
	// 	{
	// 		renderItem->setWantSubSceneConsolidation( true );
	// 		renderItem->setMatrix( &itemAndMatrices.second[0] );
	// 	}
	// 	else
	// 	{
	// 		renderItem->setWantSubSceneConsolidation( false );
	// 		setInstanceTransformArray( *renderItem, itemAndMatrices.second );
	// 		// \todo: we should make sure that we can select individual
	// 		// instances. For now instanced rendering is disabled anyway,
	// 		// though, because of a problem in Maya that I don't quite
	// 		// understand.
	// 	}
	// }
}

bool SceneShapeSubSceneOverride::getInstancedSelectionPath( const MRenderItem &renderItem, const MIntersection &intersection, MDagPath &dagPath ) const
{
	auto it = m_renderItemNameToDagPath.find( std::string( renderItem.name().asChar() ) );
	if( it != m_renderItemNameToDagPath.end() )
	{
		dagPath.set( it->second );
		return true;
	}

	return false;
}

void SceneShapeSubSceneOverride::updateSelectionGranularity( const MDagPath &path, MSelectionContext &selectionContext )
{
	MDagPath parent( path );
	parent.pop();

	if( componentsSelectable( parent ) )
	{
		selectionContext.setSelectionLevel( MHWRender::MSelectionContext::kComponent );
	}
	else
	{
		selectionContext.setSelectionLevel( MHWRender::MSelectionContext::kObject );
	}
}

DrawAPI SceneShapeSubSceneOverride::supportedDrawAPIs() const
{
	return kAllDevices;
}

SceneShapeSubSceneOverride::SceneShapeSubSceneOverride( const MObject& obj )
	: MPxSubSceneOverride( obj ), m_drawTagsFilter( "" ), m_time( -1 ), m_drawRootBounds( false ), m_shaderOutPlug(), m_materialIsDirty( true ), m_instancedRendering( false /* instancedRendering switch */ )
{
	MStatus status;
	MFnDependencyNode node( obj, &status );

	if( status )
	{
		m_sceneShape = dynamic_cast<SceneShape*>( node.userNode() );
	}
}

void SceneShapeSubSceneOverride::visitSceneLocations( ConstSceneInterfacePtr sceneInterface, RenderItemMap &renderItems, MSubSceneContainer &container, const Imath::M44d &matrix, bool isRoot )
{
	if( !sceneInterface )
	{
		return;
	}

	// TODO: switch this back to accumulating transformation while walking the tree if we find that this gets too expensive
	Imath::M44d accumulatedMatrix = worldTransform( sceneInterface.get(), m_time );
	MMatrix mayaMatrix = IECore::convert<MMatrix, Imath::M44d>( accumulatedMatrix );

	// Dispatch to all children.
	SceneInterface::NameList childNames;
	sceneInterface->childNames( childNames );

	for( const auto &childName : childNames )
	{
		visitSceneLocations( sceneInterface->child( childName ), renderItems, container, accumulatedMatrix );
	}

	// Now handle current location.

	std::string name;
	IECoreScene::SceneInterface::Path path;
	sceneInterface->path( path );
	IECoreScene::SceneInterface::pathToString( path, name );

	// \todo: this should get cleaned up and squirreled away into a more encapsulated way of building the MRenderItems
	if( isRoot && m_drawRootBounds )
	{
		std::string rootItemName = name + "_root";

		// \todo: There's gotta be a better way...
		Imath::Box3d boundingBoxD = sceneInterface->readBound( m_time );
		Imath::Box3f boundingBox( Imath::V3f( boundingBoxD.min.x, boundingBoxD.min.y, boundingBoxD.min.z ), Imath::V3f( boundingBoxD.max.x, boundingBoxD.max.y, boundingBoxD.max.z ) );
		const MBoundingBox mayaBoundingBox = IECore::convert<MBoundingBox>( boundingBox ); //\todo: can this be constructed from Box3d?

		int count = 0;
		for( const auto &instance : m_instances )
		{
			std::string instanceName = rootItemName + "_" + std::to_string( count++ );
			if( sceneIsAnimated( sceneInterface ) )
			{
				instanceName += "_" + std::to_string( m_time );
			}

			Imath::M44d instanceMatrix = accumulatedMatrix * instance.transformation;
			MMatrix instanceMayaMatrix = IECore::convert<MMatrix, Imath::M44d>( instanceMatrix );

			bool isEmpty;
			MString itemName = renderItemName( instanceName, RenderStyle::BoundingBox );
			MRenderItem *renderItem = getRenderItem( container, IECore::NullObject::defaultNullObject(), itemName, boundingBox, RenderStyle::BoundingBox, isEmpty );

			MShaderInstance *shader = getShader( m_sceneShape->thisMObject(), RenderStyle::BoundingBox, instance.componentMode, /* isComponentSelected = */ false );
			renderItem->setShader( shader );

			container.add( renderItem );

			if( isEmpty )
			{
				GeometryDataPtr geometryData = g_boundDataCache.get( BoundDataCacheGetterKey( boundingBox ) );
				setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->wireframeIndexBuffer), &mayaBoundingBox );
				m_renderItemNameToDagPath[renderItem->name().asChar()] = instance.path;
			}
		}
	}

	if( !sceneInterface->hasObject() )
	{
		return;
	}

	IECore::ConstObjectPtr object = sceneInterface->readObject( m_time );
	if( !objectCanBeRendered( object ) )
	{
		return;
	}

	// respect tags
	if( !m_drawTagsFilter.empty() && !sceneInterface->hasTag( m_drawTagsFilter ) )
	{
		return;
	}

	// respect visibility attribute
	if( sceneInterface->hasAttribute( "scene:visible" ) )
	{
		ConstBoolDataPtr vis = runTimeCast<const BoolData>( sceneInterface->readAttribute( "scene:visible", m_time ) );
		if( vis && !vis->readable() )
		{
			return;
		}
	}

	// We're going to render this object - compute its bounds only once and reuse them.
	// \todo: There's gotta be a better way...
	Imath::Box3d boundingBoxD = sceneInterface->readBound( m_time );
	Imath::Box3f bounds( Imath::V3f( boundingBoxD.min.x, boundingBoxD.min.y, boundingBoxD.min.z ), Imath::V3f( boundingBoxD.max.x, boundingBoxD.max.y, boundingBoxD.max.z ) );
	const MBoundingBox mayaBoundingBox = IECore::convert<MBoundingBox>( bounds ); //\todo: can this be constructed from Box3d?

	int componentIndex = m_sceneShape->selectionIndex( name );

	// Adding RenderItems as needed
	// ----------------------------
	for( RenderStyle style = RenderStyle::BoundingBox; style != RenderStyle::Last; ++style )
	{
		// Skipping unneeded MRenderItems.
		// NOTE: wireframe visibility is determined mostly by per-instance data.
		if( style !=RenderStyle::Wireframe )
		{
			int maskIndex = (int)style;

			if( style == RenderStyle::Textured )
			{
				maskIndex = (int)RenderStyle::Solid;
			}

			if( !m_styleMask.test( maskIndex ) )
			{
				continue;
			}
		}

		// When rendering instances, we gather all matrices and associate them
		// with a single MRenderItem. In the case of non-instanced rendering,
		// we make sure that we have an MRenderItem per instance. This has grown
		// around a problem we've encountered when rendering instances in Maya
		// and will need some cleanup once we've figured out what the issue is.

		// \note: We're passing a copy to Maya as we'd like to manage a pool of
		// MRenderItems ourselves. We can't hand off our own MRenderItems
		// directly because at that point Maya takes ownership and invalidates
		// them when it sees fit (for example when clearing the container Maya
		// gave us).

		// MRenderItem *storageCopy = nullptr;
		// if( m_instancedRendering ) // \todo: disabled and needs some work.
		// {
		// 	// \todo: per-instance shading
		// 	bool componentMode = m_instances[0].componentMode;
		// 	std::string pathKey = m_instances[0].path.fullPathName().asChar();
		// 	bool componentSelected = m_selectedComponents[pathKey].count( componentIndex ) > 0;
		// 	MShaderInstance *shader = getShader( m_sceneShape->thisMObject(), style, componentMode, componentMode ? componentSelected : m_instances[0].selected );

		// 	RenderItemWrapperCacheGetterKey key( this, name, object, style, container, shader, bounds, /* hashName = */ false );
		// 	RenderItemWrapperPtr renderItemWrapper = g_renderItemCache.get( key );
		// 	// \todo: mh.
		// 	if( !renderItemWrapper )
		// 	{
		// 		continue;
		// 	}

		// 	storageCopy = MRenderItem::Create( *(renderItemWrapper->get()) );

		// 	std::string copyName = storageCopy->name().asChar();

		// 	for( const auto &instance : m_instances )
		// 	{
		// 		if( style == RenderStyle::Wireframe )
		// 		{
		// 			if( !m_styleMask.test( (int)RenderStyle::Wireframe ) && !instance.selected && !instance.componentMode )
		// 			{
		// 				continue;
		// 			}
		// 		}

		// 		MMatrix instanceMatrix = IECore::convert<MMatrix, Imath::M44d>( accumulatedMatrix * instance.transformation );
		// 		appendMatrixToRenderItem( renderItems, copyName, storageCopy, instanceMatrix );
		// 		m_renderItemNameToDagPath[copyName] = instance.path;
		// 	}

		// 	storageCopy->enable( true );
		// 	storageCopy->setShader( shader ); // potentially updating an old shader for a cached render item
		// }
		// else
		// {
			int instanceIndex = 0;
			std::string instanceBaseName = name +  "_" + std::to_string( (int)style );

			// For animated geometry, we create an MRenderItem per frame.
			// Static geometry can be reused between frames and don't need an update.
			if( sceneIsAnimated( sceneInterface ) )
			{
				instanceBaseName += "_" + std::to_string( m_time );
			}

			for( const auto &instance : m_instances )
			{
				if( style == RenderStyle::Wireframe ) // wireframe visibility is mostly drive by instances and needs to be handles here.
				{
					if( !m_styleMask.test( (int)RenderStyle::Wireframe ) && !m_styleMask.test( (int)RenderStyle::Wireframe ) && !instance.selected && !instance.componentMode )
					{
						continue;
					}
				}

				std::string instanceName = instanceBaseName + std::to_string( instanceIndex++ );
				MString itemName = renderItemName( instanceName.c_str(), style );

				bool isEmpty;
				MRenderItem *renderItem = getRenderItem( container, object, itemName, bounds, style, isEmpty );

				ConstPrimitivePtr primitive = IECore::runTimeCast<const Primitive>( object );

				// Before setting geometry, a shader has to be assigned so that the data requirements are clear.
				std::string pathKey = instance.path.fullPathName().asChar();
				bool componentSelected = m_selectedComponents[pathKey].count( componentIndex ) > 0;
				MShaderInstance *shader = getShader( m_sceneShape->thisMObject(), style, instance.componentMode, instance.componentMode ? componentSelected : instance.selected );
				renderItem->setShader( shader );

				container.add( renderItem );

				// set the geometry on the render item if it's a new one.
				// \todo: combine the two caches (bounds and geo)
				if( isEmpty )
				{
					GeometryDataPtr geometryData;

					switch( style )
					{
					case RenderStyle::BoundingBox :
						geometryData = g_boundDataCache.get( BoundDataCacheGetterKey( bounds ) );
						setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->wireframeIndexBuffer), &mayaBoundingBox );
						break;
					case RenderStyle::Wireframe :
						geometryData = g_geometryDataCache.get( GeometryDataCacheGetterKey( primitive, renderItem->requiredVertexBuffers() ) );
						setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->wireframeIndexBuffer), &mayaBoundingBox );
						break;
					case RenderStyle::Solid :
					case RenderStyle::Textured :
						geometryData = g_geometryDataCache.get( GeometryDataCacheGetterKey( primitive, renderItem->requiredVertexBuffers() ) );
						setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->indexBuffer), &mayaBoundingBox );
					default :
						break;
					}

					MMatrix instanceMatrix = IECore::convert<MMatrix, Imath::M44d>( accumulatedMatrix * instance.transformation );
					renderItem->setMatrix( &instanceMatrix );
					renderItem->setCustomData( new RenderItemUserData( componentIndex ) );
					MDrawRegistry::registerComponentConverter( renderItem->name(), ComponentConverter::creator );
					m_renderItemNameToDagPath[renderItem->name().asChar()] = instance.path;
				}
			}
	  //}
	}
}

void SceneShapeSubSceneOverride::gatherInstanceInformation( Instances &instances, bool globalWireframeDrawing ) const
{
	MSelectionList selectionList;
	MGlobal::getActiveSelectionList( selectionList );

	MFnDagNode dagNode( m_sceneShape->thisMObject() );
	MDagPathArray dagPaths;
	dagNode.getAllPaths(dagPaths);
	int numInstances = dagPaths.length();

	instances.reserve( numInstances );
	for( int pathIndex = 0; pathIndex < numInstances; ++pathIndex )
	{
		MDagPath& path = dagPaths[pathIndex];
		Imath::M44d matrix = IECore::convert<Imath::M44d, MMatrix>( path.inclusiveMatrix() );
		bool pathSelected = isPathSelected( selectionList, path );
		bool componentMode = componentsSelectable( path );

		instances.emplace_back( matrix, pathSelected, componentMode, path );
	}
}

bool SceneShapeSubSceneOverride::renderAllBounds( unsigned int displayStyle ) const
{
	bool userOverride = false;

	MPlug drawAllBoundsPlug( m_sceneShape->thisMObject(), SceneShape::aDrawChildBounds );
	drawAllBoundsPlug.getValue( userOverride );

	// We draw bounding boxes if the user requests them explicitly or ...
	if( userOverride )
	{
		return true;
	}

	// ... if the display settings in Maya require them.
	if( ( displayStyle & MHWRender::MFrameContext::kBoundingBox ) > 0 )
	{
		return true;
	}

	return false;
}

bool SceneShapeSubSceneOverride::renderAllWireframes( unsigned int displayStyle ) const
{
	bool userOverride = false;

	MPlug drawGeometryPlug( m_sceneShape->thisMObject(), SceneShape::aDrawGeometry );
	drawGeometryPlug.getValue( userOverride );

	if( !userOverride )
	{
		return false;
	}

	if( ( displayStyle & MHWRender::MFrameContext::kWireFrame ) > 0 )
	{
		return true;
	}

	return false;
}

bool SceneShapeSubSceneOverride::renderAllShaded( unsigned int displayStyle ) const
{
	bool userOverride = false;

	MPlug drawGeometryPlug( m_sceneShape->thisMObject(), SceneShape::aDrawGeometry );
	drawGeometryPlug.getValue( userOverride );

	if( !userOverride )
	{
		return false;
	}

	if( ( displayStyle & ( MHWRender::MFrameContext::kGouraudShaded | MHWRender::MFrameContext::kTextured ) ) > 0 )
	{
		return true;
	}

	return false;
}

