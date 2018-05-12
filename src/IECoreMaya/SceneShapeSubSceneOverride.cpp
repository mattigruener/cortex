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
#include <maya/MGlobal.h>
#include <maya/MFnTransform.h>
#include <maya/MFnMatrixData.h>
#include <maya/MObjectArray.h>
#include <maya/MPlugArray.h>
#include <maya/MUserData.h>
#include <maya/MFnInstancer.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MItDependencyNodes.h>

#include "OpenEXR/ImathVec.h"

// todo: remove
#include "IECore/Timer.h"
#include "IECore/Exception.h"
#include "IECore/MessageHandler.h"

#include "IECore/LRUCache.h"
#include "IECoreMaya/SceneShapeSubSceneOverride.h"
#include "IECoreMaya/Convert.h"
#include "IECoreMaya/MayaTypeIds.h"
#include "IECoreScene/TriangulateOp.h"
#include "IECoreScene/MeshNormalsOp.h"
#include "IECoreScene/SampledSceneInterface.h"
#include "IECore/Object.h"

#include <algorithm>
#include <math.h>

using namespace IECoreScene;
using namespace IECoreMaya;
using namespace MHWRender;

namespace
{

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
	GeometryDataCacheGetterKey( const IECoreScene::MeshPrimitive *meshPrimitive, const MVertexBufferDescriptorList &descriptorList )
		:	m_meshPrimitive( meshPrimitive ), m_descriptorList( descriptorList )
	{
		m_meshPrimitive->hash( m_hash );
	}

	operator const IECore::MurmurHash & () const
	{
		return m_hash;
	}

	const IECoreScene::MeshPrimitive *m_meshPrimitive;
	const MVertexBufferDescriptorList &m_descriptorList;
	IECore::MurmurHash m_hash;
};

struct RenderItemWrapperCacheGetterKey
{
	RenderItemWrapperCacheGetterKey( SceneShapeSubSceneOverride *sceneShape,
									 const std::string &name,
									 const IECoreScene::MeshPrimitive *meshPrimitive,
									 const RenderStyle style,
									 MSubSceneContainer &container,
									 MShaderInstance *shader,
									 bool hashName = true)
		:	m_meshPrimitive( meshPrimitive ), m_name( name ), m_style( style ), m_sceneShape( sceneShape ), m_container( container ), m_shader( shader )
	{
		m_meshPrimitive->hash( m_hash );
		m_hash.append( (int)m_style );

		if( hashName )
		{
			m_hash.append( name );
		}
	}

	operator const IECore::MurmurHash & () const
	{
		return m_hash;
	}

	// most of these members aren't needed to compute the hash, but to then
	// construct an MRenderItem
	const IECoreScene::MeshPrimitive *m_meshPrimitive;
	const std::string m_name;
	const RenderStyle m_style;
	SceneShapeSubSceneOverride *m_sceneShape;
	MSubSceneContainer &m_container;
	MShaderInstance *m_shader;

	IECore::MurmurHash m_hash;
};

void ensureFaceVaryingData( IECore::ConstIntVectorDataPtr &i, IECore::ConstV3fVectorDataPtr &p, IECore::ConstV3fVectorDataPtr &n, std::vector<int> &additionalIndices, bool interpolationIsLinear )
{
	if( !( p && n ) )
	{
		return;
	}

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
	IECore::ScopedTimer foo( "SceneShapeSubSceneOverride::fillBoundData" );
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

void fillGeometryData( ConstMeshPrimitivePtr meshPrimitive, const MVertexBufferDescriptorList &descriptorList, GeometryDataPtr geometryData )
{
	IECore::ScopedTimer foo( "SceneShapeSubSceneOverride::fillGeometryData" );

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

		// default descriptor which might not be correct \todo
		MVertexBufferDescriptor uvDescriptor( "", MGeometry::kTexture, MGeometry::kFloat, 2 );
		// MVertexBufferDescriptor uvDescriptor( "uvCoord", MGeometry::kTexture, "mayauvcoordsemantic", MGeometry::kFloat, 2 );

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

// \todo: should this compute an estimate of the cost in terms of GPU memory so we can limit how much we're using?
// Maya lets us query the available GPU memory via MRenderer
GeometryDataPtr geometryGetter( const GeometryDataCacheGetterKey &key, size_t &cost )
{
	cost = 1;

	GeometryDataPtr geometryData( new GeometryData() );
	fillGeometryData( key.m_meshPrimitive, key.m_descriptorList, geometryData );
	return geometryData;
}

// Cache for BoundData
using BoundDataCache = IECore::LRUCache<IECore::MurmurHash, GeometryDataPtr, IECore::LRUCachePolicy::Parallel, BoundDataCacheGetterKey> ;
BoundDataCache g_boundDataCache(boundGetter, 100);

// Cache for GeometryData
using GeometryDataCache = IECore::LRUCache<IECore::MurmurHash, GeometryDataPtr, IECore::LRUCachePolicy::Parallel, GeometryDataCacheGetterKey>;
GeometryDataCache g_geometryDataCache(geometryGetter, 100);

RenderItemWrapperPtr renderItemGetter( const RenderItemWrapperCacheGetterKey &key, size_t &cost )
{
	cost = 1;

	// The key that we get contains all data that's needed to generate the MRenderItem
	// These members are available:
	//    - m_meshPrimitive
	//    - m_name
	//    - m_style
	//    - m_sceneShape
	//    - m_container
	//    - m_shader

	Imath::Box3f boundingBox = key.m_meshPrimitive->bound();
	// const MBoundingBox bbox = IECore::convert<MBoundingBox>( boundingBox );
	MBoundingBox bbox = key.m_sceneShape->sceneBoundingBox();

	// \note: data can only be added to an MRenderItem if it has been added to
	// the container and has a shader assigned that determines the geometry
	// requirements.
	MRenderItem *renderItem;
	GeometryDataPtr geometryData;
	if( key.m_style == RenderStyle::BoundingBox )
	{
		MString name( ( key.m_name + "_bb" ).c_str() );
		renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kLines);

		renderItem->setDrawMode( MGeometry::kAll );
		renderItem->castsShadows(false);
		renderItem->receivesShadows(false);
		renderItem->setExcludedFromPostEffects(true);

		renderItem->setShader( key.m_shader );
		key.m_container.add( renderItem );

		geometryData = g_boundDataCache.get( BoundDataCacheGetterKey( boundingBox ) );
		key.m_sceneShape->setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->wireframeIndexBuffer), &bbox );
	}
	else if( key.m_style == RenderStyle::Wireframe )
	{
		MString name( ( key.m_name + "_wf" ).c_str() );
		renderItem = MRenderItem::Create( name, MRenderItem::DecorationItem, MGeometry::kLines );

		renderItem->setDrawMode( MGeometry::kAll );
		renderItem->castsShadows( false );
		renderItem->receivesShadows( false );
		renderItem->setExcludedFromPostEffects( true );
		renderItem->depthPriority( MRenderItem::sActiveWireDepthPriority );

		renderItem->setShader( key.m_shader );
		key.m_container.add( renderItem );

		geometryData = g_geometryDataCache.get( GeometryDataCacheGetterKey( key.m_meshPrimitive, renderItem->requiredVertexBuffers() ) );
		key.m_sceneShape->setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->wireframeIndexBuffer), &bbox );
	}
	else if( key.m_style == RenderStyle::Solid || key.m_style == RenderStyle::Textured )
	{
		MString name( (key.m_name + "_s").c_str() );
		renderItem = MRenderItem::Create( name, MRenderItem::MaterialSceneItem, MGeometry::kTriangles );

		renderItem->setDrawMode( (key.m_style == RenderStyle::Solid) ? MGeometry::kShaded : MGeometry::kTextured ); // TODO: what about textured?
		renderItem->castsShadows( true );
		renderItem->receivesShadows( true );
		renderItem->setExcludedFromPostEffects( true );

		renderItem->setShader( key.m_shader );
		key.m_container.add( renderItem );

		geometryData = g_geometryDataCache.get( GeometryDataCacheGetterKey( key.m_meshPrimitive, renderItem->requiredVertexBuffers() ) );
		key.m_sceneShape->setGeometryForRenderItem( *renderItem, *(geometryData->vertexBufferArray), *(geometryData->indexBuffer), &bbox );
	}
	else
	{
		throw( IECore::Exception( boost::str( boost::format( "Could not create MRenderItem %1%." ) % key.m_name ) ) );
	}

	RenderItemWrapper *wrapper = new RenderItemWrapper( MRenderItem::Create( *renderItem ), geometryData );
	RenderItemWrapperPtr renderItemWrapper( wrapper );
	return renderItemWrapper;
}

using RenderItemCache = IECore::LRUCache<IECore::MurmurHash, RenderItemWrapperPtr, IECore::LRUCachePolicy::Parallel, RenderItemWrapperCacheGetterKey>;
RenderItemCache g_renderItemCache(renderItemGetter, 10000);

// \todo: Fix up what happens when shader is deleted.
// class ShadedItemUserData : public MUserData
// {
// public:
// 	ShadedItemUserData( SceneShapeSubSceneOverride* ov )
// 		: MUserData( false ), subSceneOverride( ov ) {}
// 	~ShadedItemUserData() { subSceneOverride = NULL; }

// 	SceneShapeSubSceneOverride* subSceneOverride;
// };

// For the given node return the out plug on the surface shader that is assigned
MPlug getShaderOutPlug( MObject sceneShapeNode )
{
	MPlug result = MPlug();

	MObjectArray sets, components;
	MFnDagNode node( sceneShapeNode );
	MDagPathArray instances;
	node.getAllPaths(instances);

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
	// IECore::ScopedTimer foo( "GeometryDataCacheGetterKey::requiresUpdate" );

	// TIME UPDATED?
	if( m_sceneShape->time() != m_time && sceneIsAnimated() )
	{
		return true;
	}

	if( m_sceneInterface != m_sceneShape->getSceneInterface() )
	{
		return true;
	}

	// SELECTION UPDATED?
	// \todo: this should take instances into account.
	MSelectionList list;
	MFnDagNode nodeFn;
	nodeFn.setObject( m_sceneShape->thisMObject() );

	MDagPath p;
	MStatus status;
	MGlobal::getActiveSelectionList( list, status );
	MObject parent = nodeFn.parent( 0 );
	nodeFn.setObject( parent );
	nodeFn.getPath( p );
	if( list.hasItem( p ) != m_isSelected )
	{
		return true;
	}

	// DRAW GEOMETRY SETTINGS UPDATED?
	const unsigned int displayStyle = frameContext.getDisplayStyle();

	MPlug drawGeometryPlug( m_sceneShape->thisMObject(), SceneShape::aDrawGeometry );
	bool drawGeometry;
	drawGeometryPlug.getValue( drawGeometry );

	bool tmpDrawAsWireframe = (displayStyle & MHWRender::MFrameContext::kWireFrame) > 0;
	if( ((tmpDrawAsWireframe && drawGeometry) || (m_isSelected && drawGeometry)) != m_styleMask.test( int( RenderStyle::Wireframe ) ) )
	{
		return true;
	}

	bool tmpDrawAsSolid = (displayStyle & (MHWRender::MFrameContext::kGouraudShaded | MHWRender::MFrameContext::kTextured)) > 0;
	if( (tmpDrawAsSolid && drawGeometry) != m_styleMask.test( int( RenderStyle::Solid ) ) )
	{
		return true;
	}

	// CHILD BOUNDING BOC SETTING UPDATED?
	MPlug drawAllBoundsPlug( m_sceneShape->thisMObject(), SceneShape::aDrawChildBounds );
	bool drawAllBounds = false;
	drawAllBoundsPlug.getValue( drawAllBounds );
	bool tmpDrawAsBounds = (displayStyle & (MHWRender::MFrameContext::kBoundingBox) ) > 0;
	if( ( tmpDrawAsBounds || drawAllBounds ) != m_styleMask.test( int( RenderStyle::BoundingBox ) ) )
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
	m_drawTagsFilterPlug.getValue( tmpTagsFilter );
	if( tmpTagsFilter.asChar() != m_drawTagsFilter )
	{
		return true;
	}

	// TRANSFORMATION UPDATED?
	// \todo: currently does not take instances into account.
	MFnDagNode dagNode( m_sceneShape->thisMObject() );
	MDagPathArray dagPaths;
	dagNode.getAllPaths(dagPaths);
	MDagPath& path = dagPaths[0];
	MMatrix mm = path.inclusiveMatrix();
	if( mm != m_transformation )
	{
		return true;
	}

	// SHADER UPDATED?
	if( getShaderOutPlug( m_sceneShape->thisMObject() ) != m_shaderOutPlug )
	{
		return true;
	}

	return false;
}

void SceneShapeSubSceneOverride::update( MSubSceneContainer& container, const MFrameContext& frameContext )
{
	IECore::ScopedTimer foo( "SceneShapeSubSceneOverride::update, frame: " + std::to_string( m_sceneShape->time() * 24 ) );

	// We'll set internal state based on settings in maya and then perform
	// updates by walking the tree.

	m_time = m_sceneShape->time();
	m_sceneInterface = m_sceneShape->getSceneInterface();

	// STYLE
	const unsigned int displayStyle = frameContext.getDisplayStyle();

	bool renderWireFrame( (displayStyle & MHWRender::MFrameContext::kWireFrame) > 0);
	renderWireFrame ? m_styleMask.set( (int)RenderStyle::Wireframe ) : m_styleMask.reset( (int)RenderStyle::Wireframe );

	bool renderSolid( ( displayStyle & (MHWRender::MFrameContext::kGouraudShaded | MHWRender::MFrameContext::kTextured)) > 0 );
	renderSolid ? m_styleMask.set( (int)RenderStyle::Solid ) : m_styleMask.reset( (int)RenderStyle::Solid );

	bool renderBound( ( displayStyle & MHWRender::MFrameContext::kBoundingBox ) > 0 );
	renderBound ? m_styleMask.set( (int)RenderStyle::BoundingBox ) : m_styleMask.reset( (int)RenderStyle::BoundingBox );

	// SELECTION - potentially overrides wireframe visibility
	MSelectionList list;
	MFnDagNode thisNodeFn;
	MFnDagNode parentNodeFn;
	MDagPath nodePath;
	MDagPath parentPath;

	thisNodeFn.setObject( m_sceneShape->thisMObject() );
	thisNodeFn.getPath( nodePath );
	MGlobal::getActiveSelectionList( list );
	MObject parent = thisNodeFn.parent( 0 );
	parentNodeFn.setObject( parent );
	parentNodeFn.getPath( parentPath );

	if( list.hasItem( parentPath ) != m_isSelected )
	{
		m_isSelected = list.hasItem( parentPath );
	}

	if( m_isSelected )
	{
		m_styleMask.set( (int)RenderStyle::Wireframe );
	}

	// SETTINGS ON NODE - trump all other settings
	MPlug drawGeometryPlug( m_sceneShape->thisMObject(), SceneShape::aDrawGeometry );
	bool drawGeometry;
	drawGeometryPlug.getValue( drawGeometry );
	if( !drawGeometry )
	{
		m_styleMask.reset( (int)RenderStyle::Wireframe );
		m_styleMask.reset( (int)RenderStyle::Solid );
	}

	MPlug drawChildBoundsPlug( m_sceneShape->thisMObject(), SceneShape::aDrawChildBounds );
	bool drawChildBounds = false;
	drawChildBoundsPlug.getValue( drawChildBounds );
	if( drawChildBounds )
	{
		m_styleMask.set( (int)RenderStyle::BoundingBox );
	}

	MPlug drawRootBoundsPlug( m_sceneShape->thisMObject(), SceneShape::aDrawRootBound );
	drawRootBoundsPlug.getValue( m_drawRootBounds );

	// TAGS
	MString tmpTagsFilter;
	m_drawTagsFilterPlug.getValue( tmpTagsFilter );
	if( tmpTagsFilter.asChar() != m_drawTagsFilter )
	{
		m_drawTagsFilter = tmpTagsFilter.asChar();
	}

	// TRANSFORMATION - sort out existence of instances at the same time.
	std::vector<Imath::M44d> matrices;
	MFnDagNode dagNode( m_sceneShape->thisMObject() );
	MDagPathArray dagPaths;
	dagNode.getAllPaths(dagPaths);

	int numInstances = dagPaths.length();
	matrices.reserve( numInstances );
	for( int i = 0; i < numInstances; ++i )
	{
		MDagPath& path = dagPaths[i];
		m_transformation = path.inclusiveMatrix(); // \todo: needs cache of all matrices for update trigger
		matrices.push_back( IECore::convert<Imath::M44d, MMatrix>( m_transformation ) );
	}
	// \todo: remove once caching all matrices
	MDagPath& path = dagPaths[0]; // 0 is used for the cache atm
	m_transformation = path.inclusiveMatrix(); // \todo: needs cache of all matrices for update trigger

	// SHADING
	MPlug currentShaderOutPlug = getShaderOutPlug( m_sceneShape->thisMObject() );
	if( currentShaderOutPlug != m_shaderOutPlug )
	{
		m_shaderOutPlug = currentShaderOutPlug;
		m_materialIsDirty = true;
	}

	// Perform update
	{
		IECore::ScopedTimer visitLocationsTimer( "visitingLocations" );

		m_renderItems.clear();

		printf( "state before updating %i %i %i \n", (int)m_styleMask.test( 0 ), (int)m_styleMask.test( 1 ), (int)m_styleMask.test( 2 ) );
		// Gather geometry to render into m_renderItems

		container.clear(); // Clear out previous render items so that they don't interfere with new ones.

		visitSceneLocations( m_sceneInterface, container, matrices, Imath::M44d(), /* isRoot = */ true );

		container.clear(); // MRenderItems might have been added temporarily so that Maya lets us fill them

		// Actually add all gathered MRenderItems
		printf( "Will add %i render items.\n", (int)m_renderItems.size() );
		for( auto namedRenderItem : m_renderItems )
		{
			auto itemAndMatrices = namedRenderItem.second;
			MRenderItem *renderItem = itemAndMatrices.first;

			if( !renderItem )
			{
				continue;
			}

			// \todo
			// renderItem->enable( true );
			container.add( renderItem );

			// note: the following is true for all entries if
			// !m_instancedRendering because we're creating a separate
			// render item per instance
			//printf( "instances: %i\n", (int)itemAndMatrices.second.length() );
			if( itemAndMatrices.second.length() == 1 )
			{
				// renderItem->setWantSubSceneConsolidation( true );
				renderItem->setMatrix( &itemAndMatrices.second[0] );
			}
			else
			{
				renderItem->setWantSubSceneConsolidation( false );
				setInstanceTransformArray( *renderItem, itemAndMatrices.second );
				// \todo: we should make sure that we can select individual instances.
			}
		}
	}

	// Get timing output - a little expensive in itself
	IECore::ScopedTimer::printAllChannels();
	IECore::ScopedTimer::resetAllChannels();
}

DrawAPI SceneShapeSubSceneOverride::supportedDrawAPIs() const
{
	return kAllDevices;
}

SceneShapeSubSceneOverride::SceneShapeSubSceneOverride( const MObject& obj )
	: MPxSubSceneOverride(obj), m_object( obj ), m_drawTagsFilter( "" ), m_time( -1 ), m_isSelected( false ), m_drawRootBounds( false ), m_shaderOutPlug(), m_materialIsDirty(true), m_instancedRendering( false )
{
	MStatus status;
	MFnDependencyNode node( obj, &status );

	if( status )
	{
		m_sceneShape = dynamic_cast<SceneShape*>( node.userNode() );
		m_drawTagsFilterPlug = MPlug( m_sceneShape->thisMObject(), SceneShape::aDrawTagsFilter );
	}
}

void SceneShapeSubSceneOverride::visitSceneLocations( ConstSceneInterfacePtr sceneInterface, MSubSceneContainer &container, const std::vector<Imath::M44d> &rootMatrices, const Imath::M44d &matrix, bool isRoot )
{
	if( !sceneInterface )
	{
		return;
	}

	Imath::M44d accumulatedMatrix = matrix * sceneInterface->readTransformAsMatrix( m_time );
	MMatrix mayaMatrix = IECore::convert<MMatrix, Imath::M44d>( accumulatedMatrix );

	// Dispatch to all children.
	SceneInterface::NameList childNames;
	sceneInterface->childNames( childNames );

	for( const auto &childName : childNames )
	{
		visitSceneLocations( sceneInterface->child( childName ), container, rootMatrices, accumulatedMatrix );
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

		const MBoundingBox bbox = IECore::convert<MBoundingBox>( boundingBox );
		GeometryDataPtr boundData = g_boundDataCache.get( BoundDataCacheGetterKey( boundingBox ) );

		if( m_instancedRendering )
		{
			MRenderItem *renderItem = MRenderItem::Create( MString( rootItemName.c_str() ), MRenderItem::DecorationItem, MGeometry::kLines );
			container.add( renderItem );
			renderItem->setShader( getWireShader() );
			renderItem->enable( true );
			setGeometryForRenderItem( *renderItem, *(boundData->vertexBufferArray), *(boundData->wireframeIndexBuffer), &bbox );

			// The MRenderItem constructed above gets invalidated as soon as we clear out the container.
			MRenderItem *storageCopy = MRenderItem::Create( *renderItem );
			std::string name = storageCopy->name().asChar();

			for( const auto &rootMatrix : rootMatrices )
			{
				Imath::M44d instanceMatrix = accumulatedMatrix * rootMatrix;
				MMatrix instanceMayaMatrix = IECore::convert<MMatrix, Imath::M44d>( instanceMatrix );

				appendMatrixToRenderItem( name, storageCopy, instanceMayaMatrix );
			}
		}
		else
		{
			int count = 0;
			for( const auto &rootMatrix : rootMatrices )
			{
				Imath::M44d instanceMatrix = accumulatedMatrix * rootMatrix;
				MMatrix instanceMayaMatrix = IECore::convert<MMatrix, Imath::M44d>( instanceMatrix );

				std::string instanceName = rootItemName + "_" + std::to_string( count++ );
				MRenderItem *renderItem = MRenderItem::Create( MString( instanceName.c_str() ), MRenderItem::DecorationItem, MGeometry::kLines );

				container.add( renderItem );
				renderItem->setShader( getWireShader() );
				renderItem->enable( true );
				renderItem->setMatrix( &instanceMayaMatrix );

				setGeometryForRenderItem( *renderItem, *(boundData->vertexBufferArray), *(boundData->wireframeIndexBuffer), &bbox );

				MRenderItem *storageCopy = MRenderItem::Create( *renderItem );
				appendMatrixToRenderItem( instanceName, storageCopy, instanceMayaMatrix );
			}
		}
	}

	if( !sceneInterface->hasObject() )
	{
		return;
	}

	IECoreScene::ConstMeshPrimitivePtr meshPrimitive;
	IECore::ConstObjectPtr object;
	{
		{
			IECore::ScopedTimer fooN( "readObject", "readObject" );
			object = sceneInterface->readObject( m_time );
		}
		{
			IECore::ScopedTimer fooN( "castToMesh", "castToMesh" );
			meshPrimitive = IECore::runTimeCast<const IECoreScene::MeshPrimitive>( object );
		}
		if( !meshPrimitive )
		{
			return;
		}
	}

	// respect tags
	if( !m_drawTagsFilter.empty() && !sceneInterface->hasTag( m_drawTagsFilter ) )
	{
		return;
	}

	// Adding RenderItems as needed
	// ----------------------------
	for( RenderStyle style = RenderStyle::BoundingBox; style != RenderStyle::Last; ++style )
	{
		IECore::ScopedTimer fooI( "fillingRenderItem", "fillingRenderItem" );

		// based on style, sort out shading and visibility, and potential skipping to improve performance
		MShaderInstance *shader = nullptr;
		bool visibility = true;
		if( style == RenderStyle::BoundingBox )
		{
			// we always compute bounding boxes - no skipping required.
			shader = getWireShader();
			visibility = m_styleMask.test( (int)RenderStyle::BoundingBox );
		}
		else
		{
			// if only bounding boxes are requested, don't create anything else
			if( !m_styleMask.test( (int)RenderStyle::Wireframe ) && !m_styleMask.test( (int)RenderStyle::Solid ) )
			{
				continue;
			}

			if( style == RenderStyle::Solid || style == RenderStyle::Textured )
			{
				ShaderPair shaders = getAssignedSurfaceShader();
				shader = (style == RenderStyle::Solid) ? shaders.first : shaders.second;
			}
			else
			{
				shader = getWireShader();
			}

			if( style == RenderStyle::Textured )
			{
				visibility = m_styleMask.test( (int)RenderStyle::Solid );
			}
			else
			{
				visibility = m_styleMask.test( (int)style );
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

		MRenderItem *storageCopy = nullptr;
		if( m_instancedRendering )
		{
			RenderItemWrapperCacheGetterKey key( this, name, meshPrimitive.get(), style, container, shader, /* hashName = */ false );
			RenderItemWrapperPtr renderItemWrapper = g_renderItemCache.get( key );

			storageCopy = MRenderItem::Create( *(renderItemWrapper->renderItem()) );
			std::string name = storageCopy->name().asChar();

			for( const auto &rootMatrix : rootMatrices )
			{
				MMatrix instanceMatrix = IECore::convert<MMatrix, Imath::M44d>( accumulatedMatrix * rootMatrix );
				appendMatrixToRenderItem( name, storageCopy, instanceMatrix );
			}
		}
		else
		{
			int count = 0;
			std::string instanceBaseName = name + "_" + std::to_string( (int)style ) + "_";
			for( const auto &rootMatrix : rootMatrices )
			{
				std::string instanceName = instanceBaseName + std::to_string( count++ );
				MMatrix instanceMatrix = IECore::convert<MMatrix, Imath::M44d>( accumulatedMatrix * rootMatrix );

				RenderItemWrapperCacheGetterKey key( this, instanceName, meshPrimitive.get(), style, container, shader, /* hashName = */ true );
				RenderItemWrapperPtr renderItemWrapper = g_renderItemCache.get( key );

				storageCopy = MRenderItem::Create( *(renderItemWrapper->renderItem()) );
				appendMatrixToRenderItem( instanceName, storageCopy, instanceMatrix );  // note that the name is unique
			}
		}

		storageCopy->enable( visibility );
	}
}

MShaderInstance* SceneShapeSubSceneOverride::getWireShader()
{
	static MHWRender::MShaderInstance *s_shader = nullptr;

	if( !s_shader )
	{
		MRenderer* renderer = MRenderer::theRenderer();
		if( !renderer )
		{
			return nullptr;
		}

		const MShaderManager* shaderManager = renderer->getShaderManager();
		if( !shaderManager )
		{
			return nullptr;
		}

		s_shader = shaderManager->getStockShader( MShaderManager::k3dSolidShader );
		const float wireColor[] = {0.0f, 1.0f, 0.0f, 1.0f};
		s_shader->setParameter("solidColor", wireColor );
	}

	return s_shader;
}


SceneShapeSubSceneOverride::ShaderPair SceneShapeSubSceneOverride::getAssignedSurfaceShader()
{
	static ShaderPair shaders = std::make_pair( nullptr, nullptr );

	if( !shaders.first || !shaders.second || m_materialIsDirty )
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

		MObjectArray sets, components;
		MFnDagNode node( m_sceneShape->thisMObject() );
		MDagPathArray instances;
		node.getAllPaths(instances);

		if( !node.getConnectedSetsAndMembers( 0, sets, components, true ) )
		{
			return shaders;
		}

		for (unsigned int i = 0; i < sets.length(); i++)
		{
			MStatus status;
			MFnDependencyNode fnSet( sets[i], &status );

			if( !status )
			{
				continue;
			}

			MPlug shaderPlug = fnSet.findPlug( "surfaceShader" );
			if( shaderPlug.isNull() )
			{
				continue;
			}

			MPlugArray connectedPlugs;
			shaderPlug.connectedTo( connectedPlugs, true, false );

			if( connectedPlugs.length() >= 1 )
			{
				MShaderInstance *gray = shaderManager->getShaderFromNode( connectedPlugs[0].node(), instances[0], 0, 0, 0, 0, /*nonTextured = */ true );
				MShaderInstance *textured = shaderManager->getShaderFromNode( connectedPlugs[0].node(), instances[0], 0, 0, 0, 0, /*nonTextured = */ false );
				shaders = std::make_pair( gray, textured );
				break;
			}
		}
	}

	return shaders;
}

// Convenience function that should eventually help with also rendering all instances generated by instancers.
MObjectArray SceneShapeSubSceneOverride::getConnectedInstancers() const
{
	MObjectArray result;

	MStatus status;
	MObject current;

	MDagPathArray paths;
	MMatrixArray matrices;
	MIntArray pathStartIndices;
	MIntArray pathIndices;

	MObject sceneShape = m_sceneShape->thisMObject();

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
				if( connectedNode.object() == sceneShape || connectedNode.hasChild( sceneShape ) )
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

void SceneShapeSubSceneOverride::appendMatrixToRenderItem( const std::string &name, MRenderItem *renderItem, MMatrix instanceMatrix )
{
	auto it = m_renderItems.find( name );
	if( it == m_renderItems.end() )
	{
		MMatrixArray array = MMatrixArray();
		it = m_renderItems.emplace( name, std::make_pair( renderItem, array ) ).first;
	}

	it->second.second.append( instanceMatrix );
}

bool SceneShapeSubSceneOverride::sceneIsAnimated() const
{
	ConstSampledSceneInterfacePtr scene = IECore::runTimeCast< const SampledSceneInterface >( m_sceneInterface );
	return ( !scene || scene->numBoundSamples() > 1 );
}

RenderItemWrapper::RenderItemWrapper( MRenderItem *renderItem, GeometryDataPtr geometryData )
	: m_renderItem( renderItem ), m_geometryData( geometryData )
{
}

RenderItemWrapper::~RenderItemWrapper()
{
	MRenderItem::Destroy( m_renderItem );
}
