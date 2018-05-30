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
#ifndef IE_COREMAYA_SCENESHAPESUBSCENEOVERRIDE_H
#define IE_COREMAYA_SCENESHAPESUBSCENEOVERRIDE_H

#include <bitset>
#include <map>

#include "IECore/MurmurHash.h"
#include "IECore/InternedString.h"
#include "IECore/RefCounted.h"

#include "IECoreScene/SceneInterface.h"
#include "IECoreScene/MeshPrimitive.h"

#include "IECoreMaya/Export.h"
#include "IECoreMaya/SceneShape.h"

#include "maya/MPxSubSceneOverride.h"
#include "maya/MSelectionContext.h"

namespace IECoreMaya
{

enum class RenderStyle { BoundingBox, Wireframe, Solid, Textured, Last };

// Smart pointers for Maya's buffers for automatic clean-up
using IndexBufferPtr = std::shared_ptr<MHWRender::MIndexBuffer> ;
using VertexBufferArrayPtr = std::shared_ptr<MHWRender::MVertexBufferArray>;

// struct BoundData
// {
// 	BoundData()
// 		: vertexBufferArray( new MHWRender::MVertexBufferArray )
// 	{
// 	}

// 	~BoundData()
// 	{
// 		for( unsigned int i = 0; i < vertexBufferArray->count(); ++i )
// 		{
// 			delete vertexBufferArray->getBuffer( i );
// 		}
// 		vertexBufferArray->clear();
// 	}

// 	std::shared_ptr<MHWRender::MVertexBufferArray> vertexBufferArray;
// 	IndexBufferPtr wireframeIndexBuffer;
// };

// using BoundDataPtr = std::shared_ptr<BoundData>;

struct GeometryData
{
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

	std::shared_ptr<MHWRender::MVertexBufferArray> vertexBufferArray;
	IndexBufferPtr indexBuffer;
	IndexBufferPtr wireframeIndexBuffer;
};

using GeometryDataPtr = std::shared_ptr<GeometryData>;

// \todo: use an alias with std::shared_ptr instead of custom stuff
class IECOREMAYA_API RenderItemWrapper : public IECore::RefCounted
{
 public:
	RenderItemWrapper( MHWRender::MRenderItem *renderItem, GeometryDataPtr geometryData );
	~RenderItemWrapper();

	MHWRender::MRenderItem* renderItem() { return m_renderItem; }
	GeometryDataPtr geometryData() { return m_geometryData; }

 private:
	MHWRender::MRenderItem *m_renderItem;
	GeometryDataPtr m_geometryData;

};

IE_CORE_DECLAREPTR( RenderItemWrapper );

typedef std::bitset<3> StyleMask;

class Instance
{
 public:
 Instance( Imath::M44d transformation, bool selected, bool componentMode, MDagPath path )
	 : transformation( transformation ), selected( selected ), componentMode( componentMode ), path( path )
	{}

	bool operator==( const Instance &rhs ) const
	{
		return transformation == rhs.transformation && selected == rhs.selected && path == rhs.path && componentMode == rhs.componentMode;
	}

	Imath::M44d transformation;
	bool selected;
	bool componentMode;
	MDagPath path;
};

typedef std::vector<Instance> Instances;

class IECOREMAYA_API SceneShapeSubSceneOverride : public MHWRender::MPxSubSceneOverride
{
	public :

		static MString& drawDbClassification();
		static MString& drawDbId();

		static MHWRender::MPxSubSceneOverride* Creator( const MObject& obj );

		~SceneShapeSubSceneOverride() override;

		bool requiresUpdate(const MHWRender::MSubSceneContainer& container, const MHWRender::MFrameContext& frameContext) const override;
		void update(MHWRender::MSubSceneContainer&  container, const MHWRender::MFrameContext& frameContext) override;
		bool getInstancedSelectionPath( const MHWRender::MRenderItem &renderItem, const MHWRender::MIntersection &intersection, MDagPath &dagPath ) const override;
		// \todo: remove?
		void updateSelectionGranularity( const MDagPath &path, MHWRender::MSelectionContext &selectionContext ) override;

		MHWRender::DrawAPI supportedDrawAPIs() const override;

	protected :

		SceneShapeSubSceneOverride( const MObject& obj );

	private :

		// traverse the scene and create MRenderItems as necessary
		void visitSceneLocations( IECoreScene::ConstSceneInterfacePtr sceneInterface, MHWRender::MSubSceneContainer &container, const Imath::M44d &matrix, bool isRoot = false );

		// shaders usually come in two flavours: non-textured/textured or deselected/selected
		typedef std::pair<MHWRender::MShaderInstance*, MHWRender::MShaderInstance*> ShaderPair;

		ShaderPair getWireShaders() const;
		ShaderPair getComponentWireShaders() const;
		ShaderPair getAssignedSurfaceShaders();
		MHWRender::MShaderInstance *getShader( RenderStyle style, const Instance &instance, const std::string &location );

		MObjectArray getConnectedInstancers() const;

		void appendMatrixToRenderItem( const std::string &name, MHWRender::MRenderItem *renderItem, MMatrix instanceMatrix );
		bool sceneIsAnimated() const;
		void gatherInstanceInformation( Instances &instances, bool globalWireframeDrawing ) const;

		// retrieve global display settings (can be overridden by instances)
		bool renderAllBounds( unsigned int displayStyle ) const;
		bool renderAllWireframes( unsigned int displayStyle ) const;
		bool renderAllShaded( unsigned int displayStyle ) const;

		void selectedComponentIndices( std::set<int> &selectedComponents ) const;
		bool componentsSelectable( const MDagPath &path ) const;

		Imath::M44d worldTransform( const IECoreScene::SceneInterface *scene, double time ) const;

		SceneShape *m_sceneShape;

		std::string m_drawTagsFilter;
		double m_time;

		StyleMask m_styleMask; // \todo: now that things are simpler, consider replacing this with three bools.
		Instances m_instances;

		bool m_drawRootBounds;
		MPlug m_shaderOutPlug;
		bool m_materialIsDirty;
		bool m_instancedRendering;
		IECoreScene::ConstSceneInterfacePtr m_sceneInterface;
		bool m_componentsSelectable;
		bool m_wireframeInstanceOverride;

		std::map<const std::string, std::pair<MHWRender::MRenderItem*, MMatrixArray> > m_renderItems;
		std::map<const std::string, MDagPath> m_itemToPathMap;
		std::set<int> m_selectedComponents;

};

} // namespace IECoreMaya

#endif // IE_COREMAYA_SCENESHAPESUBSCENEOVERRIDE_H
