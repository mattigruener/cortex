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

namespace IECoreMaya
{

enum class RenderStyle { BoundingBox, Wireframe, Solid, Last };

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

// todo: documentation
/* struct IECOREMAYA_API RenderItem */
/* { */
/* 	RenderItem( IECore::InternedString location, MHWRender::MRenderItem *renderItem, float time, RenderStyle style) */
/* 	: location( location ), renderItem( renderItem ), time(time), style(style) */
/* 	{ */
/* 	} */

/* 	const IECore::InternedString location; */
/* 	MHWRender::MRenderItem *renderItem; */
/* 	const float time; */
/* 	const RenderStyle style; */

/* 	// sort by location and by time within that location block */
/* 	// that allows tight loops over all styles. */
/* 	bool operator<( const RenderItem &rhs ) const */
/* 	{ */
/* 		// \todo: how fast is this if it's converting things into tuples first? */
/* 		return std::tie( location, time, style ) < std::tie( rhs.location, rhs.time, rhs.style ); */
/* 	} */
/* }; */

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

/* struct LookupByName */
/* { */
/* 	bool operator()( const RenderItem &lhs, const RenderItem &rhs ) const */
/* 	{ */
/* 		return lhs.location < rhs.location; */
/* 	} */
/* }; */

/* struct LookupByNameAndTime */
/* { */
/* 	bool operator()( const RenderItem &lhs, const RenderItem &rhs ) const */
/* 	{ */
/* 		return std::tie( lhs.location, lhs.time ) < std::tie( rhs.location, rhs.time ); */
/* 	} */
/* }; */

typedef std::bitset<3> StyleMask;

/* // todo: documentation */
/* class RenderItemRepository */
/* { */

/* public : */

/* 	MHWRender::MRenderItem *find( const std::string &name, float time, RenderStyle style ) const; */
/* 	MHWRender::MRenderItem *createRenderItem( const std::string &name, float time, RenderStyle style ); */

/* 	void setLocationEnabled( const std::string &name, bool enabled, float time, StyleMask styles ); */
/* 	void setSolidShader( MHWRender::MShaderInstance *shader ); */

/* private : */

/* 	typedef std::set<RenderItem> RenderItemContainer; */
/* 	RenderItemContainer m_storage; */

/* }; */

class IECOREMAYA_API SceneShapeSubSceneOverride : public MHWRender::MPxSubSceneOverride
{
	public :

		static MString& drawDbClassification();
		static MString& drawDbId();

		static MHWRender::MPxSubSceneOverride* Creator( const MObject& obj );

		~SceneShapeSubSceneOverride() override;

		bool requiresUpdate(const MHWRender::MSubSceneContainer& container, const MHWRender::MFrameContext& frameContext) const override;
		void update(MHWRender::MSubSceneContainer&  container, const MHWRender::MFrameContext& frameContext) override;

		MHWRender::DrawAPI supportedDrawAPIs() const override;

		MBoundingBox sceneBoundingBox() const
		{
			return m_sceneShape->boundingBox();
		}

	protected :

		SceneShapeSubSceneOverride( const MObject& obj );

	private :

		void visitSceneLocations( IECoreScene::ConstSceneInterfacePtr sceneInterface, MHWRender::MSubSceneContainer &container, const std::vector<Imath::M44d> &rootMatrices, const Imath::M44d &matrix, bool isRoot = false );

		MHWRender::MShaderInstance* getWireShader();
		MHWRender::MShaderInstance* getAssignedSurfaceShader();

		MObjectArray getConnectedInstancers() const;

		void appendMatrixToRenderItem( const std::string &name, MHWRender::MRenderItem *renderItem, MMatrix instanceMatrix );
		bool sceneIsAnimated() const;

		const MObject m_object;
		SceneShape *m_sceneShape;

		MPlug m_drawTagsFilterPlug;
		std::string m_drawTagsFilter;
		double m_time;
		// boost::optional<double> m_timeToDisable;
		MMatrix m_transformation;

		// RenderItemRepository m_repository;
		StyleMask m_styleMask;

		bool m_isSelected;
		bool m_drawRootBounds;
		MPlug m_shaderOutPlug;
		bool m_materialIsDirty;
		bool m_instancedRendering;
		IECoreScene::ConstSceneInterfacePtr m_sceneInterface;

		std::map<const std::string, std::pair<MHWRender::MRenderItem*, MMatrixArray> > m_renderItems;
};

} // namespace IECoreMaya

#endif // IE_COREMAYA_SCENESHAPESUBSCENEOVERRIDE_H
