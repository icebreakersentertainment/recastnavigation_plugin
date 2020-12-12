#include <cstring>

#include <boost/format.hpp>

//#include <Detour/DetourCommon.h>
//#include <Detour/DetourMath.h>
//#include <Detour/DetourAlloc.h>
//#include <Detour/DetourAssert.h>
//
//#include <DebugUtils/RecastDebugDraw.h>
//#include <DebugUtils/DetourDebugDraw.h>

#include <DetourCommon.h>
#include <DetourMath.h>
#include <DetourAlloc.h>
#include <DetourAssert.h>

#include <RecastDebugDraw.h>
#include <DetourDebugDraw.h>

#include "detail/DebugSerializer.hpp"

#include "RecastNavigation.hpp"

#include "exceptions/Exception.hpp"

//if (!dtCreateNavMeshData(&navigationMesh.params, &navData, &navDataSize))
//{
//	throw std::runtime_error("Unable to build Detour navmesh.");
//}

//bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize);
//bool* dtCreateNavMeshData _fp =(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize)&dtCreateNavMeshData;

// hack to make sure this function does not get discarded in gcc
bool (*dtCreateNavMeshData_fp)(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize) = &dtCreateNavMeshData;

std::ostream& operator<<(std::ostream& os, const rcChunkyTriMesh& chunkyMesh)
{
	return os << "(rcChunkyTriMesh: nnodes " << chunkyMesh.nnodes << " ntris " << chunkyMesh.ntris << " maxTrisPerChunk " << chunkyMesh.maxTrisPerChunk << ")";
}

namespace ice_engine
{
namespace pathfinding
{
namespace recastnavigation
{

// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
static const int EXPECTED_LAYERS_PER_TILE = 4;
static const int MAX_LAYERS = 32;

namespace
{

static std::string getErrorMessage(const dtStatus status)
{
    if (status & DT_WRONG_MAGIC)   return "Input data is not recognized.";
    if (status & DT_WRONG_VERSION) return "Input data is in wrong version.";
    if (status & DT_OUT_OF_MEMORY) return "Operation ran out of memory.";
    if (status & DT_INVALID_PARAM) return "An input parameter was invalid.";
    if (status & DT_BUFFER_TOO_SMALL) return "Result buffer for the query was too small to store all results.";
    if (status & DT_OUT_OF_NODES) return "Query ran out of nodes during search.";
    if (status & DT_PARTIAL_RESULT) return "Query did not reach the end location, returning best guess.";
    if (status & DT_ALREADY_OCCUPIED) return "A tile has already been assigned to the given x,y coordinate";
}

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = gridWidth * gridHeight;
	return headerSize + gridSize*4;
}

//struct RasterizationContext
//{
//	RasterizationContext() :
//		solid(0),
//		triareas(0),
//		lset(0),
//		chf(0),
//		ntiles(0)
//	{
//		memset(tiles, 0, sizeof(TileCacheData)*MAX_LAYERS);
//	}
//
//	~RasterizationContext()
//	{
//		rcFreeHeightField(solid);
//		delete [] triareas;
//		rcFreeHeightfieldLayerSet(lset);
//		rcFreeCompactHeightfield(chf);
//		for (int i = 0; i < MAX_LAYERS; ++i)
//		{
//			dtFree(tiles[i].data);
//			tiles[i].data = 0;
//		}
//	}
//
//	rcHeightfield* solid;
//	unsigned char* triareas;
//	rcHeightfieldLayerSet* lset;
//	rcCompactHeightfield* chf;
//	TileCacheData tiles[MAX_LAYERS];
//	int ntiles;
//};

std::vector<TileCacheData> rasterizeTileLayers(
	const int tx,
	const int ty,
	const rcChunkyTriMesh* chunkyMesh,
	const RecastNavigationPolygonMesh& polygonMesh,
	const int maxTiles,
	logger::ILogger* logger
)
{
	rcContext ctx = rcContext();

//	unsigned char* triareas;
//	TileCacheData tiles[MAX_LAYERS];

	const float* verts = &polygonMesh.vertices[0].x;
	const int nverts = static_cast<int>(polygonMesh.vertices.size());

	// Tile bounds.
	const float tcs = polygonMesh.config.tileSize * polygonMesh.config.cs;

	rcConfig tcfg;
	memcpy(&tcfg, &polygonMesh.config, sizeof(tcfg));

	tcfg.bmin[0] = polygonMesh.config.bmin[0] + tx*tcs;
	tcfg.bmin[1] = polygonMesh.config.bmin[1];
	tcfg.bmin[2] = polygonMesh.config.bmin[2] + ty*tcs;
	tcfg.bmax[0] = polygonMesh.config.bmin[0] + (tx+1)*tcs;
	tcfg.bmax[1] = polygonMesh.config.bmax[1];
	tcfg.bmax[2] = polygonMesh.config.bmin[2] + (ty+1)*tcs;
	tcfg.bmin[0] -= tcfg.borderSize*tcfg.cs;
	tcfg.bmin[2] -= tcfg.borderSize*tcfg.cs;
	tcfg.bmax[0] += tcfg.borderSize*tcfg.cs;
	tcfg.bmax[2] += tcfg.borderSize*tcfg.cs;

	// Rasterize input
	auto heightField = std::unique_ptr<rcHeightfield, decltype(&rcFreeHeightField)>(rcAllocHeightfield(), rcFreeHeightField);
	if (!heightField) throw std::bad_alloc(); //("Unable to create height field - out of memory.");

	if (!rcCreateHeightfield(&ctx, *heightField, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch)) throw Exception("Unable to create height field.");

	auto triangleAreas = std::vector<unsigned char>(chunkyMesh->maxTrisPerChunk, 0);

	float tbmin[2], tbmax[2];
	tbmin[0] = tcfg.bmin[0];
	tbmin[1] = tcfg.bmin[2];
	tbmax[0] = tcfg.bmax[0];
	tbmax[1] = tcfg.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid) return std::vector<TileCacheData>(); // empty

	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* tris = &chunkyMesh->tris[node.i*3];
		const int ntris = node.n;

		std::fill(triangleAreas.begin(), triangleAreas.end(), 0);
		rcMarkWalkableTriangles(&ctx, tcfg.walkableSlopeAngle, verts, nverts, tris, ntris, &triangleAreas[0]);

		if (!rcRasterizeTriangles(&ctx, verts, nverts, tris, &triangleAreas[0], ntris, *heightField, tcfg.walkableClimb)) return std::vector<TileCacheData>();
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(&ctx, tcfg.walkableClimb, *heightField);
	rcFilterLedgeSpans(&ctx, tcfg.walkableHeight, tcfg.walkableClimb, *heightField);
	rcFilterWalkableLowHeightSpans(&ctx, tcfg.walkableHeight, *heightField);

	auto compactHeightfield = std::unique_ptr<rcCompactHeightfield, decltype(&rcFreeCompactHeightfield)>(rcAllocCompactHeightfield(), rcFreeCompactHeightfield);
	if (!rcBuildCompactHeightfield(&ctx, tcfg.walkableHeight, tcfg.walkableClimb, *heightField, *compactHeightfield))
	{
		throw Exception("Unable to build compact data.");
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(&ctx, tcfg.walkableRadius, *compactHeightfield))
	{
		throw Exception("Unable to erode.");
	}

	// (Optional) Mark areas.
//	const ConvexVolume* vols = m_geom->getConvexVolumes();
//	for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
//	{
//		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts,
//							 vols[i].hmin, vols[i].hmax,
//							 (unsigned char)vols[i].area, *rc.chf);
//	}

	auto heightfieldLayerSet = std::unique_ptr<rcHeightfieldLayerSet, decltype(&rcFreeHeightfieldLayerSet)>(rcAllocHeightfieldLayerSet(), rcFreeHeightfieldLayerSet);
	if (!rcBuildHeightfieldLayers(&ctx, *compactHeightfield, tcfg.borderSize, tcfg.walkableHeight, *heightfieldLayerSet))
	{
		throw Exception("Could not build heighfield layers.");
	}

	FastLZCompressor comp;
	std::vector<TileCacheData> tileCacheData;

	auto numLayers = rcMin(heightfieldLayerSet->nlayers, MAX_LAYERS);
	for (int i = 0; i < numLayers; ++i)
	{
		tileCacheData.push_back({});
		TileCacheData* tile = &tileCacheData.back();
		const rcHeightfieldLayer* layer = &heightfieldLayerSet->layers[i];

		// Store header
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;

		// Tile layer location in the navmesh.
		header.tx = tx;
		header.ty = ty;
		header.tlayer = i;
		dtVcopy(header.bmin, layer->bmin);
		dtVcopy(header.bmax, layer->bmax);

		// Tile info.
		header.width = (unsigned char)layer->width;
		header.height = (unsigned char)layer->height;
		header.minx = (unsigned char)layer->minx;
		header.maxx = (unsigned char)layer->maxx;
		header.miny = (unsigned char)layer->miny;
		header.maxy = (unsigned char)layer->maxy;
		header.hmin = (unsigned short)layer->hmin;
		header.hmax = (unsigned short)layer->hmax;

		const dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons, &tile->data, &tile->dataSize);
		if (dtStatusFailed(status))
		{
			throw Exception("Could not build tile cache layer: " + getErrorMessage(status));
//			return 0;
		}
	}

	// Transfer ownsership of tile data from build context to the caller.
//	int n = 0;
//	for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
//	{
//		tiles[n++] = rc.tiles[i];
//		rc.tiles[i].data = 0;
//		rc.tiles[i].dataSize = 0;
//	}
//
//	return n;
	return tileCacheData;









//	rcMarkWalkableTriangles(&ctx, polygonMesh.config.walkableSlopeAngle, &terrain->vertices()[0].x, terrain->vertices().size()*3, reinterpret_cast<const int*>(&terrain->indices()[0]), terrain->indices().size()/3, &triangleAreas[0]);
//
//	if (!rcRasterizeTriangles(&ctx, &terrain->vertices()[0].x, terrain->vertices().size()*3, reinterpret_cast<const int*>(&terrain->indices()[0]), &triangleAreas[0], terrain->indices().size()/3, *polygonMesh.heightField, polygonMesh.config.walkableClimb))
//	{
//		throw std::runtime_error("Unable to rasterize triangles.");
//	}

	// Filter walkables surfaces
//	rcFilterLowHangingWalkableObstacles(&ctx, polygonMesh.config.walkableClimb, *polygonMesh.heightField);
//	rcFilterLedgeSpans(&ctx, polygonMesh.config.walkableHeight, polygonMesh.config.walkableClimb, *polygonMesh.heightField);
//	rcFilterWalkableLowHeightSpans(&ctx, polygonMesh.config.walkableHeight, *polygonMesh.heightField);

//	if (polygonMesh.heightField->width == 0 || polygonMesh.heightField->height == 0) throw std::runtime_error("Height field has height or width of zero.");

	// Partition walkable surface to simple regions
//	polygonMesh.compactHeightfield = std::unique_ptr<rcCompactHeightfield, decltype(&rcFreeCompactHeightfield)>(rcAllocCompactHeightfield(), rcFreeCompactHeightfield);
//	if (!polygonMesh.compactHeightfield)
//	{
//		throw std::bad_alloc(); //("Unable to create compact height field - out of memory.");
//	}
//
//	if (!rcBuildCompactHeightfield(&ctx, polygonMesh.config.walkableHeight, polygonMesh.config.walkableClimb, *polygonMesh.heightField, *polygonMesh.compactHeightfield))
//	{
//		throw std::runtime_error("Unable to build compact data.");
//	}

	// Erode the walkable area by agent radius.
//	if (!rcErodeWalkableArea(&ctx, polygonMesh.config.walkableRadius, *polygonMesh.compactHeightfield))
//	{
//		throw std::runtime_error("Unable to erode.");
//	}

	/*
	// Build regions based on partition type
	RecastNavigationPartitionType partitionType = RECAST_NAVIGATION_PARTITION_WATERSHED;
	if (partitionType == RECAST_NAVIGATION_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(&ctx, *polygonMesh.compactHeightfield))
		{
			throw std::runtime_error("Unable to build distance field.");
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(&ctx, *polygonMesh.compactHeightfield, 0, polygonMesh.config.minRegionArea, polygonMesh.config.mergeRegionArea))
		{
			throw std::runtime_error("Unable to build watershed regions.");
		}
	}
	else if (partitionType == RECAST_NAVIGATION_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(&ctx, *polygonMesh.compactHeightfield, 0, polygonMesh.config.minRegionArea, polygonMesh.config.mergeRegionArea))
		{
			throw std::runtime_error("Unable to build monotone regions.");
		}
	}
	else // RECAST_NAVIGATION_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(&ctx, *polygonMesh.compactHeightfield, 0, polygonMesh.config.minRegionArea))
		{
			throw std::runtime_error("Unable to build layer regions.");
		}
	}

	if (polygonMesh.compactHeightfield->width == 0 || polygonMesh.compactHeightfield->height == 0) throw std::runtime_error("Compact height field has height or width of zero.");
	if (polygonMesh.compactHeightfield->spanCount == 0) throw std::runtime_error("Compact height field set has no spans.");

	// Trace and simplify region contours.
	polygonMesh.contourSet = std::unique_ptr<rcContourSet, decltype(&rcFreeContourSet)>(rcAllocContourSet(), rcFreeContourSet);
	if (!polygonMesh.contourSet)
	{
		throw std::bad_alloc(); //("Unable to create contours - out of memory.");
	}
	if (!rcBuildContours(&ctx, *polygonMesh.compactHeightfield, polygonMesh.config.maxSimplificationError, polygonMesh.config.maxEdgeLen, *polygonMesh.contourSet))
	{
		throw std::runtime_error("Unable to create contours.");
	}
	if (polygonMesh.contourSet->width == 0 || polygonMesh.contourSet->height == 0) throw std::runtime_error("Contour set has height or width of zero.");
	if (polygonMesh.contourSet->nconts == 0) throw std::runtime_error("Contour set has no contours.");

	// Build polygons mesh from contours.
	polygonMesh.polyMesh = std::unique_ptr<rcPolyMesh, decltype(&rcFreePolyMesh)>(rcAllocPolyMesh(), rcFreePolyMesh);
	if (!polygonMesh.polyMesh)
	{
		throw std::bad_alloc(); //("Unable to triangulate contours - out of memory.");
	}
	if (!rcBuildPolyMesh(&ctx, *polygonMesh.contourSet, polygonMesh.config.maxVertsPerPoly, *polygonMesh.polyMesh))
	{
		throw std::runtime_error("Unable to triangulate contours.");
	}

	// Create detail mesh which allows to access approximate height on each polygon.
	polygonMesh.polyMeshDetail = std::unique_ptr<rcPolyMeshDetail, decltype(&rcFreePolyMeshDetail)>(rcAllocPolyMeshDetail(), rcFreePolyMeshDetail);
	if (!polygonMesh.polyMeshDetail)
	{
		throw std::bad_alloc(); //("Unable to build detail mesh - out of memory.");
	}

	if (!rcBuildPolyMeshDetail(&ctx, *polygonMesh.polyMesh, *polygonMesh.compactHeightfield, polygonMesh.config.detailSampleDist, polygonMesh.config.detailSampleMaxError, *polygonMesh.polyMeshDetail))
	{
		throw std::runtime_error("Unable to build detail mesh.");
	}
	*/
}


MovementRequestState moveRequestStateToMovementRequestState(const unsigned char moveRequestState)
{
	switch (moveRequestState)
	{
		case DT_CROWDAGENT_TARGET_NONE:
			return MovementRequestState::NONE;

		case DT_CROWDAGENT_TARGET_FAILED:
			return MovementRequestState::FAILED;

		case DT_CROWDAGENT_TARGET_VALID:
			return MovementRequestState::VALID;

		case DT_CROWDAGENT_TARGET_REQUESTING:
			return MovementRequestState::REQUESTING;

		case DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE:
			return MovementRequestState::WAITING_FOR_QUEUE;

		case DT_CROWDAGENT_TARGET_WAITING_FOR_PATH:
			return MovementRequestState::WAITING_FOR_PATH;

		case DT_CROWDAGENT_TARGET_VELOCITY:
			return MovementRequestState::VELOCITY;

		default:
			assert(false && "We should never get here");
			break;
	}

	return MovementRequestState::NONE;
}

AgentState agentStateToAgentState(const unsigned char agentState)
{
	switch (agentState)
	{
		case DT_CROWDAGENT_STATE_INVALID:
			return AgentState::INVALID;

		case DT_CROWDAGENT_STATE_WALKING:
			return AgentState::WALKING;

		case DT_CROWDAGENT_STATE_OFFMESH:
			return AgentState::OFFMESH;

		default:
			assert(false && "We should never get here");
			break;
	}
}

void drawObstacles(DebugRenderer& debugRenderer, const dtTileCache& tc)
{
	// Draw obstacles
	for (int i = 0; i < tc.getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = tc.getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY) continue;
		float bmin[3], bmax[3];
		tc.getObstacleBounds(ob, bmin,bmax);

		unsigned int col = 0;
		if (ob->state == DT_OBSTACLE_PROCESSING)
			col = duRGBA(255,255,0,128);
		else if (ob->state == DT_OBSTACLE_PROCESSED)
			col = duRGBA(255,192,0,192);
		else if (ob->state == DT_OBSTACLE_REMOVING)
			col = duRGBA(220,0,0,128);

		duDebugDrawCylinder(&debugRenderer, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], col);
		duDebugDrawCylinderWire(&debugRenderer, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duDarkenCol(col), 2);
	}
}

void debugRender(
	const RecastNavigationCrowd& crowd,
	const RecastNavigationNavigationMesh& navigationMesh,
	const RecastNavigationPolygonMesh& polygonMesh,
	DebugRenderer& debugRenderer
)
{
	if (navigationMesh.navMesh)
	{
		//duDebugDrawNavMeshBVTree(&debugRenderer, *navigationMesh.navMesh);
		duDebugDrawNavMeshPolysWithFlags(&debugRenderer, *navigationMesh.navMesh, RECAST_NAVIGATION_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
		duDebugDrawNavMeshPolysWithFlags(&debugRenderer, *navigationMesh.navMesh, RECAST_NAVIGATION_POLYFLAGS_ALL, duRGBA(64,64,64,128));
	}
	if (navigationMesh.navMeshQuery)
	{
		//duDebugDrawNavMeshNodes(&debugRenderer, *navigationMesh.navMeshQuery);
	}
	if (polygonMesh.polyMesh)
	{
		//duDebugDrawPolyMesh(&debugRenderer, *navigationMesh.polyMesh);
	}
	if (polygonMesh.tileCache)
	{
		drawObstacles(debugRenderer, *polygonMesh.tileCache);
	}

	for (const auto& agent : crowd.agents)
	{
		const dtCrowdAgent* ag = crowd.crowd->getAgent(agent.index);
		if (!ag->active) continue;

		const float radius = ag->params.radius;
		const float* pos = ag->npos;

		/*
		{
			unsigned int col = duRGBA(0,0,0,32);
			if (m_agentDebug.idx == i)
			{
				col = duRGBA(255,0,0,128);
			}

			duDebugDrawCircle(&debugRenderer, pos[0], pos[1], pos[2], radius, col, 2.0f);
		}
		*/

		// Draw corners
		{
			if (ag->ncorners)
			{
				debugRenderer.begin(DU_DRAW_LINES, 2.0f);
				for (int j = 0; j < ag->ncorners; ++j)
				{
					const float* va = j == 0 ? pos : &ag->cornerVerts[(j-1)*3];
					const float* vb = &ag->cornerVerts[j*3];
					debugRenderer.vertex(va[0],va[1]+radius,va[2], duRGBA(128,0,0,192));
					debugRenderer.vertex(vb[0],vb[1]+radius,vb[2], duRGBA(128,0,0,192));
				}
				if (ag->ncorners && ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
				{
					const float* v = &ag->cornerVerts[(ag->ncorners-1)*3];
					debugRenderer.vertex(v[0],v[1],v[2], duRGBA(192,0,0,192));
					debugRenderer.vertex(v[0],v[1]+radius*2,v[2], duRGBA(192,0,0,192));
				}

				debugRenderer.end();
			}
		}

		// Draw path optimizations
		{
			const float* optStart = ag->corridor.getPos();
			const float* target = &ag->cornerVerts[dtMin(1,ag->ncorners-1)*3];

			debugRenderer.begin(DU_DRAW_LINES, 2.0f);
			debugRenderer.vertex(optStart[0], optStart[1] + 0.3f, optStart[2], duRGBA(0,128,0,192));
			debugRenderer.vertex(target[0], target[1] + 0.3f, target[2], duRGBA(0,128,0,192));
			debugRenderer.end();
		}

		// Agent cylinders.
		const float height = ag->params.height;

		{
			unsigned int col = duRGBA(220,220,220,128);
			if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING || ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
			{
				col = duLerpCol(col, duRGBA(128,0,255,128), 32);
			}
			else if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
			{
				col = duLerpCol(col, duRGBA(128,0,255,128), 128);
			}
			else if (ag->targetState == DT_CROWDAGENT_TARGET_FAILED)
			{
				col = duRGBA(255,32,16,128);
			}
			else if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			{
				col = duLerpCol(col, duRGBA(64,255,0,128), 128);
			}

			duDebugDrawCylinder(&debugRenderer, pos[0]-radius, pos[1]+radius*0.1f, pos[2]-radius,pos[0]+radius, pos[1]+height, pos[2]+radius, col);
		}

		// Velocity stuff
		const float* vel = ag->vel;
		const float* dvel = ag->dvel;

		{
			unsigned int col = duRGBA(220,220,220,192);
			if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING || ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
			{
				col = duLerpCol(col, duRGBA(128,0,255,192), 32);
			}
			else if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
			{
				col = duLerpCol(col, duRGBA(128,0,255,192), 128);
			}
			else if (ag->targetState == DT_CROWDAGENT_TARGET_FAILED)
			{
				col = duRGBA(255,32,16,192);
			}
			else if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			{
				col = duLerpCol(col, duRGBA(64,255,0,192), 128);
			}

			duDebugDrawCircle(&debugRenderer, pos[0], pos[1]+height, pos[2], radius, col, 2.0f);

			duDebugDrawArrow(&debugRenderer, pos[0],pos[1]+height,pos[2],
				 pos[0]+dvel[0],pos[1]+height+dvel[1],pos[2]+dvel[2],
				 //0.0f, 0.4f, duRGBA(0,192,255,192), (m_agentDebug.idx == i) ? 2.0f : 1.0f);
				 0.0f, 0.4f, duRGBA(0,192,255,192), 1.0f);

			duDebugDrawArrow(&debugRenderer, pos[0],pos[1]+height,pos[2],
							 pos[0]+vel[0],pos[1]+height+vel[1],pos[2]+vel[2],
							 0.0f, 0.4f, duRGBA(0,0,0,160), 2.0f);
		}
	}
}

}

RecastNavigation::RecastNavigation(utilities::Properties* properties, fs::IFileSystem* fileSystem, logger::ILogger* logger)
{
	properties_ = properties;
	fileSystem_ = fileSystem;
	logger_ = logger;
}

void RecastNavigation::tick(const PathfindingSceneHandle& pathfindingSceneHandle, const float32 delta)
{
	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];

//	if (pathfindingScene.debugRendering && debugRenderer_)
//	{
//		for (const auto& crowd : pathfindingScene.crowds)
//		{
//			const auto& navigationMesh = navigationMeshes_[crowd.navigationMeshHandle];
//			const auto& polygonMesh = polygonMeshes_[navigationMesh.polygonMeshHandle];
//
//			debugRender(crowd, navigationMesh, polygonMesh, *debugRenderer_);
//		}
//	}
	
//	for (auto& polygonMesh : polygonMeshes_)
//	{
//		for (auto& navigationMesh : navigationMeshes_)
//		{
//			if (polygonMesh.tileCache && navigationMesh.navMesh) polygonMesh.tileCache->update(delta, navigationMesh.navMesh.get());
//			break;
//		}
//	}

	for (auto& crowd : pathfindingScene.crowds)
	{
		const auto& navigationMesh = navigationMeshes_[crowd.navigationMeshHandle];
		const auto& polygonMesh = polygonMeshes_[navigationMesh.polygonMeshHandle];

		if (polygonMesh.tileCache && navigationMesh.navMesh) polygonMesh.tileCache->update(delta, navigationMesh.navMesh.get());

		for (auto& agent : crowd.agents)
		{
			const dtCrowdAgent* ag = crowd.crowd->getAgent(agent.index);
			if (!ag->active) continue;

			if (ag->targetState != agent.moveRequestState)
			{
				agent.moveRequestState = ag->targetState;
				if (agent.movementRequestStateChangeListener) agent.movementRequestStateChangeListener->update(moveRequestStateToMovementRequestState(agent.moveRequestState));
			}
		}

		crowd.crowd->update(delta, nullptr);

		for (const auto& agent : crowd.agents)
		{
			const dtCrowdAgent* ag = crowd.crowd->getAgent(agent.index);
			if (!ag->active) continue;

			if (agent.agentMotionChangeListener) agent.agentMotionChangeListener->update(glm::vec3(ag->npos[0], ag->npos[1], ag->npos[2]));
		}

		for (auto& agent : crowd.agents)
		{
			const dtCrowdAgent* ag = crowd.crowd->getAgent(agent.index);
			if (!ag->active) continue;

			if (ag->state != agent.state)
			{
				agent.state = ag->state;
				if (agent.agentStateChangeListener) agent.agentStateChangeListener->update(agentStateToAgentState(agent.state));
			}
		}
	}
}

void RecastNavigation::renderDebug(const PathfindingSceneHandle& pathfindingSceneHandle)
{
    const auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];

    if (pathfindingScene.debugRendering && debugRenderer_)
    {
        for (const auto& crowd : pathfindingScene.crowds)
        {
            const auto& navigationMesh = navigationMeshes_[crowd.navigationMeshHandle];
            const auto& polygonMesh = polygonMeshes_[navigationMesh.polygonMeshHandle];

            debugRender(crowd, navigationMesh, polygonMesh, *debugRenderer_);
        }
    }
}

PathfindingSceneHandle RecastNavigation::createPathfindingScene()
{
    LOG_DEBUG(logger_, "Creating pathfinding scene");

	return pathfindingScenes_.create();
}

void RecastNavigation::destroyPathfindingScene(const PathfindingSceneHandle& pathfindingSceneHandle)
{
    LOG_DEBUG(logger_, "Destroying pathfinding scene %s", pathfindingSceneHandle);

	pathfindingScenes_.destroy(pathfindingSceneHandle);
}

void RecastNavigation::setPathfindingDebugRenderer(IPathfindingDebugRenderer* pathfindingDebugRenderer)
{
	debugRenderer_ = std::make_unique<DebugRenderer>(pathfindingDebugRenderer);
}

void RecastNavigation::setDebugRendering(const PathfindingSceneHandle& pathfindingSceneHandle, const bool enabled)
{
	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	pathfindingScene.debugRendering = enabled;
}

PolygonMeshHandle RecastNavigation::createPolygonMesh(const ITerrain* terrain, const PolygonMeshConfig& polygonMeshConfig)
{
    LOG_DEBUG(logger_, "Creating polygon mesh with polygon mesh config %s", polygonMeshConfig);

	const auto polygonMeshHandle = polygonMeshes_.create();
	auto& polygonMesh = polygonMeshes_[polygonMeshHandle];

	rcContext ctx = rcContext();

	polygonMesh.vertices = terrain->vertices();
	polygonMesh.indices = terrain->indices();

	float bmin[3];
	float bmax[3];

	rcCalcBounds(&polygonMesh.vertices[0].x, static_cast<int>(polygonMesh.vertices.size()), bmin, bmax);

	// recast configuration
	polygonMesh.config.cs = polygonMeshConfig.cellSize;
	polygonMesh.config.ch = polygonMeshConfig.cellHeight;
	polygonMesh.config.walkableSlopeAngle = polygonMeshConfig.walkableSlopeAngle;
	polygonMesh.config.walkableHeight = polygonMeshConfig.walkableHeight;
	polygonMesh.config.walkableClimb = polygonMeshConfig.walkableClimb;
	polygonMesh.config.walkableRadius = polygonMeshConfig.walkableRadius;
	polygonMesh.config.maxEdgeLen = polygonMeshConfig.maxEdgeLength;
	polygonMesh.config.maxSimplificationError = polygonMeshConfig.maxSimplificationError;
	polygonMesh.config.minRegionArea = polygonMeshConfig.minRegionArea;
	polygonMesh.config.mergeRegionArea = polygonMeshConfig.mergeRegionArea;
	polygonMesh.config.maxVertsPerPoly = polygonMeshConfig.maxVertsPerPoly;
	polygonMesh.config.tileSize = polygonMeshConfig.tileSize;

	polygonMesh.config.borderSize = polygonMesh.config.walkableRadius + 3; // Reserve enough padding.
	polygonMesh.config.width = polygonMesh.config.tileSize + polygonMesh.config.borderSize*2;
	polygonMesh.config.height = polygonMesh.config.tileSize + polygonMesh.config.borderSize*2;

	polygonMesh.config.detailSampleDist = polygonMeshConfig.detailSampleDist;
	polygonMesh.config.detailSampleMaxError = polygonMeshConfig.detailSampleMaxError;

	rcVcopy(polygonMesh.config.bmin, bmin);
	rcVcopy(polygonMesh.config.bmax, bmax);

	rcCalcGridSize(polygonMesh.config.bmin, polygonMesh.config.bmax, polygonMesh.config.cs, &polygonMesh.gridWidth, &polygonMesh.gradeHeight);

	const int tileWidth = (polygonMesh.gridWidth + polygonMeshConfig.tileSize-1) / polygonMeshConfig.tileSize;
	const int tileHeight = (polygonMesh.gradeHeight + polygonMeshConfig.tileSize-1) / polygonMeshConfig.tileSize;

	LOG_TRACE(logger_, "Tile width and height %d, %d", tileWidth, tileHeight);

	// tile cache parameters
	rcVcopy(polygonMesh.tileCacheParams.orig, bmin);
	polygonMesh.tileCacheParams.cs = polygonMesh.config.cs;
	polygonMesh.tileCacheParams.ch = polygonMesh.config.ch;
	polygonMesh.tileCacheParams.width = polygonMeshConfig.tileSize;
	polygonMesh.tileCacheParams.height = polygonMeshConfig.tileSize;
	polygonMesh.tileCacheParams.walkableHeight = static_cast<float>(polygonMeshConfig.walkableHeight);
	polygonMesh.tileCacheParams.walkableRadius = static_cast<float>(polygonMeshConfig.walkableRadius);
	polygonMesh.tileCacheParams.walkableClimb = static_cast<float>(polygonMeshConfig.walkableClimb);
	polygonMesh.tileCacheParams.maxSimplificationError = polygonMeshConfig.maxSimplificationError;
	polygonMesh.tileCacheParams.maxTiles = tileWidth*tileHeight*EXPECTED_LAYERS_PER_TILE;
	polygonMesh.tileCacheParams.maxObstacles = polygonMeshConfig.maxObstacles;

	// create tile cache
	LOG_TRACE(logger_, "Creating tile cache");
	polygonMesh.tileCache = std::unique_ptr<dtTileCache, decltype(&dtFreeTileCache)>(dtAllocTileCache(), dtFreeTileCache);
	if (!polygonMesh.tileCache) throw std::bad_alloc();

	dtStatus status = polygonMesh.tileCache->init(&polygonMesh.tileCacheParams, &polygonMesh.linearAllocator, &polygonMesh.fastrLzCompressor, &polygonMesh.meshProcess);
	if (dtStatusFailed(status)) throw Exception("Could not init tile cache: " + getErrorMessage(status));

 	const int trianglesPerChunk = 256;

 	LOG_TRACE(logger_, "creating chunky triangle mesh with %d triangles per chunk", trianglesPerChunk);

	if (!rcCreateChunkyTriMesh(&polygonMesh.vertices[0].x, reinterpret_cast<const int*>(&polygonMesh.indices[0]), static_cast<int>(polygonMesh.indices.size())/3, trianglesPerChunk, &polygonMesh.chunkyMesh))
	{
		throw Exception("Failed to build chunky mesh.");
	}

	LOG_TRACE(logger_, "created chunky triangle mesh %1%", polygonMesh.chunkyMesh);
	LOG_TRACE(logger_, "total number of vertices %d", polygonMesh.vertices.size());

	for (int y = 0; y < tileHeight; ++y)
	{
		for (int x = 0; x < tileWidth; ++x)
		{
//			TileCacheData tiles[MAX_LAYERS];
//			memset(tiles, 0, sizeof(tiles));
			auto  tiles = rasterizeTileLayers(x, y, &polygonMesh.chunkyMesh, polygonMesh, MAX_LAYERS, logger_);

			for (auto& t : tiles)
			{
				polygonMesh.tiles.push_back(std::move(t));

				const auto& tile = polygonMesh.tiles.back();

				status = polygonMesh.tileCache->addTile(tile.data, tile.dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile.data);
					polygonMesh.tiles.pop_back();
//					tile.data = 0;
//					continue;
					 throw Exception("Could not add tile to tile cache: " + getErrorMessage(status));
				}

				polygonMesh.cacheLayerCount++;
				polygonMesh.cacheCompressedSize += tile.dataSize;
				polygonMesh.cacheRawSize += calcLayerBufferSize(polygonMesh.tileCacheParams.width, polygonMesh.tileCacheParams.height);
			}
//			for (int i = 0; i < ntiles; ++i)
//			{
//				TileCacheData* tile = &tiles[i];
//				status = polygonMesh.tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
//				if (dtStatusFailed(status))
//				{
//					dtFree(tile->data);
//					tile->data = 0;
////					continue;
//					 throw std::runtime_error("Could not add tile to tile cache.");
//				}
//
//				polygonMesh.cacheLayerCount++;
//				polygonMesh.cacheCompressedSize += tile->dataSize;
//				polygonMesh.cacheRawSize += calcLayerBufferSize(polygonMesh.tileCacheParams.width, polygonMesh.tileCacheParams.height);
//			}
		}
	}

	LOG_TRACE(logger_, "cache layer count %d", polygonMesh.cacheLayerCount);
	LOG_TRACE(logger_, "cache compressed size %d", polygonMesh.cacheCompressedSize);
	LOG_TRACE(logger_, "cache raw size %d", polygonMesh.cacheRawSize);

	// build initial meshes
//	for (int y = 0; y < tileHeight; ++y)
//	{
//		for (int x = 0; x < tileWidth; ++x)
//		{
//			polygonMesh.tileCache->buildNavMeshTilesAt(x,y, m_navMesh);
//		}
//	}

	return polygonMeshHandle;
}

void RecastNavigation::destroy(const PolygonMeshHandle& polygonMeshHandle)
{
    LOG_DEBUG(logger_, "Destroying polygon mesh %s", polygonMeshHandle);

	polygonMeshes_.destroy(polygonMeshHandle);
}

ObstacleHandle RecastNavigation::createObstacle(const PolygonMeshHandle& polygonMeshHandle, const glm::vec3& position, const float32 radius, const float32 height)
{
    LOG_DEBUG(logger_, "Creating obstacle with polygon mesh %s, position %s, radius %s, and height %s", polygonMeshHandle, position, radius, height);

	auto& polygonMesh = polygonMeshes_[polygonMeshHandle];

	auto obstacleHandle = polygonMesh.obstacles.create();
	auto& obstacle = polygonMesh.obstacles[obstacleHandle];

    const dtStatus status = polygonMesh.tileCache->addObstacle(&position.x, radius, height, &obstacle.obstacleReference);

	if (dtStatusFailed(status))
	{
        polygonMesh.obstacles.destroy(obstacleHandle);

		throw Exception("Unable to add obstacle: " + getErrorMessage(status));
	}

	return obstacleHandle;
}

void RecastNavigation::destroy(const PolygonMeshHandle& polygonMeshHandle, const ObstacleHandle& obstacleHandle)
{
    LOG_DEBUG(logger_, "Destroying obstacle %s with polygon mesh %s", obstacleHandle, polygonMeshHandle);

	auto& polygonMesh = polygonMeshes_[polygonMeshHandle];
	auto& obstacle = polygonMesh.obstacles[obstacleHandle];

	polygonMesh.tileCache->removeObstacle(obstacle.obstacleReference);

	polygonMesh.obstacles.destroy(obstacleHandle);

}

NavigationMeshHandle RecastNavigation::createNavigationMesh(const PolygonMeshHandle& polygonMeshHandle, const NavigationMeshConfig& navigationMeshConfig)
{
    LOG_DEBUG(logger_, "Creating navigation mesh with polygon mesh %s and navigation mesh config %s", polygonMeshHandle, navigationMeshConfig);

	auto navigationMeshHandle = navigationMeshes_.create();
	auto& navigationMesh = navigationMeshes_[navigationMeshHandle];

	navigationMesh.polygonMeshHandle = polygonMeshHandle;

	const auto& polygonMesh = polygonMeshes_[polygonMeshHandle];

	// Create Detour data from Recast poly mesh.
//	if (polygonMesh.config.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
//	{
		navigationMesh.navMeshQuery = std::unique_ptr<dtNavMeshQuery, decltype(&dtFreeNavMeshQuery)>(dtAllocNavMeshQuery(), dtFreeNavMeshQuery);
/*
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		for (int i = 0; i < polygonMesh.polyMesh->npolys; ++i)
		{
			if (polygonMesh.polyMesh->areas[i] == RC_WALKABLE_AREA)
				polygonMesh.polyMesh->areas[i] = RECAST_NAVIGATION_POLYAREA_GROUND;

			if (polygonMesh.polyMesh->areas[i] == RECAST_NAVIGATION_POLYAREA_GROUND ||
				polygonMesh.polyMesh->areas[i] == RECAST_NAVIGATION_POLYAREA_GRASS ||
				polygonMesh.polyMesh->areas[i] == RECAST_NAVIGATION_POLYAREA_ROAD)
			{
				polygonMesh.polyMesh->flags[i] = RECAST_NAVIGATION_POLYFLAGS_WALK;
			}
			else if (polygonMesh.polyMesh->areas[i] == RECAST_NAVIGATION_POLYAREA_WATER)
			{
				polygonMesh.polyMesh->flags[i] = RECAST_NAVIGATION_POLYFLAGS_SWIM;
			}
			else if (polygonMesh.polyMesh->areas[i] == RECAST_NAVIGATION_POLYAREA_DOOR)
			{
				polygonMesh.polyMesh->flags[i] = RECAST_NAVIGATION_POLYFLAGS_WALK | RECAST_NAVIGATION_POLYFLAGS_DOOR;
			}
		}
*/
//		navigationMesh.params.verts = polygonMesh.polyMesh->verts;
//		navigationMesh.params.vertCount = polygonMesh.polyMesh->nverts;
//		navigationMesh.params.polys = polygonMesh.polyMesh->polys;
//		navigationMesh.params.polyAreas = polygonMesh.polyMesh->areas;
//		navigationMesh.params.polyFlags = polygonMesh.polyMesh->flags;
//		navigationMesh.params.polyCount = polygonMesh.polyMesh->npolys;
//		navigationMesh.params.nvp = polygonMesh.polyMesh->nvp;
//		navigationMesh.params.detailMeshes = polygonMesh.polyMeshDetail->meshes;
//		navigationMesh.params.detailVerts = polygonMesh.polyMeshDetail->verts;
//		navigationMesh.params.detailVertsCount = polygonMesh.polyMeshDetail->nverts;
//		navigationMesh.params.detailTris = polygonMesh.polyMeshDetail->tris;
//		navigationMesh.params.detailTriCount = polygonMesh.polyMeshDetail->ntris;
//		navigationMesh.params.offMeshConVerts = nullptr;
//		navigationMesh.params.offMeshConRad = nullptr;
//		navigationMesh.params.offMeshConDir = nullptr;
//		navigationMesh.params.offMeshConAreas = nullptr;
//		navigationMesh.params.offMeshConFlags = nullptr;
//		navigationMesh.params.offMeshConUserID = nullptr;
//		navigationMesh.params.offMeshConCount = 0;
//		navigationMesh.params.walkableHeight = navigationMeshConfig.walkableHeight;
//		navigationMesh.params.walkableRadius = navigationMeshConfig.walkableRadius;
//		navigationMesh.params.walkableClimb = navigationMeshConfig.walkableClimb;
//		rcVcopy(navigationMesh.params.bmin, polygonMesh.polyMesh->bmin);
//		rcVcopy(navigationMesh.params.bmax, polygonMesh.polyMesh->bmax);
//		navigationMesh.params.cs = polygonMesh.config.cs;
//		navigationMesh.params.ch = polygonMesh.config.ch;
//		navigationMesh.params.buildBvTree = true;

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)dtIlog2(dtNextPow2(polygonMesh.tileCacheParams.maxTiles)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;

		rcVcopy(navigationMesh.navMeshParams.orig, polygonMesh.config.bmin);
		navigationMesh.navMeshParams.tileWidth = polygonMesh.config.tileSize * polygonMesh.config.cs;
		navigationMesh.navMeshParams.tileHeight = polygonMesh.config.tileSize * polygonMesh.config.cs;
		navigationMesh.navMeshParams.maxTiles = 1 << tileBits;
		navigationMesh.navMeshParams.maxPolys = 1 << polyBits;

//		if (!dtCreateNavMeshData(&navigationMesh.params, &navData, &navDataSize))
//		{
//			throw std::runtime_error("Unable to build Detour navmesh.");
//		}

		navigationMesh.navMesh = std::unique_ptr<dtNavMesh, decltype(&dtFreeNavMesh)>(dtAllocNavMesh(), dtFreeNavMesh);
		if (!navigationMesh.navMesh)
		{
//			dtFree(navData);
			throw Exception("Unable to create Detour navmesh");
		}

		dtStatus status;

//		status = navigationMesh.navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		status = navigationMesh.navMesh->init(&navigationMesh.navMeshParams);
		if (dtStatusFailed(status))
		{
//			dtFree(navData);
			throw Exception("Unable to init Detour navmesh: " + getErrorMessage(status));
		}

		status = navigationMesh.navMeshQuery->init(navigationMesh.navMesh.get(), 2048);
		if (dtStatusFailed(status))
		{
			throw Exception("Unable to init Detour navmesh query: " + getErrorMessage(status));
		}

		const int tileWidth = (polygonMesh.gridWidth + polygonMesh.config.tileSize-1) / polygonMesh.config.tileSize;
		const int tileHeight = (polygonMesh.gradeHeight + polygonMesh.config.tileSize-1) / polygonMesh.config.tileSize;

		// build initial meshes
		for (int y = 0; y < tileHeight; ++y)
		{
			for (int x = 0; x < tileWidth; ++x)
			{
				polygonMesh.tileCache->buildNavMeshTilesAt(x,y, navigationMesh.navMesh.get());
			}
		}

		polygonMesh.tileCache->update(0, navigationMesh.navMesh.get());
//	}

	return navigationMeshHandle;
}

void RecastNavigation::destroy(const NavigationMeshHandle& navigationMeshHandle)
{
    LOG_DEBUG(logger_, "Destroying navigation mesh %s", navigationMeshHandle);

	navigationMeshes_.destroy(navigationMeshHandle);
}

CrowdHandle RecastNavigation::createCrowd(const PathfindingSceneHandle& pathfindingSceneHandle, const NavigationMeshHandle& navigationMeshHandle, const CrowdConfig& crowdConfig)
{
    LOG_DEBUG(logger_, "Creating crowd in scene %s with navigation mesh %s and crowd config %s", pathfindingSceneHandle, navigationMeshHandle, crowdConfig);

	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];

	auto crowdHandle = pathfindingScene.crowds.create();
	auto& crowd = pathfindingScene.crowds[crowdHandle];

	crowd.crowd = std::unique_ptr<dtCrowd, decltype(&dtFreeCrowd)>(dtAllocCrowd(), dtFreeCrowd);
	crowd.navigationMeshHandle = navigationMeshHandle;

	auto& navigationMesh = navigationMeshes_[navigationMeshHandle];

	crowd.crowd->init(crowdConfig.maxAgents, crowdConfig.maxAgentRadius, navigationMesh.navMesh.get()); //MAX_AGENTS, m_sample->getAgentRadius(), nav);

	// Make polygons with 'disabled' flag invalid.
	crowd.crowd->getEditableFilter(0)->setExcludeFlags(RECAST_NAVIGATION_POLYFLAGS_DISABLED);

	// Setup local avoidance params to different qualities.
	dtObstacleAvoidanceParams params;
	// Use mostly default settings, copy from dtCrowd.
	memcpy(&params, crowd.crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

	// Low (11)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 1;
	crowd.crowd->setObstacleAvoidanceParams(0, &params);

	// Medium (22)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 2;
	crowd.crowd->setObstacleAvoidanceParams(1, &params);

	// Good (45)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 3;
	crowd.crowd->setObstacleAvoidanceParams(2, &params);

	// High (66)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 3;
	params.adaptiveDepth = 3;

	crowd.crowd->setObstacleAvoidanceParams(3, &params);

	return crowdHandle;
}

void RecastNavigation::destroy(const PathfindingSceneHandle& pathfindingSceneHandle, const CrowdHandle& crowdHandle)
{
    LOG_DEBUG(logger_, "Destroying crowd %s in scene %s", crowdHandle, pathfindingSceneHandle);

	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	pathfindingScene.crowds.destroy(crowdHandle);
}

AgentHandle RecastNavigation::createAgent(
        const PathfindingSceneHandle& pathfindingSceneHandle,
        const CrowdHandle& crowdHandle,
        const glm::vec3& position,
        const AgentParams& agentParams,
        std::unique_ptr<IAgentMotionChangeListener> agentMotionChangeListener,
        std::unique_ptr<IAgentStateChangeListener> agentStateChangeListener,
        std::unique_ptr<IMovementRequestStateChangeListener> movementRequestStateChangeListener,
        const boost::any &userData
)
{
	LOG_DEBUG(logger_, "Creating agent in crowd %s in scene %s with parameters %s", crowdHandle, pathfindingSceneHandle, agentParams);

	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	auto& crowd = pathfindingScene.crowds[crowdHandle];

	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));

	ap.radius = agentParams.radius;
	ap.height = agentParams.height;
	ap.maxAcceleration = agentParams.maxAcceleration;
	ap.maxSpeed = agentParams.maxSpeed;
	ap.collisionQueryRange = agentParams.collisionQueryRange;
	ap.pathOptimizationRange = agentParams.pathOptimizationRange;
	ap.separationWeight = agentParams.separationWeight;

	ap.updateFlags = 0;
	//if (m_toolParams.m_anticipateTurns)
		ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	//if (m_toolParams.m_optimizeVis)
		ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	//if (m_toolParams.m_optimizeTopo)
		ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	//if (m_toolParams.m_obstacleAvoidance)
		ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	//if (m_toolParams.m_separation)
		ap.updateFlags |= DT_CROWD_SEPARATION;
	ap.obstacleAvoidanceType = (unsigned char)3.0f;

	LOG_TRACE(logger_, "Attempting to add agent at %s with parameters %s", position, ap);
	int idx = crowd.crowd->addAgent(&position.x, &ap);
	if (idx == -1) throw Exception("Unable to add agent");

    const dtCrowdAgent* ag = crowd.crowd->getAgent(idx);
    if (ag->state == DT_CROWDAGENT_STATE_INVALID)
    {
        LOG_WARN(logger_, "Agent with index %s isn't in a valid state", idx);
    }

	auto agentHandle = crowd.agents.create();
	auto& agent = crowd.agents[agentHandle];

	agent.index = idx;
	agent.agentMotionChangeListener = std::move(agentMotionChangeListener);
	agent.agentStateChangeListener = std::move(agentStateChangeListener);
	agent.movementRequestStateChangeListener = std::move(movementRequestStateChangeListener);

	return agentHandle;
}

void RecastNavigation::destroy(const PathfindingSceneHandle& pathfindingSceneHandle, const CrowdHandle& crowdHandle, const AgentHandle& agentHandle)
{
    LOG_DEBUG(logger_, "Destroying agent %s in crowd %s in scene %s", agentHandle, crowdHandle, pathfindingSceneHandle);

	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	auto& crowd = pathfindingScene.crowds[crowdHandle];
	crowd.agents.destroy(agentHandle);
}

void RecastNavigation::requestMoveTarget(
	const PathfindingSceneHandle& pathfindingSceneHandle,
	const CrowdHandle& crowdHandle,
	const AgentHandle& agentHandle,
	const glm::vec3& position
)
{
    LOG_DEBUG(logger_, "Requesting move target for agent %s in crowd %s in scene %s to position %s", agentHandle, crowdHandle, pathfindingSceneHandle, position);

	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	auto& crowd = pathfindingScene.crowds[crowdHandle];
	const auto& agent = crowd.agents[agentHandle];

	const auto& navigationMesh = navigationMeshes_[crowd.navigationMeshHandle];

	/*
	int32 entityIndex = -1;
	for (int32 i=0; i < entities_.size(); i++)
	{
		if (entities_[i] == entity)
		{
			entityIndex = i;
			break;
		}
	}
	if (entityIndex == -1)
	{
		LOG_WARN(logger_, "Unable to move agent - agent not found.");
		return;
	}
	*/

	const dtQueryFilter* filter = crowd.crowd->getFilter(0);
	const float* ext = crowd.crowd->getQueryExtents();
	dtPolyRef targetRef;
	float targetPos[3];
	float p[3] = {position.x, position.y, position.z};

	navigationMesh.navMeshQuery->findNearestPoly(p, ext, filter, &targetRef, targetPos);

	if (!targetRef)
	{
		LOG_WARN(logger_, "Unable to move agent %s in crowd %s in scene %s to position %s - A suitable target position was not found.", agentHandle, crowdHandle, pathfindingSceneHandle, position);
		return;
	}
	
	const auto moveRequestSuccess = crowd.crowd->requestMoveTarget(agent.index, targetRef, targetPos);
	
	if (!moveRequestSuccess) LOG_WARN(logger_, "Unable to move agent - move request failed.");
}

void RecastNavigation::resetMoveTarget(
	const PathfindingSceneHandle& pathfindingSceneHandle,
	const CrowdHandle& crowdHandle,
	const AgentHandle& agentHandle
)
{
    LOG_DEBUG(logger_, "Resetting move target for agent %s in crowd %s in scene %s", agentHandle, crowdHandle, pathfindingSceneHandle);

	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	auto& crowd = pathfindingScene.crowds[crowdHandle];
	const auto& agent = crowd.agents[agentHandle];

	const auto& navigationMesh = navigationMeshes_[crowd.navigationMeshHandle];

	/*
	int32 entityIndex = -1;
	for (int32 i=0; i < entities_.size(); i++)
	{
		if (entities_[i] == entity)
		{
			entityIndex = i;
			break;
		}
	}
	if (entityIndex == -1)
	{
		LOG_WARN(logger_, "Unable to move agent - agent not found.");
		return;
	}
	*/

	const auto resetRequestSuccess = crowd.crowd->resetMoveTarget(agent.index);

	if (!resetRequestSuccess) LOG_WARN(logger_, "Unable to reset agent move target - reset move target request failed.");
}

void RecastNavigation::requestMoveVelocity(
	const PathfindingSceneHandle& pathfindingSceneHandle,
	const CrowdHandle& crowdHandle,
	const AgentHandle& agentHandle,
	const glm::vec3& velocity
)
{
    LOG_DEBUG(logger_, "Requesting move velocity for agent %s in crowd %s in scene %s to velocity %s", agentHandle, crowdHandle, pathfindingSceneHandle, velocity);

	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	auto& crowd = pathfindingScene.crowds[crowdHandle];
	const auto& agent = crowd.agents[agentHandle];

	const auto& navigationMesh = navigationMeshes_[crowd.navigationMeshHandle];

	/*
	int32 entityIndex = -1;
	for (int32 i=0; i < entities_.size(); i++)
	{
		if (entities_[i] == entity)
		{
			entityIndex = i;
			break;
		}
	}
	if (entityIndex == -1)
	{
		LOG_WARN(logger_, "Unable to move agent - agent not found.");
		return;
	}
	*/

	const auto moveRequestSuccess = crowd.crowd->requestMoveVelocity(agent.index, &velocity[0]);

	if (!moveRequestSuccess) LOG_WARN(logger_, "Unable to move agent - move velocity request failed.");
}

void RecastNavigation::setMotionChangeListener(
	const PathfindingSceneHandle& pathfindingSceneHandle,
	const CrowdHandle& crowdHandle,
	const AgentHandle& agentHandle,
	std::unique_ptr<IAgentMotionChangeListener> agentMotionChangeListener
)
{
	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	auto& crowd = pathfindingScene.crowds[crowdHandle];
	auto& agent = crowd.agents[agentHandle];
	
	agent.agentMotionChangeListener = std::move(agentMotionChangeListener);
}

void RecastNavigation::setStateChangeListener(
	const PathfindingSceneHandle& pathfindingSceneHandle,
	const CrowdHandle& crowdHandle,
	const AgentHandle& agentHandle,
	std::unique_ptr<IAgentStateChangeListener> agentStateChangeListener
)
{
	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	auto& crowd = pathfindingScene.crowds[crowdHandle];
	auto& agent = crowd.agents[agentHandle];

	agent.agentStateChangeListener = std::move(agentStateChangeListener);
}

void RecastNavigation::setMovementRequestChangeListener(
	const PathfindingSceneHandle& pathfindingSceneHandle,
	const CrowdHandle& crowdHandle,
	const AgentHandle& agentHandle,
	std::unique_ptr<IMovementRequestStateChangeListener> movementRequestStateChangeListener
)
{
	auto& pathfindingScene = pathfindingScenes_[pathfindingSceneHandle];
	auto& crowd = pathfindingScene.crowds[crowdHandle];
	auto& agent = crowd.agents[agentHandle];

	agent.movementRequestStateChangeListener = std::move(movementRequestStateChangeListener);
}

void RecastNavigation::setUserData(const PathfindingSceneHandle& pathfindingSceneHandle, const CrowdHandle& crowdHandle, const AgentHandle& agentHandle, const boost::any& userData)
{

}

boost::any ud;
boost::any& RecastNavigation::getUserData(const PathfindingSceneHandle& pathfindingSceneHandle, const CrowdHandle& crowdHandle, const AgentHandle& agentHandle) const
{
	return ud;
}

}
}
}
