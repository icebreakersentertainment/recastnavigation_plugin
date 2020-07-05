#ifndef RECASTNAVIGATION_H_
#define RECASTNAVIGATION_H_

#include <memory>
#include <ostream>

//#include <Recast/Recast.h>
//#include <Detour/DetourCommon.h>
//#include <Detour/DetourNavMesh.h>
//#include <Detour/DetourNavMeshBuilder.h>
//#include <Detour/DetourNavMeshQuery.h>
//#include <DetourTileCache/DetourTileCache.h>
//#include <DetourTileCache/DetourTileCacheBuilder.h>
//#include <DetourCrowd/DetourCrowd.h>

#include <Recast.h>
#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <DetourTileCache.h>
#include <DetourTileCacheBuilder.h>
#include <DetourCrowd.h>

#include <ChunkyTriMesh.h>

#include <fastlz.h>
#include <boost/any.hpp>

#include "pathfinding/IPathfindingEngine.hpp"

#include "DebugRenderer.hpp"

#include "handles/HandleVector.hpp"
#include "utilities/Properties.hpp"
#include "fs/IFileSystem.hpp"
#include "logger/ILogger.hpp"

#include "detail/DebugSerializer.hpp"

inline std::ostream& operator<<(std::ostream& os, const dtCrowdAgentParams& agentParams)
{
    os << "dtCrowdAgentParams("
    	PRINT_TO_STREAM(agentParams, radius)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, height)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, maxAcceleration)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, maxSpeed)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, collisionQueryRange)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, pathOptimizationRange)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, separationWeight)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, updateFlags)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, obstacleAvoidanceType)
		PRINT_DELIMITER() PRINT_TO_STREAM(agentParams, queryFilterType)
		<< ")";

    return os;
}

namespace ice_engine
{
namespace pathfinding
{
namespace recastnavigation
{

struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize* 1.05f);
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
							  unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
								unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;

	LinearAllocator(const size_t cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const size_t cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}

	virtual void* alloc(const size_t size)
	{
		if (!buffer)
			return 0;
		if (top+size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

enum RecastNavigationPartitionType
{
	RECAST_NAVIGATION_PARTITION_WATERSHED = 0,
	RECAST_NAVIGATION_PARTITION_MONOTONE,
	RECAST_NAVIGATION_PARTITION_LAYERS,
};

enum RecastNavigationPolyAreas
{
	RECAST_NAVIGATION_POLYAREA_GROUND,
	RECAST_NAVIGATION_POLYAREA_WATER,
	RECAST_NAVIGATION_POLYAREA_ROAD,
	RECAST_NAVIGATION_POLYAREA_DOOR,
	RECAST_NAVIGATION_POLYAREA_GRASS,
	RECAST_NAVIGATION_POLYAREA_JUMP,
};

enum RecastNavigationPolyFlags
{
	RECAST_NAVIGATION_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
	RECAST_NAVIGATION_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
	RECAST_NAVIGATION_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
	RECAST_NAVIGATION_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
	RECAST_NAVIGATION_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
	RECAST_NAVIGATION_POLYFLAGS_ALL			= 0xffff	// All abilities.
};

struct MeshProcess : public dtTileCacheMeshProcess
{
//	InputGeom* m_geom;
//
//	inline MeshProcess() : m_geom(0)
//	{
//	}
//
//	inline void init(InputGeom* geom)
//	{
//		m_geom = geom;
//	}

	virtual void process(struct dtNavMeshCreateParams* params, unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = RECAST_NAVIGATION_POLYAREA_GROUND;

			if (polyAreas[i] == RECAST_NAVIGATION_POLYAREA_GROUND ||
				polyAreas[i] == RECAST_NAVIGATION_POLYAREA_GRASS ||
				polyAreas[i] == RECAST_NAVIGATION_POLYAREA_ROAD)
			{
				polyFlags[i] = RECAST_NAVIGATION_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == RECAST_NAVIGATION_POLYAREA_WATER)
			{
				polyFlags[i] = RECAST_NAVIGATION_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == RECAST_NAVIGATION_POLYAREA_DOOR)
			{
				polyFlags[i] = RECAST_NAVIGATION_POLYFLAGS_WALK | RECAST_NAVIGATION_POLYFLAGS_DOOR;
			}
		}

		// Pass in off-mesh connections.
//		if (m_geom)
//		{
			params->offMeshConVerts = nullptr;
			params->offMeshConRad = nullptr;
			params->offMeshConDir = nullptr;
			params->offMeshConAreas = nullptr;
			params->offMeshConFlags = nullptr;
			params->offMeshConUserID = nullptr;
			params->offMeshConCount = 0;
//		}
	}
};

struct TileCacheData
{
//	std::vector<unsigned char> data;
	unsigned char* data = nullptr;
	int dataSize = 0;
};

struct Obstacle
{
	dtObstacleRef obstacleReference;
};

struct RecastNavigationPolygonMesh
{
	RecastNavigationPolygonMesh()
	{
		memset(&config, 0, sizeof(config));
		memset(&tileCacheParams, 0, sizeof(tileCacheParams));
	}

	std::unique_ptr<dtTileCache, decltype(&dtFreeTileCache)> tileCache = std::unique_ptr<dtTileCache, decltype(&dtFreeTileCache)>(nullptr, dtFreeTileCache);

	std::unique_ptr<rcContourSet, decltype(&rcFreeContourSet)> contourSet = std::unique_ptr<rcContourSet, decltype(&rcFreeContourSet)>(nullptr, rcFreeContourSet);
	std::unique_ptr<rcPolyMesh, decltype(&rcFreePolyMesh)> polyMesh = std::unique_ptr<rcPolyMesh, decltype(&rcFreePolyMesh)>(nullptr, rcFreePolyMesh);
	std::unique_ptr<rcPolyMeshDetail, decltype(&rcFreePolyMeshDetail)> polyMeshDetail = std::unique_ptr<rcPolyMeshDetail, decltype(&rcFreePolyMeshDetail)>(nullptr, rcFreePolyMeshDetail);

	LinearAllocator linearAllocator = LinearAllocator(32000);
	FastLZCompressor fastrLzCompressor;
	MeshProcess meshProcess;

	std::vector<glm::vec3> vertices;
	std::vector<uint32> indices;
	rcChunkyTriMesh chunkyMesh;

	rcConfig config;
	dtTileCacheParams tileCacheParams;

	handles::HandleVector<Obstacle, ObstacleHandle> obstacles;

	int gridWidth = 0;
	int gradeHeight = 0;
	int cacheLayerCount = 0;
	int cacheCompressedSize = 0;
	int cacheRawSize = 0;
	std::vector<TileCacheData> tiles;
};

struct RecastNavigationNavigationMesh
{
	RecastNavigationNavigationMesh()
	{
		memset(&params, 0, sizeof(params));
		memset(&navMeshParams, 0, sizeof(navMeshParams));
	}

	PolygonMeshHandle polygonMeshHandle;

	std::unique_ptr<dtNavMesh, decltype(&dtFreeNavMesh)> navMesh = std::unique_ptr<dtNavMesh, decltype(&dtFreeNavMesh)>(nullptr, dtFreeNavMesh);
	std::unique_ptr<dtNavMeshQuery, decltype(&dtFreeNavMeshQuery)> navMeshQuery = std::unique_ptr<dtNavMeshQuery, decltype(&dtFreeNavMeshQuery)>(nullptr, dtFreeNavMeshQuery);

	dtNavMeshCreateParams params;
	dtNavMeshParams navMeshParams;
};

struct RecastNavigationAgent
{
	int32 index = -1;
	unsigned char state = DT_CROWDAGENT_STATE_INVALID;
	unsigned char moveRequestState = DT_CROWDAGENT_TARGET_NONE;
	std::unique_ptr<IAgentMotionChangeListener> agentMotionChangeListener = nullptr;
	std::unique_ptr<IAgentStateChangeListener> agentStateChangeListener = nullptr;
	std::unique_ptr<IMovementRequestStateChangeListener> movementRequestStateChangeListener = nullptr;
};

struct RecastNavigationCrowd
{
	NavigationMeshHandle navigationMeshHandle;
	std::unique_ptr<dtCrowd, decltype(&dtFreeCrowd)> crowd = std::unique_ptr<dtCrowd, decltype(&dtFreeCrowd)>(nullptr, dtFreeCrowd);

	CrowdConfig crowdConfig;

	handles::HandleVector<RecastNavigationAgent, AgentHandle> agents;
};

struct RecastNavigationPathfindingScene
{
	handles::HandleVector<RecastNavigationCrowd, CrowdHandle> crowds;

	bool debugRendering = false;
};

class RecastNavigation : public IPathfindingEngine
{
public:
	RecastNavigation(utilities::Properties* properties, fs::IFileSystem* fileSystem, logger::ILogger* logger);
	virtual ~RecastNavigation() override = default;

	RecastNavigation(const RecastNavigation& other) = delete;

	virtual void tick(const PathfindingSceneHandle& pathfindingSceneHandle, const float32 delta) override;

	virtual PathfindingSceneHandle createPathfindingScene() override;
	virtual void destroyPathfindingScene(const PathfindingSceneHandle& pathfindingSceneHandle) override;

	virtual void setPathfindingDebugRenderer(IPathfindingDebugRenderer* pathfindingDebugRenderer) override;
	virtual void setDebugRendering(const PathfindingSceneHandle& pathfindingSceneHandle, const bool enabled) override;

	virtual PolygonMeshHandle createPolygonMesh(const ITerrain* terrain, const PolygonMeshConfig& polygonMeshConfig = PolygonMeshConfig()) override;
	virtual void destroy(const PolygonMeshHandle& polygonMeshHandle) override;

	virtual ObstacleHandle createObstacle(const PolygonMeshHandle& polygonMeshHandle, const glm::vec3& position, const float32 radius, const float32 height) override;
	virtual void destroy(const PolygonMeshHandle& polygonMeshHandle, const ObstacleHandle& obstacleHandle) override;

	virtual NavigationMeshHandle createNavigationMesh(const PolygonMeshHandle& polygonMeshHandle, const NavigationMeshConfig& navigationMeshConfig = NavigationMeshConfig()) override;
	virtual void destroy(const NavigationMeshHandle& navigationMeshHandle) override;

	virtual CrowdHandle createCrowd(const PathfindingSceneHandle& pathfindingSceneHandle, const NavigationMeshHandle& navigationMeshHandle, const CrowdConfig& crowdConfig) override;
	virtual void destroy(const PathfindingSceneHandle& pathfindingSceneHandle, const CrowdHandle& crowdHandle) override;

	virtual AgentHandle createAgent(
            const PathfindingSceneHandle& pathfindingSceneHandle,
            const CrowdHandle& crowdHandle,
            const glm::vec3& position,
            const AgentParams& agentParams = AgentParams(),
            std::unique_ptr<IAgentMotionChangeListener> agentMotionChangeListener = nullptr,
            std::unique_ptr<IAgentStateChangeListener> agentStateChangeListener = nullptr,
            std::unique_ptr<IMovementRequestStateChangeListener> movementRequestStateChangeListener = nullptr,
            const boost::any &userData = UserData()
	) override;
	virtual void destroy(const PathfindingSceneHandle& pathfindingSceneHandle, const CrowdHandle& crowdHandle, const AgentHandle& agentHandle) override;

	virtual void requestMoveTarget(
		const PathfindingSceneHandle& pathfindingSceneHandle,
		const CrowdHandle& crowdHandle,
		const AgentHandle& agentHandle,
		const glm::vec3& position
	) override;

	virtual void resetMoveTarget(
		const PathfindingSceneHandle& pathfindingSceneHandle,
		const CrowdHandle& crowdHandle,
		const AgentHandle& agentHandle
	) override;

	virtual void requestMoveVelocity(
		const PathfindingSceneHandle& pathfindingSceneHandle,
		const CrowdHandle& crowdHandle,
		const AgentHandle& agentHandle,
		const glm::vec3& velocity
	) override;

	//virtual void test2(entityx::Entity entity, Scene* scene, const glm::vec3& position, const glm::quat& orientation) override;
	//virtual void test3(entityx::Entity entity, const glm::vec3& position, const glm::quat& orientation) override;

	virtual void setMotionChangeListener(
		const PathfindingSceneHandle& pathfindingSceneHandle,
		const CrowdHandle& crowdHandle,
		const AgentHandle& agentHandle,
		std::unique_ptr<IAgentMotionChangeListener> agentMotionChangeListener
	) override;
	virtual void setStateChangeListener(
		const PathfindingSceneHandle& pathfindingSceneHandle,
		const CrowdHandle& crowdHandle,
		const AgentHandle& agentHandle,
		std::unique_ptr<IAgentStateChangeListener> agentStateChangeListener
	) override;
	virtual void setMovementRequestChangeListener(
		const PathfindingSceneHandle& pathfindingSceneHandle,
		const CrowdHandle& crowdHandle,
		const AgentHandle& agentHandle,
		std::unique_ptr<IMovementRequestStateChangeListener> movementRequestStateChangeListener
	) override;

	virtual void setUserData(
            const PathfindingSceneHandle& pathfindingSceneHandle,
            const CrowdHandle& crowdHandle,
            const AgentHandle& agentHandle,
            const boost::any& userData
	) override;
	virtual boost::any& getUserData(
		const PathfindingSceneHandle& pathfindingSceneHandle,
		const CrowdHandle& crowdHandle,
		const AgentHandle& agentHandle
	) const override;
	

private:
	utilities::Properties* properties_;
	fs::IFileSystem* fileSystem_;
	logger::ILogger* logger_;

	handles::HandleVector<RecastNavigationPathfindingScene, PathfindingSceneHandle> pathfindingScenes_;
	handles::HandleVector<RecastNavigationPolygonMesh, PolygonMeshHandle> polygonMeshes_;
	handles::HandleVector<RecastNavigationNavigationMesh, NavigationMeshHandle> navigationMeshes_;

	std::unique_ptr<DebugRenderer> debugRenderer_;
};

}
}
}

#endif /* RECASTNAVIGATION_H_ */
