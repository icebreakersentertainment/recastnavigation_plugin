#include "RecastNavigationFactory.hpp"

#include "RecastNavigation.hpp"

namespace ice_engine
{
namespace pathfinding
{
namespace recastnavigation
{

std::unique_ptr<IPathfindingEngine> RecastNavigationFactory::create(
	utilities::Properties* properties,
	fs::IFileSystem* fileSystem,
	logger::ILogger* logger
)
{
	std::unique_ptr<IPathfindingEngine> ptr = std::make_unique< RecastNavigation >(
		properties,
		fileSystem,
		logger
	);
	
	return std::move( ptr );
}

}
}
}
