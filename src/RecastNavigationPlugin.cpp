#include <boost/config.hpp> // for BOOST_SYMBOL_EXPORT

#include "RecastNavigationPlugin.hpp"

#include "RecastNavigationFactory.hpp"

namespace ice_engine
{

std::string RecastNavigationPlugin::getName() const
{
	return std::string("recastnavigation");
}

std::unique_ptr<pathfinding::IPathfindingEngineFactory> RecastNavigationPlugin::createFactory() const
{
	std::unique_ptr<pathfinding::IPathfindingEngineFactory> ptr = std::make_unique< pathfinding::recastnavigation::RecastNavigationFactory >();
	
	return std::move( ptr );
}

// Exporting `my_namespace::plugin` variable with alias name `plugin`
// (Has the same effect as `BOOST_DLL_ALIAS(my_namespace::plugin, plugin)`)
extern "C" BOOST_SYMBOL_EXPORT RecastNavigationPlugin plugin;
RecastNavigationPlugin plugin;

}
