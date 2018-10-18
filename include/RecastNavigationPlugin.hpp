#ifndef RECASTNAVIGATIONPLUGIN_H_
#define RECASTNAVIGATIONPLUGIN_H_

#include <memory>

#include "IPathfindingPlugin.hpp"

namespace ice_engine
{

class RecastNavigationPlugin : public IPathfindingPlugin
{
public:
	RecastNavigationPlugin() = default;
	virtual ~RecastNavigationPlugin() override = default;

	virtual std::string getName() const override;

	virtual std::unique_ptr<pathfinding::IPathfindingEngineFactory> createFactory() const override;

};

}

#endif /* RECASTNAVIGATIONPLUGIN_H_ */
