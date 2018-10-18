#ifndef RECASTNAVIGATIONFACTORY_H_
#define RECASTNAVIGATIONFACTORY_H_

#include <memory>

#include "pathfinding/IPathfindingEngineFactory.hpp"

namespace ice_engine
{
namespace pathfinding
{
namespace recastnavigation
{

class RecastNavigationFactory : public IPathfindingEngineFactory
{
public:
	RecastNavigationFactory() = default;
	virtual ~RecastNavigationFactory() override = default;

	virtual std::unique_ptr<IPathfindingEngine> create(
		utilities::Properties* properties,
		fs::IFileSystem* fileSystem,
		logger::ILogger* logger
	) override;

};

}
}
}

#endif /* RECASTNAVIGATIONFACTORY_H_ */
