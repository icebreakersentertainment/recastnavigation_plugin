#ifndef PATHFINDINGDEBUGRENDERER_H_
#define PATHFINDINGDEBUGRENDERER_H_

//#include <DebugUtils/DebugDraw.h>
#include <DebugDraw.h>

#include "pathfinding/IPathfindingDebugRenderer.hpp"

#include "Platform.hpp"

#if defined(PLATFORM_LINUX)
inline unsigned int duDebugDraw::areaToCol(unsigned int area)
{
	if (area == 0)
	{
		// Treat zero area type as default.
		return duRGBA(0, 192, 255, 255);
	}
	else
	{
		return duIntToCol(area, 255);
	}
}
#endif

namespace ice_engine
{
namespace pathfinding
{
namespace recastnavigation
{

class DebugRenderer : public duDebugDraw
{
public:
	DebugRenderer(pathfinding::IPathfindingDebugRenderer* pathfindingDebugRenderer) : pathfindingDebugRenderer_(pathfindingDebugRenderer)
	{
	}
	
	~DebugRenderer() override = default;
	
	void depthMask(bool state) override {};
	void texture(bool state) override {};

	void begin(duDebugDrawPrimitives prim, float size = 1.0f) override
	{
		primitiveType_ = prim;
	};

	void vertex(const float* pos, unsigned int color) override
	{
		vertex(glm::vec3(pos[0], pos[1], pos[2]), parseColor(color));
	};

	void vertex(const float x, const float y, const float z, unsigned int color) override
	{
		vertex(glm::vec3(x, y, z), parseColor(color));
	};

	void vertex(const float* pos, unsigned int color, const float* uv) override
	{
		vertex(glm::vec3(pos[0], pos[1], pos[2]), parseColor(color));
	};

	void vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v) override
	{
		vertex(glm::vec3(x, y, z), parseColor(color));
	};

	void end() override {};

	glm::vec3 parseColor(unsigned int color)
	{
		return glm::vec3(color & 0xFF000000,color & 0x00FF0000,color & 0x0000FF00);
	}

	void vertex(const glm::vec3& vertex, const glm::vec3& color)
	{
		switch (primitiveType_)
		{
			case DU_DRAW_POINTS:
				break;

			case DU_DRAW_LINES:
				vertices_.push_back(vertex);
				if (vertices_.size() > 1)
				{
					pathfindingDebugRenderer_->pushLine(vertices_[0], vertices_[1], color);
					vertices_.clear();
				}

				break;

			case DU_DRAW_TRIS:
				vertices_.push_back(vertex);
				if (vertices_.size() > 2)
				{
					pathfindingDebugRenderer_->pushLine(vertices_[0], vertices_[1], color);
					pathfindingDebugRenderer_->pushLine(vertices_[1], vertices_[2], color);
					pathfindingDebugRenderer_->pushLine(vertices_[2], vertices_[0], color);
					vertices_.clear();
				}

				break;

			case DU_DRAW_QUADS:
				vertices_.push_back(vertex);
				if (vertices_.size() > 3)
				{
					pathfindingDebugRenderer_->pushLine(vertices_[0], vertices_[1], color);
					pathfindingDebugRenderer_->pushLine(vertices_[1], vertices_[2], color);
					pathfindingDebugRenderer_->pushLine(vertices_[2], vertices_[3], color);
					pathfindingDebugRenderer_->pushLine(vertices_[3], vertices_[0], color);
					vertices_.clear();
				}

				break;
		}
	};

	/*
	void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override
	{
		pathfindingDebugRenderer_->pushLine(glm::vec3(from.x(), from.y(), from.z()), glm::vec3(to.x(), to.y(), to.z()), glm::vec3(color.x(), color.y(), color.z()));
	};

	void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) override {};
	void reportErrorWarning(const char* warningString) override {};
	void draw3dText(const btVector3& location, const char* textString) override {};
	void setDebugMode(int debugMode) override
	{
		debugDrawMode_ = debugMode;
	};

	int getDebugMode() const override
	{
		return debugDrawMode_;
	};
	*/
private:
	IPathfindingDebugRenderer* pathfindingDebugRenderer_ = nullptr;
	
	duDebugDrawPrimitives primitiveType_;
	std::vector<glm::vec3> vertices_;
	glm::vec3 color_;
};

}
}
}

#endif /* PATHFINDINGDEBUGRENDERER_H_ */
