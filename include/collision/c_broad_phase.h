#pragma once


#include "bvhtree.h"
#include "../body/body.h"

namespace physicalEngine
{
	class BroadPhase
	{
	public:

		static void detectOverlapAABB(BVHTree* bvhTree, std::vector<std::pair<Body*, Body*>>& overlapedAABBObjects)
		{
			detectOverlap(bvhTree->root, overlapedAABBObjects);
		}

	private:
		static void detectOverlap(BVHTreeNode* Node, std::vector<std::pair<Body*, Body*>>& overlapedAABBObjects)
		{

			if (Node == nullptr || Node->isLeaf())
				return;

			if (Node->childLeft->aabbBound.overlap(Node->childRight->aabbBound))
				detectOverlap(Node->childLeft, Node->childRight, overlapedAABBObjects);

			detectOverlap(Node->childLeft, overlapedAABBObjects);
			detectOverlap(Node->childRight, overlapedAABBObjects);

		}
		static void detectOverlap(BVHTreeNode * NodeL, BVHTreeNode * NodeR, std::vector<std::pair<Body*, Body*>>& overlapedAABBObjects)
		{
			if (NodeL == nullptr || NodeR == nullptr) return;

			bool result = NodeL->aabbBound.overlap(NodeR->aabbBound) ||
						  NodeL->aabbBound.Contain(NodeR->aabbBound) ||
						  NodeR->aabbBound.Contain(NodeL->aabbBound);

			if (!result) return;

			if (NodeL->isLeaf() && NodeR->isLeaf())
			{
				if (AABB(NodeL->body).overlap(AABB(NodeR->body)))
				{
					std::pair<Body*, Body*> bodyPair = { NodeL->body, NodeR->body };
					overlapedAABBObjects.emplace_back(bodyPair);
				}
			}

			if (NodeL->isLeaf() && NodeR->isBranch())
			{
				detectOverlap(NodeL, NodeR->childLeft, overlapedAABBObjects);
				detectOverlap(NodeL, NodeR->childRight, overlapedAABBObjects);
			}

			if (NodeR->isLeaf() && NodeL->isBranch())
			{
				detectOverlap(NodeR, NodeL->childLeft, overlapedAABBObjects);
				detectOverlap(NodeR, NodeL->childRight, overlapedAABBObjects);
			}

			if (NodeL->isBranch() && NodeR->isBranch())
			{
				detectOverlap(NodeL->childLeft, NodeR, overlapedAABBObjects);
				detectOverlap(NodeL->childRight, NodeR, overlapedAABBObjects);
			}
		}
	};
}
