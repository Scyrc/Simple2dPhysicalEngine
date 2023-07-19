
#include "../../include/collision/bvhtree.h"
#include <queue>
#include <iostream>

namespace physicalEngine
{
	physicalEngine::BVHTree::~BVHTree()
	{

	}

	void BVHTree::destory()
	{

	}

	void BVHTree::generate(std::vector<Body*> BodyList)
	{
		for (auto& body : BodyList)
		{
			insert(body);
		}
	}


	void physicalEngine::BVHTree::insert(Body* body)
	{
		static int count = 1;
		auto iter = leavesMap.find(body);
		if (iter != leavesMap.end())
		{
			update(body);
			return;
		}
		AABB leafAABB = AABB(body);
		leafAABB.zoomSize(1.3);

		auto leafNode = new BVHTreeNode(body, leafAABB);
		leavesMap.emplace(body, leafNode);
		insertLeaf(leafNode);
	}
	void BVHTree::insertLeaf(BVHTreeNode* leafNode)
	{
		if (root == nullptr)
		{
			root = leafNode;
			root->parent = nullptr;
			return;
		}

		BVHTreeNode* Sibing = findBestSibling(leafNode);

		merge(Sibing, leafNode);
		
	}

	void physicalEngine::BVHTree::update(Body* body)
	{
		auto iter = leavesMap.find(body);
		if (iter == leavesMap.end())
		{
			insert(body);
			return;
		}

		auto leafNode = iter->second;

		if (leafNode->aabbBound.Contain(AABB(body)))
		{
			return;
		}

		RemoveLeaf(leafNode);

		AABB fatAABB(body);
		fatAABB.zoomSize(1.3);
		leafNode->aabbBound = fatAABB;

		insertLeaf(leafNode);

	}

	void BVHTree::merge(BVHTreeNode* Sibing, BVHTreeNode* leafNode)
	{
		auto oldParent = Sibing->parent;
		AABB leafAABB = leafNode->aabbBound;

		AABB combineAABB(leafAABB, Sibing->aabbBound);
		auto newParent = new BVHTreeNode(nullptr, combineAABB);
		newParent->parent = oldParent;

		if(oldParent != nullptr)
		{
			if (oldParent->childLeft == Sibing)
			{
				oldParent->childLeft = newParent;
			}
			else
			{
				oldParent->childRight = newParent;
			}

			newParent->childLeft = Sibing;
			newParent->childRight = leafNode;

			Sibing->parent = newParent;
			leafNode->parent = newParent;
		}

		else
		{
			newParent->childLeft = Sibing;
			newParent->childRight = leafNode;

			Sibing->parent = newParent;
			leafNode->parent = newParent;

			root = newParent;
		}

		updateAABB(newParent);

	}

	void BVHTree::updateAABB(BVHTreeNode* Node)
	{
		while (Node != nullptr)
		{
			Node = banlace(Node);
			Node->aabbBound.Combine(Node->childLeft->aabbBound, Node->childRight->aabbBound);
			Node = Node->parent;
		}
	}


	BVHTreeNode* BVHTree::banlace(BVHTreeNode* NodeA)
	{
		if (NodeA->isLeaf() || computeHeight(NodeA) < 2)
			return NodeA;


		BVHTreeNode* NodeB = NodeA->childLeft;
		BVHTreeNode* NodeC = NodeA->childRight;

		int diff = computeHeight(NodeC) - computeHeight(NodeB);

		// Rotate C up
		if (diff > 1)
		{
			BVHTreeNode* NodeF = NodeC->childLeft;
			BVHTreeNode* NodeG = NodeC->childRight;

			assert(NodeF != nullptr);
			assert(NodeG != nullptr);

			// Swap A and C

			NodeC->childLeft = NodeA;
			NodeC->parent = NodeA->parent;
			NodeA->parent = NodeC;

			// A's old parent should point to C

			if (NodeC->parent != nullptr)
			{
				if (NodeC->parent->childLeft == NodeA)
				{
					NodeC->parent->childLeft = NodeC;
				}
				else
				{
					assert(NodeB->parent->childRight = NodeA);
					NodeC->parent->childRight = NodeC;
				}
			}

			else
			{
				root = NodeC;
			}

			// Rotate

			if (computeHeight(NodeF) > computeHeight(NodeG))
			{
				NodeC->childRight = NodeF;
				NodeA->childRight = NodeG;
				NodeG->parent = NodeA;
				NodeA->aabbBound.Combine(NodeB->aabbBound, NodeG->aabbBound);
				NodeC->aabbBound.Combine(NodeA->aabbBound, NodeF->aabbBound);

			}
			else
			{
				NodeC->childRight = NodeG;
				NodeA->childRight = NodeF;
				NodeF->parent = NodeA;
				NodeA->aabbBound.Combine(NodeB->aabbBound, NodeF->aabbBound);
				NodeC->aabbBound.Combine(NodeA->aabbBound, NodeG->aabbBound);
			}

			return NodeC;
		}

		// Rotate B up
		if (diff < -1)
		{
			BVHTreeNode* NodeD = NodeB->childLeft;
			BVHTreeNode* NodeE = NodeB->childRight;
			assert(NodeD != nullptr);
			assert(NodeE != nullptr);


			// swap a and b

			NodeB->childLeft = NodeA;
			NodeB->parent = NodeA->parent;
			NodeA->parent = NodeB;

			// A's old parent should point to B
			if (NodeB->parent != nullptr)
			{
				if (NodeB->parent->childLeft == NodeA)
				{
					NodeB->parent->childLeft = NodeB;

				}
				else
				{
					assert(NodeB->parent->childRight = NodeA);
					NodeB->parent->childRight = NodeB;
				}
			}
			else
			{
				root = NodeB;
			}

			//Rotate

			if (computeHeight(NodeD) > computeHeight(NodeE))
			{
				NodeB->childRight = NodeD;
				NodeA->childLeft = NodeE;
				NodeE->parent = NodeA;

				NodeA->aabbBound.Combine(NodeC->aabbBound, NodeE->aabbBound);
				NodeB->aabbBound.Combine(NodeA->aabbBound, NodeD->aabbBound);
			}

			else
			{
				NodeB->childRight = NodeE;
				NodeA->childLeft = NodeD;
				NodeD->parent = NodeA;

				NodeA->aabbBound.Combine(NodeC->aabbBound, NodeD->aabbBound);
				NodeB->aabbBound.Combine(NodeA->aabbBound, NodeE->aabbBound);
			}

			return NodeB;
		}

		return NodeA;
	}

	int BVHTree::computeHeight(BVHTreeNode* Node)
	{
		if (Node == nullptr)
			return 0;

		if (Node->isLeaf())
			return 0;

		return 1 + std::max(computeHeight(Node->childLeft), computeHeight(Node->childRight));
	}



	void BVHTree::freeNode(BVHTreeNode* Node)
	{

	}

	void BVHTree::update(std::vector<Body*> BodyList)
	{
		for (auto& body : BodyList)
		{
			update(body);
		}
	}

	void BVHTree::RemoveLeaf(BVHTreeNode* leafNode)
	{
		if (leafNode == root)
		{
			root = nullptr;
			return;
		}

		BVHTreeNode* parent = leafNode->parent;
		BVHTreeNode* grandParent = parent->parent;
		BVHTreeNode* siblingNode;
		if (parent->childLeft == leafNode)
		{
			siblingNode = parent->childRight;
		}
		else
		{
			assert(parent->childRight == leafNode);
			siblingNode = parent->childLeft;
		}

		if (grandParent != nullptr) // Destroy parent and connect sibling to grandParent
		{
			if (grandParent->childLeft == parent)
			{
				grandParent->childLeft = siblingNode;
			}
			else
			{
				assert(grandParent->childRight == parent);
				grandParent->childRight = siblingNode;
			}

			siblingNode->parent = grandParent;
			freeNode(parent);

			updateAABB(grandParent);
		}
		else
		{
			root = siblingNode;
			siblingNode->parent = nullptr;
			freeNode(parent);
		}
	}

	BVHTreeNode* BVHTree::findBestSibling(BVHTreeNode* Node)
	{
		AABB leafAABB = Node->aabbBound;
		
		Double bestCost = 100000.f;
		BVHTreeNode* bestNode = root;
		std::queue<BVHTreeNode*> candidateNodes;
		candidateNodes.push(root);
		while (!candidateNodes.empty())
		{
			BVHTreeNode* temp = candidateNodes.front();
			candidateNodes.pop();


			AABB combineAABB;
			combineAABB.Combine(temp->aabbBound, leafAABB);

			float combineArea = combineAABB.getSurfaceArea();
			Double inheritedCost = 0;
			getInheritedCost(temp, combineAABB, inheritedCost);
			float Cost = combineArea + inheritedCost;

			if (Cost < bestCost)
			{
				bestCost = Cost;
				bestNode = temp;
			}
			if (true) //todo leafAABB.getSurfaceArea() + combineArea - temp->aabbBound.getSurfaceArea() + inheritedCost < bestCost
			{
				if (temp->childLeft != nullptr)
					candidateNodes.push(temp->childLeft);

				if (temp->childRight != nullptr)
					candidateNodes.push(temp->childRight);
			}
		}

		return bestNode;
	}

	void BVHTree::getInheritedCost(BVHTreeNode* currentNode, AABB& NewAABB, Double& InheritedCost)
	{
		if (currentNode->parent == nullptr) return;
		float oldArea = currentNode->parent->aabbBound.getSurfaceArea();

		if (currentNode->parent->childLeft == currentNode)
		{
			if (currentNode->parent->childRight)
			{
				AABB tempAABB;
				tempAABB.Combine(NewAABB, currentNode->parent->childRight->aabbBound);
				float NewArea = tempAABB.getSurfaceArea();

				if (NewArea > oldArea)
				{
					InheritedCost += (NewArea - oldArea);
					getInheritedCost(currentNode->parent, tempAABB, InheritedCost);
				}
			}
		}

		else if(currentNode->parent->childRight == currentNode)
		{
			if (currentNode->parent->childLeft)
			{
				AABB tempAABB;
				tempAABB.Combine(NewAABB, currentNode->parent->childLeft->aabbBound);
				float NewArea = tempAABB.getSurfaceArea();

				if (NewArea > oldArea)
				{
					InheritedCost += (NewArea - oldArea);
					getInheritedCost(currentNode->parent, tempAABB, InheritedCost);
				}
			}
		}
	}

	void BVHTree::draw()
	{
		if (root == nullptr) return;

		std::queue<BVHTreeNode*> nodeQueue;
		nodeQueue.push(root);

		while (!nodeQueue.empty())
		{
			BVHTreeNode* node = nodeQueue.front();
			nodeQueue.pop();
			node->aabbBound.draw();

			if (node->childLeft)
				nodeQueue.push(node->childLeft);

			if (node->childRight)
				nodeQueue.push(node->childRight);
		}
	}
}


