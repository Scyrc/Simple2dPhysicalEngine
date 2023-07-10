#pragma once
#include "body.h"
#include "aabb.h"
#include <map>
namespace physicalEngine
{
	struct BVHTreeNode
	{
		BVHTreeNode(Body* body_, AABB aabbBound_) :body(body_), aabbBound(aabbBound_){}
		Body* body = nullptr;        
		AABB aabbBound;  // enlarged AABB bound

		BVHTreeNode* parent = nullptr;
		BVHTreeNode* childLeft = nullptr;
		BVHTreeNode* childRight = nullptr;

		bool isLeaf() const { return childLeft == nullptr && childRight ==  nullptr;}
	};


	class BVHTree
	{
	public:
		BVHTree() =default;
		~BVHTree();
		void insert(Body* body);
		

		void generate(std::vector<Body*> BodyList);
		void update(std::vector<Body*> BodyList);
		void draw();
		
	protected:
		BVHTreeNode* banlace(BVHTreeNode* Node);
		int computeHeight(BVHTreeNode* Node);
		void insertLeaf(BVHTreeNode* leafNode);
		void freeNode(BVHTreeNode* Node);
		void update(Body* body);

		void RemoveLeaf(BVHTreeNode* leafNode);
		void merge(BVHTreeNode* Sibing, BVHTreeNode* Node);
		void updateAABB(BVHTreeNode* Node);

		BVHTreeNode* findBestSibing(BVHTreeNode* Node);
		void getInheritedCost(BVHTreeNode* currentNode, AABB& NewAABB, Double& InheritedCost);
	private:
		
		std::map<Body*, BVHTreeNode*> leavesMap;

		BVHTreeNode* root = nullptr;
	};

}