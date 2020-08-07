#pragma once

/*
 *	aabbTree.h
 *
 *	Dynamic AABB tree for collision broadphase.
 *
 */

#include"physics/physStruct.h"
#include"objects/collider.h"


struct AABB{
	const Collider* collider;
	Vec3 upperBound;
	Vec3 lowerBound;
	bool isStatic;

	AABB() : collider(nullptr) {}
};

struct AABBPair{
	AABB& aabb1;
	AABB& aabb2;
};


class AABBTree{

public:
	struct Node{
		AABB* aabb;
		AABB* aabbMargin;
		Node* parent;
		Node* child1;
		Node* child2;
		bool branchChecked;

		Node() : aabb(new AABB()), aabbMargin(nullptr), parent(nullptr), child1(nullptr), child2(nullptr), branchChecked(false) {}

		~Node(){
			delete aabb;
			delete aabbMargin;
		}

		bool isLeaf(){
			return child1 == nullptr;
		}
	};


private:
	Node* root_;
	vector<AABBPair> overlapping_;
	vector<Node*> invalid_;


	void updateNode(Node* node);
	void updateAABB(Node* node);

	void clear(Node* node);

	void addNode(Node* node, Node* parent);

	void remove(const Collider* collider, Node* node);
	void removeNode(Node* node, bool deleteNode = true);


	void resetBranchChecked(Node* node);
	void checkOverlap(Node* node1, Node* node2);
	bool overlapping(AABB* aabb1, AABB* aabb2);

public:
	AABBTree();

	void update();
	void clear();

	void add(const Collider* collider);
	void remove(const Collider* collider);

	const vector<AABBPair>& getOverlapping();
};
