#include"aabbtree.h"

#include"physics/physDefine.h"
#include"objects/portal.h"
#include<algorithm>
#include<limits>

using std::min;
using std::max;


AABBTree::AABBTree() : root_(nullptr) {

}

void AABBTree::update(){

	if(!root_)
		return;

	// Clear invalid nodes
	invalid_.clear();

	// Update tree
	updateNode(root_);

	// Re-insert invaild nodes
	for(Node* node : invalid_){

		// Update large AABB
		node->aabbMargin->lowerBound = node->aabb->lowerBound - NTW_AABB_MARGIN;
		node->aabbMargin->upperBound = node->aabb->upperBound + NTW_AABB_MARGIN;

		// Remove and add again
		removeNode(node, false);
		addNode(node, root_);
	}
}

void AABBTree::updateNode(Node* node){

	// Node is leaf
	if(node->isLeaf()){

		// Update AABB is collider's parent object's hitbox has been updated
		if(node->aabb->collider->parent && node->aabb->collider->parent->cacheTransformedHitbox()){
			updateAABB(node);

			// If AABB has moved outside margin, mark it invalid
			if(	node->aabb->lowerBound[0] < node->aabbMargin->lowerBound[0] ||
				node->aabb->lowerBound[1] < node->aabbMargin->lowerBound[1] ||
				node->aabb->lowerBound[2] < node->aabbMargin->lowerBound[2] ||
				node->aabb->upperBound[0] > node->aabbMargin->upperBound[0] ||
				node->aabb->upperBound[1] > node->aabbMargin->upperBound[1] ||
				node->aabb->upperBound[2] > node->aabbMargin->upperBound[2]){

				invalid_.push_back(node);
			}
		}
	}

	// Node is branch, update children
	else{
		updateNode(node->child1);
		updateNode(node->child2);
	}
}

void AABBTree::updateAABB(Node* node){

	AABB* aabb = node->aabb;

	// Leaf node, update based on AABB's collider
	if(node->isLeaf()){

		// Min/max coordinate values
		aabb->lowerBound = std::numeric_limits<float>::max();
		aabb->upperBound = -aabb->lowerBound;

		bool isPortal = aabb->collider->portal;

		// Get transformed collider vertices or portal vertices
		const vector<Vec3>& vertices =	aabb->collider->parent ? aabb->collider->hitboxTransformed.vertices :
										isPortal ? aabb->collider->portal->getVertices() :
										aabb->collider->hitbox->vertices;

		// Get bounding coordinates
		for(const Vec3& v : vertices){
			aabb->lowerBound[0] = min(aabb->lowerBound[0], v[0]);
			aabb->lowerBound[1] = min(aabb->lowerBound[1], v[1]);
			aabb->lowerBound[2] = min(aabb->lowerBound[2], v[2]);

			aabb->upperBound[0] = max(aabb->upperBound[0], v[0]);
			aabb->upperBound[1] = max(aabb->upperBound[1], v[1]);
			aabb->upperBound[2] = max(aabb->upperBound[2], v[2]);
		}

		// Add margin for portals
		if(isPortal){
			aabb->lowerBound -= NTW_AABB_PORTAL_MARGIN;
			aabb->upperBound += NTW_AABB_PORTAL_MARGIN;
		}
	}

	// Branch node, take min/max of child AABBs
	else{
		for(int i = 0; i < 3; i++){

			// Use enlarged AABB for children that have them
			float cl1 = node->child1->aabbMargin ? node->child1->aabbMargin->lowerBound[i] : node->child1->aabb->lowerBound[i];
			float cl2 = node->child2->aabbMargin ? node->child2->aabbMargin->lowerBound[i] : node->child2->aabb->lowerBound[i];
			float cu1 = node->child1->aabbMargin ? node->child1->aabbMargin->upperBound[i] : node->child1->aabb->upperBound[i];
			float cu2 = node->child2->aabbMargin ? node->child2->aabbMargin->upperBound[i] : node->child2->aabb->upperBound[i];

			aabb->lowerBound[i] = min(cl1, cl2);
			aabb->upperBound[i] = max(cu1, cu2);
		}
	}
}

void AABBTree::clear(){
	
	if(!root_)
		return;

	if(root_->isLeaf())
		delete root_;

	// Recursively delete nodes
	else
		clear(root_);

	root_ = nullptr;
}

void AABBTree::clear(Node* node){

	// Delete leaf children and recursively clear branch children
	if(node->child1->isLeaf())
		delete node->child1;
	else
		clear(node->child1);

	if(node->child2->isLeaf())
		delete node->child2;
	else
		clear(node->child2);
}


void AABBTree::add(const Collider* collider){

	// Create node for this collider
	Node* node = new Node();
	node->aabb->collider = collider;
	node->aabb->isStatic = true;

	// Cache parent object's transformed hitbox and update AABB
	if(collider->parent){
		collider->parent->cacheTransformedHitbox();
		updateAABB(node);

		// Set non-static if necessary
		if(collider->parent->getPhysicsType() != PhysicsType::STATIC)
			node->aabb->isStatic = false;
	}

	// Collider belongs to portal
	else
		updateAABB(node);

	// Set large AABB for dynamic objects
	if(!node->aabb->isStatic){
		node->aabbMargin = new AABB();
		node->aabbMargin->lowerBound = node->aabb->lowerBound - NTW_AABB_MARGIN;
		node->aabbMargin->upperBound = node->aabb->upperBound + NTW_AABB_MARGIN;
	}

	// First node
	if(!root_)
		root_ = node;
	else
		addNode(node, root_);
}

void AABBTree::addNode(Node* node, Node* parent){

	// Parent is leaf
	if(parent->isLeaf()){

		// Create new branch node
		Node* newNode = new Node();
		newNode->parent = parent->parent;
		newNode->child1 = node;
		newNode->child2 = parent;

		// Update grandparent child
		if(parent->parent){
			if(parent->parent->child1 == parent)
				parent->parent->child1 = newNode;
			else
				parent->parent->child2 = newNode;
		}

		// No grandparent, parent is root
		else
			root_ = newNode;

		// Set child parents
		newNode->child1->parent = newNode;
		newNode->child2->parent = newNode;

		// Set this AABB as static if both children are static
		newNode->aabb->isStatic = newNode->child1->aabb->isStatic && newNode->child2->aabb->isStatic;

		updateAABB(newNode);
	}

	// Parent is branch
	else{
		// Compute volume difference if node is added to either child
		AABB* box	= node->aabb;
		AABB* box1	= parent->child1->aabb;
		AABB* box2	= parent->child2->aabb;

		Vec3 aabbSize1 = Vec3(
			box1->upperBound[0] - box1->lowerBound[0],
			box1->upperBound[1] - box1->lowerBound[1],
			box1->upperBound[2] - box1->lowerBound[2]
		);
		Vec3 aabbNewSize1 = Vec3(
			max(box->upperBound[0], box1->upperBound[0]) - min(box->lowerBound[0], box1->lowerBound[0]),
			max(box->upperBound[1], box1->upperBound[1]) - min(box->lowerBound[1], box1->lowerBound[1]),
			max(box->upperBound[2], box1->upperBound[2]) - min(box->lowerBound[2], box1->lowerBound[2])
		);
		Vec3 aabbSize2 = Vec3(
			box1->upperBound[0] - box1->lowerBound[0],
			box1->upperBound[1] - box1->lowerBound[1],
			box1->upperBound[2] - box1->lowerBound[2]
		);
		Vec3 aabbNewSize2 = Vec3(
			max(box->upperBound[0], box2->upperBound[0]) - min(box->lowerBound[0], box2->lowerBound[0]),
			max(box->upperBound[1], box2->upperBound[1]) - min(box->lowerBound[1], box2->lowerBound[1]),
			max(box->upperBound[2], box2->upperBound[2]) - min(box->lowerBound[2], box2->lowerBound[2])
		);

		float volumeDifference1 = (aabbNewSize1[0] * aabbNewSize1[1] * aabbNewSize1[2]) - (aabbSize1[0] * aabbSize1[1] * aabbSize1[2]);
		float volumeDifference2 = (aabbNewSize2[0] * aabbNewSize2[1] * aabbNewSize2[2]) - (aabbSize2[0] * aabbSize2[1] * aabbSize2[2]);

		// Add node to child that has less volume increase
		if(volumeDifference1 < volumeDifference2)
			addNode(node, parent->child1);
		else
			addNode(node, parent->child2);

		// Update parent
		updateAABB(parent);
	}
}

void AABBTree::remove(const Collider* collider){

	if(!root_)
		return;

	// Root node contains collider
	if(root_->isLeaf() && root_->aabb->collider == collider)
		removeNode(root_);

	// Search for and remove collider node
	else
		remove(collider, root_);
}

void AABBTree::remove(const Collider* collider, Node* node){

	// Check leaf children and recursively check branch children
	if(node->child1->isLeaf()){
		if(node->child1->aabb->collider == collider){
			removeNode(node->child1);
			return;
		}
	}
	else
		remove(collider, node->child1);

	if(node->child2->isLeaf()){
		if(node->child2->aabb->collider == collider)
			removeNode(node->child2);
		return;
	}
	else
		remove(collider, node->child2);
}

void AABBTree::removeNode(Node* node, bool deleteNode){

	// Node is root
	if(!node->parent){
		root_ = nullptr;

		if(deleteNode)
			delete node;

		return;
	}

	// Get sibling
	Node* sibling = node->parent->child1;

	if(node->parent->child1 == node)
		sibling = node->parent->child2;


	// Node has grandparent
	if(node->parent->parent){
		sibling->parent = node->parent->parent;
		
		// Replace grandparent child
		if(sibling->parent->child1 == node->parent)
			sibling->parent->child1 = sibling;
		else
			sibling->parent->child2 = sibling;
	}

	// Parent is root
	else{
		root_ = sibling;
		sibling->parent = nullptr;
	}

	delete node->parent;

	if(deleteNode)
		delete node;
	else
		node->parent = nullptr;
}


const vector<AABBPair>& AABBTree::getOverlapping(){

	overlapping_.clear();

	// No root or root is leaf, return
	if(!root_ || root_->isLeaf())
		return overlapping_;


	// Reset branch checked flags for entire tree
	resetBranchChecked(root_);

	// Check tree recusively
	checkOverlap(root_->child1, root_->child2);

	return overlapping_;
}

void AABBTree::resetBranchChecked(Node* node){
	node->branchChecked = false;

	if(!node->isLeaf()){
		if(!node->child1->isLeaf()) resetBranchChecked(node->child1);
		if(!node->child2->isLeaf()) resetBranchChecked(node->child2);
	}
}

void AABBTree::checkOverlap(Node* node1, Node* node2){

	// Two leaf nodes
	if(node1->isLeaf() && node2->isLeaf()){

		// Add to list if overlapping
		if(overlapping(node1->aabb, node2->aabb))
			overlapping_.push_back({*node1->aabb, *node2->aabb});

		return;
	}

	// Check overlaps in children
	if(!node1->isLeaf() && !node1->branchChecked){
		checkOverlap(node1->child1, node1->child2);
		node1->branchChecked = true;
	}

	if(!node2->isLeaf() && !node2->branchChecked){
		checkOverlap(node2->child1, node2->child2);
		node2->branchChecked = true;
	}


	// Check overlaps between children
	if(!node1->isLeaf()){
		if(!node2->isLeaf()){
			// Both nodes are branches, check them against each other
			checkOverlap(node1->child1, node2->child1);
			checkOverlap(node1->child1, node2->child2);
			checkOverlap(node1->child2, node2->child1);
			checkOverlap(node1->child2, node2->child2);
		}
		else{
			checkOverlap(node1->child1, node2);
			checkOverlap(node1->child2, node2);
		}
	}
	else{
		checkOverlap(node1, node2->child1);
		checkOverlap(node1, node2->child2);
	}
}

bool AABBTree::overlapping(AABB* aabb1, AABB* aabb2){

	// Disable overlaps between AABBs belonging to the same object
	if(aabb1->collider && aabb2->collider &&
		aabb1->collider->parent && aabb2->collider->parent &&
		aabb1->collider->parent == aabb2->collider->parent)
		return false;

	// Disable overlaps between two static object AABBs
	if(aabb1->isStatic && aabb2->isStatic)
		return false;

	// Check if AABBs are not overlapping on each axis
	for(int i = 0; i < 3; i++)
		if(aabb1->lowerBound[i] > aabb2->upperBound[i] || aabb2->lowerBound[i] > aabb1->upperBound[i])
			return false;

	return true;
}
