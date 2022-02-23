//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_STATE_H
#define RPMPL_STATE_H

#include "StateSpaceType.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <nanoflann.hpp>

namespace base
{
	class State
	{
	private:
		StateSpaceType stateSpaceType;
		std::shared_ptr<State> parent = nullptr;
		std::shared_ptr<std::vector<std::shared_ptr<State>>> children = 
			std::make_shared<std::vector<std::shared_ptr<base::State>>>();
		
	public:
		State(){};
		virtual ~State() = 0;

		virtual uint getTreeIdx() const = 0;
		virtual size_t getIdx() const = 0;
		virtual float getDistance() const = 0;
		virtual float getCost() const = 0;
		virtual std::shared_ptr<std::vector<Eigen::MatrixXf>> getPlanes() const = 0;
		virtual int getDimensions() const = 0;
		virtual const Eigen::VectorXf &getCoord() const = 0;
		virtual const float getCoord(int idx) const = 0;
		inline std::shared_ptr<State> getParent() const { return parent; }
		inline std::shared_ptr<std::vector<std::shared_ptr<State>>> getChildren() const { return children; };
		inline StateSpaceType getStateSpaceType() const { return stateSpaceType; }

		virtual void setTreeIdx(uint treeIdx_) = 0;
		virtual void setIdx(size_t idx_) = 0;
		virtual void setDistance(float d_c_) = 0;
		virtual void setCost(float cost_) = 0;
		virtual void setPlanes(std::shared_ptr<std::vector<Eigen::MatrixXf>> planes_) = 0;
		virtual void setCoord(const Eigen::VectorXf &coord) = 0;
		virtual void setCoord(const float coord_, int idx) = 0;
		inline void setParent(std::shared_ptr<State> parent_) { parent = parent_; }
		inline void setChildren(std::shared_ptr<std::vector<std::shared_ptr<State>>> children_) { children = children_; }
		inline void setStateSpaceType(StateSpaceType stateSpaceType_) { stateSpaceType = stateSpaceType_; }

		void addChild(std::shared_ptr<State> child);
		friend std::ostream& operator<<(std::ostream& os, const State* state);
	};

	class Tree
	{
	private:
		std::string treeName;
		uint treeIdx; 			// Tree index
		std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states = 
			std::make_shared<std::vector<std::shared_ptr<base::State>>>();	// List of all nodes in the tree
	public:
		typedef nanoflann::KDTreeSingleIndexDynamicAdaptor
			<nanoflann::L2_Simple_Adaptor<float, base::Tree>, base::Tree /* dim */> KdTree;

		Tree(std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states_) : states(states_) {}
		Tree(std::string treeName_, uint treeNum_);
		Tree() {}
		~Tree() {}

		inline const std::string &getTreeName() const { return treeName; }
		inline const uint getTreeIdx() const { return treeIdx; }
		inline std::shared_ptr<std::vector<std::shared_ptr<base::State>>> getStates() const { return states; }
		inline std::shared_ptr<base::State> getState(size_t idx) const { return states->at(idx); }

		inline void setTreeName(const std::string &treeName_) { treeName = treeName_; }
		inline void setTreeIdx(const uint treeIdx_) { treeIdx = treeIdx_; }
		inline void setStates(std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states_) { states = states_; }
		inline void setState(std::shared_ptr<base::State> state, size_t idx) { states->at(idx) = state; }

		void clearTree();
		std::shared_ptr<base::State> getNearestState(std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q);
		std::shared_ptr<base::State> getNearestStateV2(std::shared_ptr<base::State> q);
		void upgradeTree(std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q_new, std::shared_ptr<base::State> q_parent,
						 float d_c = -1, std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = nullptr, float cost = -1);
		void upgradeTree(std::shared_ptr<base::State> q_new, std::shared_ptr<base::State> q_parent,
						 float d_c = -1, std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = nullptr, float cost = -1);

		inline size_t kdtree_get_point_count() const { return states->size(); }
		inline float kdtree_get_pt(const size_t idx, const size_t dim) const { return states->at(idx)->getCoord()[dim]; }
		template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
		friend std::ostream& operator<<(std::ostream& os, const Tree& tree);
	};

}
#endif //RPMPL_STATE_H
