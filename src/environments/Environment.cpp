//
// Created by dinko on 14.02.22.
//

#include "Environment.h"

#include <glog/logging.h>

typedef std::shared_ptr <fcl::CollisionGeometry> CollisionGeometryPtr;

env::Environment::~Environment() {}

env::Environment::Environment(const std::string& filename)
{
    
}

env::Environment::Environment(const fcl::Box& box, const fcl::Transform3f& tf)
{
    CollisionGeometryPtr fclBox(new fcl::Box(box.side[0], box.side[1], box.side[2]));
	std::shared_ptr<fcl::CollisionObject> ob(new fcl::CollisionObject(fclBox, tf));

    ob->computeAABB();
    LOG(INFO) << ob->getAABB().min_ << "\t" << ob->getAABB().max_;

    parts_.emplace_back(ob);

}

const std::vector<std::shared_ptr<fcl::CollisionObject> >& env::Environment::Environment::getParts() const
{
	return parts_;
}