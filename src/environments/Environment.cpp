//
// Created by dinko on 14.02.22.
//

#include "Environment.h"

#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"

#include <glog/logging.h>

typedef std::shared_ptr <fcl::CollisionGeometry> CollisionGeometryPtr;

env::Environment::~Environment() {}

env::Environment::Environment(const std::string& filename)
{
    YAML::Node node = YAML::LoadFile(filename);
    std::vector<std::shared_ptr<fcl::CollisionObject> > parts_;
    for (size_t i = 0; i < node["obstacles"].size(); ++i)
	{
        YAML::Node obstacle = node["obstacles"][i];
        if(obstacle["box"].IsDefined())
        {
            // BOX
            YAML::Node box_size = obstacle["box"]["dim"];
            float bx = box_size[0].as<float>();
            float by = box_size[1].as<float>();
            float bz = box_size[2].as<float>();
            CollisionGeometryPtr fclBox(new fcl::Box(bx, by, bz));

            YAML::Node trans = obstacle["box"]["trans"];
            float tx = trans[0].as<float>();
            float ty = trans[1].as<float>();
            float tz = trans[2].as<float>();

            YAML::Node rot = obstacle["box"]["rot"];
            float rx = rot[1].as<float>();
            float ry = rot[2].as<float>();
            float rz = rot[3].as<float>();
            float rw = rot[0].as<float>();
            
            fcl::Vec3f tr(tx, ty, tz);
            fcl::Quaternion3f quat(rw, rx, ry, rz);

            fcl::Transform3f tf(quat, tr);
            LOG(INFO) << "Object tf: " << tf.getTranslation() << "\n" << tf.getRotation() << "\n------------";
            std::shared_ptr<fcl::CollisionObject> ob(new fcl::CollisionObject(fclBox, tf));
            ob->computeAABB();
            parts_.emplace_back(ob);
        }
    }        
}

env::Environment::Environment(const fcl::Box& box, const fcl::Transform3f& tf)
{
    CollisionGeometryPtr fclBox(new fcl::Box(box.side[0], box.side[1], box.side[2]));
	std::shared_ptr<fcl::CollisionObject> ob(new fcl::CollisionObject(fclBox, tf));

    ob->computeAABB();
    parts_.emplace_back(ob);
}

env::Environment::Environment(std::vector<env::Obstacle> obs)
{
    for (size_t i = 0; i < obs.size(); ++i)
    {
        CollisionGeometryPtr fclBox(new fcl::Box(obs[i].first.side[0], obs[i].first.side[1], obs[i].first.side[2]));
        std::shared_ptr<fcl::CollisionObject> ob(new fcl::CollisionObject(fclBox, obs[i].second));
        ob->computeAABB();
        LOG(INFO) << "Obstacle range: " << ob->getAABB().min_ << "\t" << ob->getAABB().max_;
        parts_.emplace_back(ob);
    }

}

const std::vector<std::shared_ptr<fcl::CollisionObject> >& env::Environment::Environment::getParts() const
{
	return parts_;
}