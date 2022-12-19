#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>
#include <mrnav/planner_6124e1a8685f/robot.hpp>
#include <mrnav/planner_6124e1a8685f/config.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
class Planner< Kernel >::Problem
{

public:
  using Planner_ = Planner< Kernel >;
  using Robot = typename Planner_::Robot;
  using Configuration = typename Planner_::Configuration;
  using GeometryKernel = typename Planner_::GeometryKernel;
  using Point = typename GeometryKernel::Point;
  using PolygonRegionSet = typename GeometryKernel::PolygonRegionSet;

private:
  const size_t _size;
  PolygonRegionSet ws;
  std::vector< Robot > robots;
  Configuration pos_init;
  Configuration pos_goal;

private:
  Problem();

public:
  Problem(size_t size);

  size_t size();

  void set_ws(const PolygonRegionSet& ws);
  const PolygonRegionSet& get_ws() const;

  void set_robot(size_t idx, const Robot& robot);
  const Robot& get_robot(size_t idx) const;

  void set_conf_init(const Configuration& pos);
  const Configuration& get_pos_init() const;

  void set_conf_goal(const Configuration& pos);
  const Configuration& get_pos_goal() const;

public:
  bool valid_p() const;

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "problem") const;

public:
  friend Planner_;

};

} // mrnav::planner_6124e1a8685f
