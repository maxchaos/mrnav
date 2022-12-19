#pragma once

#include <mrnav/planner_6124e1a8685f/problem.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
Planner< Kernel >::Problem::Problem(size_t size_)
  : _size{size_}
  , ws{}
  , robots{}
  , pos_init{_size}
  , pos_goal{_size}
{
  if(_size < 1)
    throw std::invalid_argument("size must be a positive integer");
  robots.resize(_size);
}

template< class Kernel >
size_t
Planner< Kernel >::Problem::size()
{
  return this->_size;
}

template< class Kernel >
void
Planner< Kernel >::Problem::set_ws(const PolygonRegionSet& ws_)
{
  this->ws = ws_;
}

template< class Kernel >
const typename Planner< Kernel >::Problem::PolygonRegionSet&
Planner< Kernel >::Problem::get_ws() const
{
  return this->ws;
}

template< class Kernel >
void
Planner< Kernel >::Problem::set_robot(size_t idx, const Robot& robot)
{
  if(idx >= this->_size)
    throw std::out_of_range("robot index out of range");
  this->robots[idx] = robot;
}

template< class Kernel >
const typename Planner< Kernel >::Problem::Robot&
Planner< Kernel >::Problem::get_robot(size_t idx) const
{
  if(idx >= this->_size)
    throw std::out_of_range("robot index out of range");
  return this->robots[idx];
}

template< class Kernel >
void
Planner< Kernel >::Problem::set_conf_init(const Configuration& pos)
{
  if(pos.size() != this->_size)
    throw std::invalid_argument("size mismatch");
  this->pos_init = pos;
}

template< class Kernel >
const typename Planner< Kernel >::Problem::Configuration&
Planner< Kernel >::Problem::get_pos_init() const
{
  return this->pos_init;
}

template< class Kernel >
void
Planner< Kernel >::Problem::set_conf_goal(const Configuration& pos)
{
  if(pos.size() != this->_size)
    throw std::invalid_argument("size mismatch");
  this->pos_goal = pos;
}

template< class Kernel >
const typename Planner< Kernel >::Problem::Configuration&
Planner< Kernel >::Problem::get_pos_goal() const
{
  return this->pos_goal;
}

template< class Kernel >
bool Planner< Kernel >::Problem::valid_p() const
{
  bool flag0 = this->robots.size() == this->_size;
  bool flag1 = this->pos_init.size() == this->_size;
  bool flag2 = this->pos_goal.size() == this->_size;
  bool flag3 = true;
  for(size_t k = 0; k < this->_size; k++)
    if(this->robots[k].get_radius() == 0) {
      flag3 = false; break;
    }
  return flag0 and flag1 and flag2 and flag3;
}

template< class Kernel >
tinyxml2::XMLElement*
Planner< Kernel >::Problem::to_xml_element(tinyxml2::XMLDocument *doc,
                                           std::string elt_name) const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  auto xml_ws = this->ws.to_xml_element(doc, "workspace");
  xml_elt->InsertEndChild(xml_ws);
  for(size_t k = 0; k < this->_size; k++)
    {
      auto xml_robot = doc->NewElement("robot");
      xml_elt->InsertEndChild(xml_robot);
      xml_robot->SetAttribute("radius", this->robots[k].get_radius());
      auto xml_pos_init = doc->NewElement("pos-init");
      xml_robot->InsertEndChild(xml_pos_init);
      xml_pos_init->SetAttribute("x", this->pos_init[k].x());
      xml_pos_init->SetAttribute("y", this->pos_init[k].y());
      auto xml_pos_goal = doc->NewElement("pos-goal");
      xml_robot->InsertEndChild(xml_pos_goal);
      xml_pos_goal->SetAttribute("x", this->pos_goal[k].x());
      xml_pos_goal->SetAttribute("y", this->pos_goal[k].y());
    }
  return xml_elt;
}



} // mrnav::planner_6124e1a8685f
