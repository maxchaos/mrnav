#pragma once

#include <mrnav/planner_6124e1a8685f/simple_slice_tree.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class BP >
Planner< BP >::SimpleSliceTree::SimpleSliceTree(
  PlannerPtr planner, size_t robot_idx)
  : _self{this}
  , _planner{planner}
  , _robot_idx{robot_idx}
  , _ws_aug{}
  , _root{}
{
  // Compute augmented workspace.
  this->_ws_aug = this->_planner->get_augmented_workspace(
    this->_planner->get_robot(this->_robot_idx).get_radius());
  // Get augmented workspace's envelope.
  // Rectangle env = this->_ws_aug->get_envelop();
  typename Planner::CoordNT x_min, x_max, y_min, y_max;
  this->_ws_aug->get_envelope(x_min, x_max, y_min, y_max);
  // Enlarge envelope a little to avoid numerical issues.
  Rectangle root_env{x_min, x_max, y_min, y_max};
  // Create root.
  this->_root = SimpleSliceSP{new SimpleSlice{this->_self, nullptr, root_env}};
  // Just for root, compute and assign its cells.
  // Notice that these cells are not adjacent.
  for(size_t idx = 0; idx < this->_ws_aug->size(); idx++) {
    this->_root->_scells.emplace_back(
      new SimpleCell{this->_root->_self, nullptr,
                       this->_ws_aug->get_region(idx), {}});
  }
}

template< class BP >
Planner< BP >::SimpleSliceTree::~SimpleSliceTree()
{
  this->_self.invalidate();
}

template< class BP >
const typename Planner< BP >::SimpleSliceTree::Robot&
Planner< BP >::SimpleSliceTree::get_robot() const
{
  return this->_planner->get_robot(this->_robot_idx);
}

template< class BP >
const typename Planner< BP >::SimpleSliceTree::PolygonRegionSet&
Planner< BP >::SimpleSliceTree::get_ws_aug() const
{
  return *(this->_ws_aug);
}



template< class BP >
tinyxml2::XMLElement*
Planner< BP >::SimpleSliceTree::to_xml_element(tinyxml2::XMLDocument *doc,
                                               std::string elt_name)
  const
{
  // Create this element's representation.
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  // Set robot's index.
  xml_elt->SetAttribute("robot_idx",
                        std::to_string(this->_robot_idx).c_str());
  // Set planner.
  tinyxml2::XMLElement *xml_planner = doc->NewElement("planner");
  xml_planner->SetAttribute(
    "id",
    std::to_string(reinterpret_cast< std::uintptr_t >(_planner.get())).c_str());
  xml_elt->InsertEndChild(xml_planner);
  // Set robot's configuration space.
  xml_elt->InsertEndChild(this->_ws_aug->to_xml_element(doc, "cs"));
  // Set root slice.
  tinyxml2::XMLElement *xml_root = this->_root->to_xml_element(doc);
  xml_elt->InsertEndChild(xml_root);
  // Return this element's representation.
  return xml_elt;
}



} // mrnav::planner_6124e1a8685f
