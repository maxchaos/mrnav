#pragma once

#include <mrnav/planner_6124e1a8685f/simple_slice.hpp>
#include <cmath>

namespace mrnav::planner_6124e1a8685f
{

template< class BP >
Planner< BP >::SimpleSlice::SimpleSlice(SimpleSliceTreePtr tree,
                                        SimpleSlicePtr parent,
                                        Rectangle rectangle)
  : _self{this}
  , _tree{tree}
  , _parent{parent}
  // , _x_min{envelope[0]}, _x_max{envelope[1]}
  // , _y_min{envelope[2]}, _y_max{envelope[3]}
  , _rect_ideal{rectangle}
  , _rect_aug{}
  , _expanded_p{false}
  , _children{}
  , _scells{}
{
  assert(this->_expanded_p == false);
  assert(this->_children.size() == 4);
  assert(this->_scells.size() == 0);
  CoordNT xmin = this->_rect_ideal.get_xmin();
  CoordNT xmax = this->_rect_ideal.get_xmax();
  CoordNT ymin = this->_rect_ideal.get_ymin();
  CoordNT ymax = this->_rect_ideal.get_ymax();
  double saf = this->_tree->_planner->_parameters.slice_augmentation_factor;
  DistNT dx = (xmax - xmin) * saf;
  DistNT dy = (ymax - ymin) * saf;
  this->_rect_aug =
    Rectangle{xmin - 0.5*dx, xmax + 0.5*dx, ymin - 0.5*dy, ymax + 0.5*dy};
}

template< class BP >
Planner< BP >::SimpleSlice::~SimpleSlice()
{
  this->_self.invalidate();
}


template< class BP >
typename Planner< BP >::SimpleSlice::DistNT
Planner< BP >::SimpleSlice::get_width() const
{
  return fabs(this->_rect_ideal.get_xmax() - this->_rect_ideal.get_xmin());
}

template< class BP >
typename Planner< BP >::SimpleSlice::DistNT
Planner< BP >::SimpleSlice::get_height() const
{
  return fabs(this->_rect_ideal.get_ymax() - this->_rect_ideal.get_ymin());
}

template< class BP >
typename Planner< BP >::SimpleSlice::DistNT
Planner< BP >::SimpleSlice::get_largest_dimension() const
{
  return std::max(this->get_width(), this->get_height());
}

template< class BP >
typename Planner< BP >::SimpleSlice::DistNT
Planner< BP >::SimpleSlice::get_smallest_dimension() const
{
  return std::min(this->get_width(), this->get_height());
}

template< class BP >
typename Planner< BP >::SimpleSlice::DistNT
Planner< BP >::SimpleSlice::get_area() const
{
  return this->get_width() * this->get_height();
}

template< class BP >
void
Planner< BP >::SimpleSlice::_expand()
{
  // If this has been expanded already, do nothing.
  if(this->expanded_p())
    return;
  CoordNT xmin = this->_rect_ideal.get_xmin();
  CoordNT xmax = this->_rect_ideal.get_xmax();
  CoordNT ymin = this->_rect_ideal.get_ymin();
  CoordNT ymax = this->_rect_ideal.get_ymax();
  DistNT dx = xmax - xmin;
  DistNT dy = ymax - ymin;
  CoordNT xm = xmin + dx/2;
  CoordNT ym = ymin + dy/2;
  // Top-left:
  this->_children[0] = SimpleSliceSP{ new SimpleSlice(
      this->_tree, this->_self,
      Rectangle{xmin, xm, ym, ymax})
  };
  // Top-right:
  this->_children[1] = SimpleSliceSP{ new SimpleSlice(
      this->_tree, this->_self,
      Rectangle{xm, xmax, ym, ymax})
  };
  // Bottom-right:
  this->_children[2] = SimpleSliceSP{ new SimpleSlice(
      this->_tree, this->_self,
      Rectangle{xm, xmax, ymin, ym})
  };
  // Bottom-left:
  this->_children[3] = SimpleSliceSP{ new SimpleSlice(
      this->_tree, this->_self,
      Rectangle{xmin, xm, ymin, ym})
  };
  // Mark slice as expanded.
  this->_expanded_p = true;
}

template< class BP >
bool
Planner< BP >::SimpleSlice::expanded_p() const
{
  return this->_expanded_p;
}

template< class BP >
const typename Planner< BP >::SimpleSlice::Robot&
Planner< BP >::SimpleSlice::get_robot() const
{
  return this->_tree->get_robot();
}



template< class BP >
tinyxml2::XMLElement*
Planner< BP >::SimpleSlice::to_xml_element(tinyxml2::XMLDocument *doc,
                                           std::string elt_name)
  const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  xml_elt->SetAttribute(
    "id",
    std::to_string(reinterpret_cast< std::uintptr_t >(this) ).c_str());
  xml_elt->SetAttribute(
    "parent_id",
    std::to_string(reinterpret_cast< std::uintptr_t >(this->_parent.get()) ).c_str());
  tinyxml2::XMLElement *xml_rect = doc->NewElement("rectangle");
  xml_elt->InsertEndChild(xml_rect);
  xml_rect->SetAttribute("xmin", this->_rect_ideal.get_xmin());
  xml_rect->SetAttribute("xmax", this->_rect_ideal.get_xmax());
  xml_rect->SetAttribute("ymin", this->_rect_ideal.get_ymin());
  xml_rect->SetAttribute("ymax", this->_rect_ideal.get_ymax());
  tinyxml2::XMLElement *xml_rect_aug = doc->NewElement("rectangle-augmented");
  xml_elt->InsertEndChild(xml_rect_aug);
  xml_rect_aug->SetAttribute("xmin", this->_rect_aug.get_xmin());
  xml_rect_aug->SetAttribute("xmax", this->_rect_aug.get_xmax());
  xml_rect_aug->SetAttribute("ymin", this->_rect_aug.get_ymin());
  xml_rect_aug->SetAttribute("ymax", this->_rect_aug.get_ymax());
  tinyxml2::XMLElement *xml_cells = doc->NewElement("cells");
  xml_elt->InsertEndChild(xml_cells);
  for(size_t k = 0; k < this->_scells.size(); k++) {
    auto xml_cell = this->_scells[k]->to_xml_element(doc);
    xml_cells->InsertEndChild(xml_cell);
  }
  tinyxml2::XMLElement *xml_children = doc->NewElement("children");
  xml_elt->InsertEndChild(xml_children);
  if(this->expanded_p()) {
    for(size_t k = 0; k < this->_children.size(); k++) {
      auto xml_child = this->_children[k]->to_xml_element(doc);
      xml_children->InsertEndChild(xml_child);
    }
  }
  return xml_elt;
}


} // mrnav::planner_6124e1a8685f
