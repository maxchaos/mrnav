#pragma once

#include <mrnav/planner_6124e1a8685f/simple_cell.hpp>
#include <cassert>

namespace mrnav::planner_6124e1a8685f
{

template< class BP >
Planner< BP >::SimpleCell::SimpleCell(
  SimpleSlicePtr slice,
  SimpleCellPtr parent,
  PolygonRegionSP shape,
  const std::vector< SimpleCellPtr > &adjacent)
  : SimpleCell{slice, parent, *shape, adjacent}
{}

template< class BP >
Planner< BP >::SimpleCell::SimpleCell(
  SimpleSlicePtr slice,
  SimpleCellPtr parent,
  const PolygonRegion &shape,
  const std::vector< SimpleCellPtr > &adjacent)
  : SimpleCell{slice, parent, shape.get_if(), adjacent}
{}

template< class BP >
Planner< BP >::SimpleCell::SimpleCell(
  SimpleSlicePtr slice,
  SimpleCellPtr parent,
  const PolygonRegionIF &shape,
  const std::vector< SimpleCellPtr > &adjacent)
  : _self{this}
  , _slice{slice}
  , _parent{parent}
  , _expanded_p{false}
  , _children{}
  , _adjacent{}
  , _shape{shape}
  , _rfpoa{}
  , _rfpua{}
{
  // Include the parent to the list of adjacent cells.
  // if(parent != nullptr)
  //   this->_adjacent.push_back(this->_parent);
  // Extract and register adjacent simple cells from the given set.
  this->_filter_and_register_adjacent(adjacent);
  // Computer the over-approximation first and the under-approximation second
  // because // the implementation of the second depends on
  // the result of the first (see this->_compute_rfpua).
  this->_compute_rfpoa();
  this->_compute_rfpua();
}

template< class BP >
Planner< BP >::SimpleCell::~SimpleCell()
{
  this->_self.invalidate();
}



template< class BP >
void
Planner< BP >::SimpleCell::_filter_and_register_adjacent(
  const std::vector< SimpleCellPtr > &adjacent)
{
  // Reserve enough space in adjacency list.
  this->_adjacent.reserve(this->_adjacent.size() + adjacent.size());
  // Iterate over possibly adjacent cells.
  for(auto sc: adjacent) {
    // If this is me, move on.
    if(sc.get() == this)
      continue;
    // Check if this candidate is already registered as adjacent.
    // If true, there is nothing to do.
    if(this->adjacent_p(sc)) {
      // Assert that adjacency is trully symmetric.
      assert(sc->adjacent_p(this->_self));
      // Move on.
      continue;
    }
    // Assert that adjacency is trully symmetric.
    assert(not sc->adjacent_p(this->_self));
    // Check if their shapes intersect.
    // If they do, these cells are trully adjacent.
    if(this->_shape.intersects_p(sc->_shape))
      {
        // Update this object's adjacency list.
        this->_adjacent.push_back(sc);
        // Update other object's adjacency list.
        sc->_adjacent.push_back(this->_self);
      }
  }
}

template< class BP >
bool
Planner< BP >::SimpleCell::adjacent_p(SimpleCellSP cand) const
{
  return this->adjacent_p(cand.get());
}

template< class BP >
bool
Planner< BP >::SimpleCell::adjacent_p(SimpleCellPtr cand) const
{
  // for(auto sc: this->_adjacent) {
  //   if(sc.get() == cand.get())
  //     return true;
  // }
  // return false;
  return this->adjacent_p(cand.get());
}

template< class BP >
bool
Planner< BP >::SimpleCell::adjacent_p(SimpleCell const *cand) const
{
  // Of course, a sell is adjacent to itself by definition.
  if(cand == this)
    return true;
  // Iterate over the register of adjacent shells.
  for(auto sc: this->_adjacent) {
    if(sc.get() == cand)
      return true;
  }
  // If no match was found, then it is not adjacent.
  return false;
}

template< class BP >
bool
Planner< BP >::SimpleCell::contains_p(const Point &p) const
{
  return this->_shape.contains_p(p);
}



template< class BP >
void
Planner< BP >::SimpleCell::_compute_rfpoa()
{
  this->_rfpoa =
    *(this->_shape.get_buffer(this->_slice->get_robot().get_radius()));
}

template< class BP >
void
Planner< BP >::SimpleCell::_compute_rfpua()
{
  // *Assumption*:
  // The footprint over-approximation has already been computed
  // before this method gets called.
  // This must be ensured in the constructor.
  this->_rfpua = this->_rfpoa;
  // Get corresponding robot.
  const auto &robot = this->_slice->get_robot();
  // Naive implementation
  for(size_t reg_idx = 0; reg_idx < this->_shape.size(); reg_idx++)
    {
      auto reg = this->_shape.get_region(reg_idx);
      // Iterate over outer boundary's vertices.
      auto outer = reg.get_outer();
      for(size_t vtx_idx = 0; vtx_idx < outer.size(); vtx_idx++)
        {
          this->_rfpua.intersect(robot.get_body(outer.get_vtx(vtx_idx)));
          if(this->_rfpua.empty_p())
            break;
        }
      // Iterate over inner boundaries.
      auto inners = reg.get_inners();
      for(size_t hole_idx = 0; hole_idx < inners.size(); hole_idx++)
        {
          auto hole = inners.get_ring(hole_idx);
          // Iterate over inner boundary's vertices.
          for(size_t vtx_idx = 0; vtx_idx < hole.size(); vtx_idx++)
            {
              this->_rfpua.intersect(robot.get_body(hole.get_vtx(vtx_idx)));
              if(this->_rfpua.empty_p())
                break;
            }
          if(this->_rfpua.empty_p())
            break;
        }
    }
  // TODO: Implement a more intelligent method which exploits the fact that
  //       treating faraway vertices first may yield empty intersections early
  //       during computation.
}



template< class BP >
bool
Planner< BP >::SimpleCell::expanded_p() const
{
  return this->_expanded_p;
}

template< class BP >
void
Planner< BP >::SimpleCell::_expand()
{
  // Check if cell has been expanded already.
  if(this->expanded_p())
    return;
  // Check if the containing slice has been expanded, otherwise expand it now.
  if(not this->_slice->expanded_p())
    this->_slice->_expand();
  // Iterate over the sub-slices.
  for(auto ss: this->_slice->_children)
    {
      // Compute the partition's shape.
      PolygonRegionSetSP shape_new =
        this->_shape.get_intersection(ss->_rect_aug);
      // Extract distinct polygonal regions from the partition.
      for(size_t k = 0; k < shape_new->size(); k++)
        {
          // Assert that the polygon region is not empty.
          assert(not shape_new->get_region(k).empty_p());
          // Construct child.
          SimpleCellSP child{new SimpleCell{ss->_self,
                                            this->_self,
                                            shape_new->get_region(k),
                                            this->_adjacent}
          };
          // Register new cell to the corresponding slice.
          ss->_scells.push_back(child);
          // Register new child to this cell.
          this->_children.push_back(child);
        }
    }
  // Update adjacency graph by including potential
  // parent-child and inter-child adjacency relations:
  // 1. Build list of pointers.
  std::vector< SimpleCellPtr > relatives_list;
  relatives_list.push_back(this->_self);
  for(auto child: this->_children)
    relatives_list.push_back(child->_self);
  // 2. Pass the list of pointers to each child.
  for(auto child: this->_children)
    child->_filter_and_register_adjacent(relatives_list);
  // Mark this cell as expanded.
  this->_expanded_p = true;
}




template< class BP >
tinyxml2::XMLElement*
Planner< BP >::SimpleCell::to_xml_element(tinyxml2::XMLDocument *doc,
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
  xml_elt->SetAttribute("expanded", this->_expanded_p);
  auto xml_shape = this->_shape.to_xml_element(doc, "shape");
  xml_elt->InsertEndChild(xml_shape);
  auto xml_rfpoa = this->_rfpoa.to_xml_element(doc, "rfpoa");
  xml_elt->InsertEndChild(xml_rfpoa);
  auto xml_rfpua = this->_rfpua.to_xml_element(doc, "rfpua");
  xml_elt->InsertEndChild(xml_rfpua);
  tinyxml2::XMLElement *xml_children = doc->NewElement("children");
  xml_elt->InsertEndChild(xml_children);
  for(size_t k = 0; k < this->_children.size(); k++) {
    auto xml_child = doc->NewElement("cell");
    xml_child->SetAttribute(
      "ssid",
      std::to_string(reinterpret_cast< std::uintptr_t >(
                       this->_children[k].get()) ).c_str());
    xml_children->InsertEndChild(xml_child);
  }
  tinyxml2::XMLElement *xml_adjacent = doc->NewElement("adjacent");
  xml_elt->InsertEndChild(xml_adjacent);
  for(size_t k = 0; k < this->_adjacent.size(); k++) {
    auto xml_cell = doc->NewElement("cell");
    xml_cell->SetAttribute(
      "ssid",
      std::to_string(reinterpret_cast< std::uintptr_t >(
                       this->_adjacent[k].get()) ).c_str());
    xml_adjacent->InsertEndChild(xml_cell);
  }
  return xml_elt;
}



} // mrnav::planner_6124e1a8685f
