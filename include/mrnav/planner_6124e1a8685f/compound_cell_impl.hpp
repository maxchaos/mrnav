#pragma once

#include <mrnav/planner_6124e1a8685f/compound_cell.hpp>
#include <cassert>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
Planner< Kernel >::CompoundCell::CompoundCell(
  PlannerPtr planner,
  CCComponents components,
  CompoundCellPtr parent)
  : _self{this}
  , _planner{planner}
  , _components{components}
  , _amount_of_robots{_components.size()}
  , _id{}
  , _label{}
  , _oa_conflict_p{}
  , _ua_conflict_p{}
  , _expanded_p{false}
  , _parent{parent}
  , _expansion{}
  , _heuristic{}
{
  // Assert that the amount or robots and the amount of components much.
  assert(_components.size() == _amount_of_robots);
  // For some reason,
  // the default constructor does not interpret _amount_of_robots as
  // the desired container's size but as it's initial value, so
  // an explicit call to resize is necessary.
  _id.resize(_amount_of_robots);
  assert(_id.size() == _amount_of_robots);
  // Compute this compound cell's id.
  for(size_t k = 0; k < this->_amount_of_robots; k++) {
    this->_id[k] = reinterpret_cast< uintptr_t >(this->_components[k].get());
  }
  // Compute this compound cell's label.
  this->_compute_label();
}

template< class Kernel >
Planner< Kernel >::CompoundCell::~CompoundCell()
{
  this->_self.invalidate();
}

template< class Kernel >
void
Planner< Kernel >::CompoundCell::_compute_label()
{
  // Initially assume that no conflicts between under-approximations exist.
  this->_ua_conflict_p.clear();
  this->_ua_conflict_p.resize(this->_amount_of_robots, false);
  // Check for conflicts between under-approximations.
  // No need to find every conflict since:
  // * just one is enough to mark a cell inadmissible, and
  // * an inandmissible cell will get expanded no more.
  for(size_t i = 0; i < this->_amount_of_robots; i++) {
    const PolygonRegionSet &si = this->_components[i]->_rfpua;
    for(size_t j = i+1; j < this->_amount_of_robots; j++) {
      const PolygonRegionSet &sj = this->_components[j]->_rfpua;
      bool res = si.intersects_p(sj);
      if(res)
        {
          this->_ua_conflict_p[i] = true;
          this->_ua_conflict_p[j] = true;
          this->_label = CCLabel::INADMISSIBLE;
          // No need to check further; this cell is rotten.
          return;
        }
    }
  }
  // Initially assume that no conflicts between over-approximations exist.
  bool oa_conflicts_exist_p = false;
  this->_oa_conflict_p.clear();
  this->_oa_conflict_p.resize(this->_amount_of_robots, false);
  // Check for conflicts between over-approximations.
  // All conflicts must be found because they are used during
  // the expansion of obscure cells.
  for(size_t i = 0; i < this->_amount_of_robots; i++) {
    // Get reference to over-approximation of robot i.
    const PolygonRegionSet &si = this->_components[i]->_rfpoa;
    // Iterate over all remaining robots.
    for(size_t j = i+1; j < this->_amount_of_robots; j++) {
      // Get reference to over-approximation of robot j.
      const PolygonRegionSet &sj = this->_components[j]->_rfpoa;
      // If both simple cells been marked as conflicting (independently of
      // whether they conflict with each other), no need to run another test.
      if(this->_oa_conflict_p[i] and this->_oa_conflict_p[j])
        continue;
      // Check for conflict;
      bool res = si.intersects_p(sj);
      if(res)
        {
          this->_oa_conflict_p[i] = true;
          this->_oa_conflict_p[j] = true;
          oa_conflicts_exist_p = true;
        }
    }
  }
  // If at least one pair of over-approximations conflict with another,
  // then mark this cell as obscure, otherwise mark it as admissible.
  if(oa_conflicts_exist_p)
    this->_label = CCLabel::OBSCURE;
  else
    this->_label = CCLabel::ADMISSIBLE;
}

template< class Kernel >
const CCID&
Planner< Kernel >::CompoundCell::get_id() const
{
  return this->_id;
}

template< class Kernel >
const typename Planner< Kernel >::CompoundCell::CCLabel&
Planner< Kernel >::CompoundCell::get_label() const
{
  return this->_label;
}



template< class Kernel >
bool
Planner< Kernel >::CompoundCell::admissible_p() const
{
  return this->_label == CCLabel::ADMISSIBLE;
}

template< class Kernel >
bool
Planner< Kernel >::CompoundCell::obscure_p() const
{
  return this->_label == CCLabel::OBSCURE;
}

template< class Kernel >
bool
Planner< Kernel >::CompoundCell::inadmissible_p() const
{
  return this->_label == CCLabel::INADMISSIBLE;
}

template< class Kernel >
bool
Planner< Kernel >::CompoundCell::expanded_p()
{
  return this->_expanded_p;
}

template< class Kernel >
bool
Planner< Kernel >::CompoundCell::contains_p(const Configuration &p)
  const
{
  // Make sure that the configuration has the right dimension.
  if(p.size() != this->_amount_of_robots)
    throw std::invalid_argument(
      "sizes of configuration and compound cell differ");
  // Check whether each corresponding simple slice contains the
  // corresponding component of the given configuration.
  for(size_t k = 0; k < this->_amount_of_robots; k++)
    if(not this->_components[k]->contains_p(p[k]))
      return false;
  return true;
}

template< class Kernel >
bool
Planner< Kernel >::CompoundCell::adjacent_p(CompoundCellSP o)
  const
{
  // Make sure that the two cells have the same planner.
  if(this->_planner != o->_planner)
    throw std::invalid_argument("planner mismatch");
  // Make sure that the two cells have the same size.
  if(this->_amount_of_robots != o->_amount_of_robots)
    throw std::invalid_argument("size mismatch");
  // Check if given cell is adjacent tot this one.
  for(size_t k = 0; k < this->_amount_of_robots; k++)
    if(not this->_components[k]->adjacent_p(o->_components[k]))
      return false;
  return true;
}



template< class Kernel >
void
Planner< Kernel >::CompoundCell::get_conflicting_oascells(
  std::vector< size_t > &indices)
{
  // Clear vector.
  indices.clear();
  // Handle admissible case.
  if(this->admissible_p()) {
    // Return nothing.
    return;
  }
  // Handle obscure case.
  if(this->obscure_p()) {
    // Assert that the number of conflict flags is valid.
    assert(this->_oa_conflict_p.size() == this->_amount_of_robots);
    // Return indices of components which have been marked as conflicting.
    for(size_t k = 0; k < this->_amount_of_robots; k++)
      if(this->_oa_conflict_p[k])
        indices.push_back(k);
  }
  // Handle inadmissible case.
  if(this->inadmissible_p()) {
    throw std::runtime_error("feature not implemented for inadmissible cells");
  }
}



template< class Kernel >
tinyxml2::XMLElement*
Planner< Kernel >::CompoundCell::to_xml_element(
  tinyxml2::XMLDocument *doc,
  std::string elt_name)
  const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  // Write id.
  xml_elt->SetAttribute(
    "id",
    std::to_string(reinterpret_cast<uintptr_t>(this)).c_str());
  // Write label.
  if(this->admissible_p())
    xml_elt->SetAttribute("label", "admissible");
  else if(this->obscure_p())
    xml_elt->SetAttribute("label", "obscure");
  else
    xml_elt->SetAttribute("label", "inadmssible");
  xml_elt->SetAttribute(
    "parent",
    std::to_string(reinterpret_cast<uintptr_t>(this->_parent.get())).c_str());
  // Write components.
  auto xml_cmps = doc->NewElement("components");
  xml_elt->InsertEndChild(xml_cmps);
  for(size_t k = 0; k < this->_amount_of_robots; k++) {
    auto xml_cmp = doc->NewElement("simple-cell");
    xml_cmp->SetAttribute("refid", std::to_string(this->_id[k]).c_str());
    xml_cmps->InsertEndChild(xml_cmp);
  }
  // Write expansion.
  auto xml_expansion = doc->NewElement("expansion");
  xml_elt->InsertEndChild(xml_expansion);
  for(auto ex: this->_expansion) {
    auto xml_ex = doc->NewElement("compound-cell");
    xml_ex->SetAttribute(
      "refid",
      std::to_string(reinterpret_cast<uintptr_t>(ex.get())).c_str());
    xml_expansion->InsertEndChild(xml_ex);
  }
  return xml_elt;
}



} // mrnav::planner_6124e1a8685f
