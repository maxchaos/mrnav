#pragma once

#include <mrnav/planner_6124e1a8685f/extended_path.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
Planner< Kernel >::ExtendedPath::ExtendedPath()
  : _str{}
  , _label{CCLabel::ADMISSIBLE}
{}

template< typename Kernel >
void
Planner< Kernel >::ExtendedPath::_compute_label()
{
  this->_label = CCLabel::ADMISSIBLE;
  for(auto lvl: this->_str) {
    // If the current level is empty, then mark path as inadmissible.
    // TODO: Devise another label to indicate empty levels.
    if(lvl.empty()) {
      this->_label = CCLabel::INADMISSIBLE;
      return;
    }
    // Find the first non-admissible cell and copy its label.
    // Assumes that a path does not contain inadmissible cells.
    auto cc = lvl.front();
    assert(cc != nullptr);
    if(not cc->admissible_p()) {
      this->_label = cc->get_label();
      break;
    }
  }
}



template< typename Kernel >
typename Planner< Kernel >::ExtendedPath::CompoundCellSP
Planner< Kernel >::ExtendedPath::_get_first_active_occell()
{
  assert(not this->admissible_p());
  for(auto lvl: this->_str) {
    assert(not lvl.empty());
    if(lvl.front()->obscure_p())
      return lvl.front();
  }
  return nullptr;
}


template< typename Kernel >
typename Planner< Kernel >::ExtendedPath::LevelList::iterator
Planner< Kernel >::ExtendedPath::_get_first_active_occell_lvl()
{
  // assert(not this->admissible_p());
  auto it = this->_str.begin();
  for(; it != this->_str.end(); it++) {
    assert(not it->empty());
    assert(not it->front()->inadmissible_p());
    if(it->front()->obscure_p())
      return it;
  }
  return this->_str.end();
}

template< typename Kernel >
void
Planner< Kernel >::ExtendedPath::_replace_active(
  typename LevelList::iterator lvl,
  const std::vector< CompoundCellSP > &cells)
{
  assert(not lvl->empty());
  lvl->pop_front();
  for(auto it = cells.rbegin(); it != cells.rend(); it++) {
    lvl->push_front(*it);
  }
  this->_compute_label();
}

template< typename Kernel >
void
Planner< Kernel >::ExtendedPath::_make_active(typename LevelList::iterator lvl,
                                              CompoundCellSP cell)
{
  assert(lvl != this->_str.end());
  lvl->remove(cell);
  lvl->push_front(cell);
  this->_compute_label();
}



template< class Kernel >
size_t
Planner< Kernel >::ExtendedPath::length() const
{
  return this->_str.size();
}

template< class Kernel >
size_t
Planner< Kernel >::ExtendedPath::engaged_cells_amount() const
{
  size_t res = 0;
  for(auto lvl: this->_str)
    res += lvl.size();
  return res;
}



template< class Kernel >
void
Planner< Kernel >::ExtendedPath::push_front_lvl(CompoundCellSP cell)
{
  Level lvl_new;
  assert(cell != nullptr);
  lvl_new.push_front(cell);
  this->push_front_lvl(lvl_new);
}

template< class Kernel >
void
Planner< Kernel >::ExtendedPath::push_front_lvl(const Level &lvl)
{
  this->_str.emplace_front(lvl.cbegin(), lvl.cend());
  this->_compute_label();
}

template< class Kernel >
void
Planner< Kernel >::ExtendedPath::push_back_lvl(CompoundCellSP cell)
{
  Level lvl_new;
  assert(cell != nullptr);
  lvl_new.push_back(cell);
  this->push_back_lvl(lvl_new);
}

template< class Kernel >
void
Planner< Kernel >::ExtendedPath::push_back_lvl(const Level &lvl)
{
  this->_str.emplace_back(lvl.cbegin(), lvl.cend());
  this->_compute_label();
}

template< class Kernel >
// typename Planner< Kernel >::ExtendedPath::Level&
typename Planner< Kernel >::ExtendedPath::LevelList::iterator
Planner< Kernel >::ExtendedPath::front_lvl()
{
  // return this->_str.front();
  return this->_str.begin();
}

template< class Kernel >
// typename Planner< Kernel >::ExtendedPath::Level&
typename Planner< Kernel >::ExtendedPath::LevelList::iterator
Planner< Kernel >::ExtendedPath::back_lvl()
{
  // return this->_str.back();
  auto it = this->_str.end();
  if(this->_str.empty())
    return it;
  return --it;
}

template< class Kernel >
void
Planner< Kernel >::ExtendedPath::pop_last_lvl()
{
  assert(not this->_str.empty());
  this->_str.pop_back();
  this->_compute_label();
}

template< class Kernel >
typename Planner< Kernel >::ExtendedPath::CompoundCellSP
Planner< Kernel >::ExtendedPath::pop_last_active()
{
  if(not this->_str.empty())
    {
      auto lvl_end = this->_str.back();
      if(not lvl_end.empty()) {
        auto cc = lvl_end.front();
        lvl_end.pop_front();
        this->_compute_label();
        return cc;
      }
    }
  return nullptr;
}

template< class Kernel >
typename Planner< Kernel >::ExtendedPath::CompoundCellSP
Planner< Kernel >::ExtendedPath::front_active()
{
  if(this->_str.empty())
    return nullptr;
  if(this->_str.front().empty())
    return nullptr;
  return this->_str.front().front();
}

template< class Kernel >
typename Planner< Kernel >::ExtendedPath::CompoundCellSP
Planner< Kernel >::ExtendedPath::back_active()
{
  if(this->_str.empty())
    return nullptr;
  if(this->_str.back().empty())
    return nullptr;
  return this->_str.back().front();
}


// template< class Kernel >
// void
// Planner< Kernel >::ExtendedPath::split_at(CompoundCellSP cell,
//                                           ExtendedPathSP &prefix,
//                                           ExtendedPathSP &suffix)
//   const
// {
//   // Allocate new prefix and suffix.
//   prefix = ExtendedPathSP{ new ExtendedPath{} };
//   suffix = ExtendedPathSP{ new ExtendedPath{} };
//   // Find level containing active cell.
//   auto it_lvl = this->_str.begin();
//   for(; it_lvl != this->_str.end(); it_lvl++) {
//     if(it_lvl->front() == cell)
//       break;
//   }
//   // If no such level exists, raise an error.
//   if(it_lvl == this->_str.end())
//     throw std::invalid_argument("active cell not found");
//   // Otherwise, split this path at the specified position.
//   // Note that, since the prefix must include the given active cell,
//   // we must increment the iterator.
//   it_lvl++;
//   prefix->_str.clear();
//   prefix->_str.insert(prefix->_str.begin(), this->_str.begin(), it_lvl);
//   prefix->_compute_label();
//   suffix->_str.clear();
//   suffix->_str.insert(suffix->_str.begin(), it_lvl, this->_str.end());
//   suffix->_compute_label();
// }

template< class Kernel >
void
Planner< Kernel >::ExtendedPath::split_at(CompoundCellSP cell,
                                          ExtendedPathSP &prefix,
                                          ExtendedPathSP &suffix)
  const
{
  // Find level containing active cell.
  auto it_lvl = this->_str.begin();
  for(; it_lvl != this->_str.end(); it_lvl++) {
    if((not it_lvl->empty()) and (it_lvl->front() == cell))
      break;
  }
  // If no such level exists, raise an error.
  if(it_lvl == this->_str.end())
    throw std::invalid_argument("active cell not found");
  // Split path at critical level.
  this->split_at(it_lvl, prefix, suffix);
}

template< class Kernel >
void
Planner< Kernel >::ExtendedPath::split_at(
  typename LevelList::const_iterator it_lvl,
  ExtendedPathSP &prefix,
  ExtendedPathSP &suffix)
  const
{
  // Allocate new prefix and suffix.
  prefix = ExtendedPathSP{ new ExtendedPath{} };
  suffix = ExtendedPathSP{ new ExtendedPath{} };
  // Split this path at the specified position.
  // Note that, since the prefix must include the given active cell,
  // we must increment the iterator.
  if(it_lvl != this->_str.end())
    it_lvl++;
  prefix->_str.clear();
  prefix->_str.insert(prefix->_str.begin(), this->_str.begin(), it_lvl);
  prefix->_compute_label();
  suffix->_str.clear();
  suffix->_str.insert(suffix->_str.begin(), it_lvl, this->_str.end());
  suffix->_compute_label();
}



template< class Kernel >
bool
Planner< Kernel >::ExtendedPath::admissible_p() const
{
  return this->_label == CCLabel::ADMISSIBLE;
}

template< class Kernel >
bool
Planner< Kernel >::ExtendedPath::obscure_p() const
{
  return this->_label == CCLabel::OBSCURE;
}

template< class Kernel >
bool
Planner< Kernel >::ExtendedPath::inadmissible_p() const
{
  return this->_label == CCLabel::INADMISSIBLE;
}

template< class Kernel >
bool
Planner< Kernel >::ExtendedPath::prefix_admissible_p(
  typename LevelList::iterator lvl)
  const
{
  for(auto it = this->_str.begin(); it != lvl; it++)
    {
      if(it->empty())
        return false;
      auto cc = it->front();
      if(not cc->admissible_p())
        return false;
    }
  return true;
}



template< class Kernel >
bool
Planner< Kernel >::ExtendedPath::contains_cell_p(CompoundCellSP cell)
  const
{
  for(auto lvl: this->_str)
    for(auto cc: lvl)
      if(cell == cc)
        return true;
  return false;
}

template< class Kernel >
bool
Planner< Kernel >::ExtendedPath::contains_active_cell_p(CompoundCellSP cell)
  const
{
  for(auto lvl: this->_str)
    if(not lvl.empty())
      if(cell == lvl.front())
        return true;
  return false;
}



template< class Kernel >
typename Planner< Kernel >::ExtendedPath::SimplePathSP
Planner< Kernel >::ExtendedPath::to_simple_path()
  const
{
  CCString str{};
  str.reserve(this->length());
  // Iterate over this path's levels and assemble active cells.
  for(auto lvl: this->_str) {
    str.push_back(lvl.front());
  }
  // Return result.
  return SimplePathSP{ new SimplePath{str} };
}



template< class Kernel >
tinyxml2::XMLElement*
Planner< Kernel >::ExtendedPath::to_xml_element(tinyxml2::XMLDocument *doc,
                                        std::string elt_name)
  const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  xml_elt->SetAttribute("length", std::to_string(this->_str.size()).c_str());
  if(this->admissible_p())
    xml_elt->SetAttribute("label", "admissible");
  else if(this->obscure_p())
    xml_elt->SetAttribute("label", "obscure");
  else
    xml_elt->SetAttribute("label", "inadmssible");
  for(auto lvl: this->_str)
    {
      auto xml_lvl = doc->NewElement("level");
      for(auto cc: lvl) {
        auto xml_cc = doc->NewElement("cell");
        xml_cc->SetAttribute(
          "refid",
          std::to_string( reinterpret_cast< uintptr_t >(cc.get()) ).c_str()
        );
        xml_lvl->InsertEndChild(xml_cc);
      }
      xml_elt->InsertEndChild(xml_lvl);
    }
  return xml_elt;
}



} // mrnav::planner_6124e1a8685f
