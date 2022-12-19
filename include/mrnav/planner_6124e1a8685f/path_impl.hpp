#pragma once

#include <mrnav/planner_6124e1a8685f/path.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< typename Kernel >
Planner< Kernel >::Path::Path(const CCString &str)
  : _str{str.begin(), str.end()}
  , _label{}
{
  // Compute label.
  this->_compute_label();
}

template< typename Kernel >
Planner< Kernel >::Path::Path(const CCString &prefix,
                          CompoundCellSP infix,
                          const CCString &suffix)
  : _str{}
  , _label{}
{
  // Build string.
  _str.reserve(prefix.size() + suffix.size() + 1);
  _str.insert(_str.end(), prefix.begin(), prefix.end());
  _str.push_back(infix);
  _str.insert(_str.end(), suffix.begin(), suffix.end());
  // Compute label.
  this->_compute_label();
}


template< typename Kernel >
Planner< Kernel >::Path::Path(const CCString &prefix,
                          const CCString &suffix)
  : _str{}
  , _label{}
{
  // Build string.
  _str.reserve(prefix.size() + suffix.size());
  _str.insert(_str.end(), prefix.begin(), prefix.end());
  _str.insert(_str.end(), suffix.begin(), suffix.end());
  // Compute label.
  this->_compute_label();
}

template< typename Kernel >
void
Planner< Kernel >::Path::_compute_label()
{
  this->_label = CCLabel::ADMISSIBLE;
  for(auto cc: this->_str)
    // Find the first non-admissible cell and copy its label.
    // Assumes that a path does not contain inadmissible cells.
    if(not cc->admissible_p()) {
      this->_label = cc->get_label();
      break;
    }
}



template< typename Kernel >
void
Planner< Kernel >::Path::split_at_ccell(CompoundCellSP ccell,
                                    CCString &prefix,
                                    CCString &suffix)
{
  // Initialize suffix and prefix.
  prefix.clear();
  suffix.clear();
  // Find cell's location.
  typename CCString::const_iterator ccell_loc;
  for(auto it = this->_str.cbegin(); it != this->_str.cend(); it++)
    if(*it == ccell) {
      ccell_loc = it;
      break;
    }
  // Extract prefix and suffix (excluding specified ccell).
  prefix.assign(this->_str.cbegin(), ccell_loc);
  prefix.assign(++ccell_loc, this->_str.cend());
}

template< typename Kernel >
typename Planner< Kernel >::Path::CompoundCellSP
Planner< Kernel >::Path::get_first_occell()
{
  for(size_t k = 0; k < this->_str.size(); k++)
    if(this->_str[k]->obscure_p())
      return this->_str[k];
  return nullptr;
}



template< class Kernel >
bool
Planner< Kernel >::Path::admissible_p() const
{
  return this->_label == CCLabel::ADMISSIBLE;
}

template< class Kernel >
bool
Planner< Kernel >::Path::obscure_p() const
{
  return this->_label == CCLabel::OBSCURE;
}

template< class Kernel >
bool
Planner< Kernel >::Path::inadmissible_p() const
{
  return this->_label == CCLabel::INADMISSIBLE;
}



template< class Kernel >
tinyxml2::XMLElement*
Planner< Kernel >::Path::to_xml_element(tinyxml2::XMLDocument *doc,
                                        std::string elt_name)
  const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  xml_elt->SetAttribute("len", std::to_string(this->_str.size()).c_str());
  if(this->admissible_p())
    xml_elt->SetAttribute("label", "admissible");
  else if(this->obscure_p())
    xml_elt->SetAttribute("label", "obscure");
  else
    xml_elt->SetAttribute("label", "inadmssible");
  for(auto cc: this->_str)
    {
      auto xml_cc = doc->NewElement("compound-cell");
      xml_cc->SetAttribute(
        "refid",
        std::to_string( reinterpret_cast< uintptr_t >(cc.get()) ).c_str()
      );
      xml_elt->InsertEndChild(xml_cc);
    }
  return xml_elt;
}



} // mrnav::planner_6124e1a8685f
