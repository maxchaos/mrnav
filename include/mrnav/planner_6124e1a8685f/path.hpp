#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>

#include <vector>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
class Planner< Kernel >::Path
{
public:
  using Planner_ = Planner< Kernel >;

  using CompoundCell = typename Planner_::CompoundCell;
  using CompoundCellSP = typename Planner_::CompoundCellSP;

  using CCString = typename Planner_::CCString;
  using CCLabel = typename Planner_::CCLabel;

private:
  using CompoundCellPtr = CompoundCell*;

private:
  CCString _str;
  CCLabel _label;

public:
  Path(const CCString &str);
  Path(const CCString &prefix, const CCString &suffix);
  Path(const CCString &prefix, CompoundCellSP infix, const CCString &suffix);

private:
  void _compute_label();

public:
  void split_at_ccell(CompoundCellSP ccell,
                      CCString &prefix, CCString &suffix);
  CompoundCellSP get_first_occell();

  bool admissible_p() const;
  bool obscure_p() const;
  bool inadmissible_p() const;

public:
  friend Planner_;

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "path") const;


};

} // mrnav::planner_6124e1a8685f
