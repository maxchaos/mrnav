#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>

#include <vector>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
class Planner< Kernel >::ExtendedPath
{
public:
  using Planner_ = Planner< Kernel >;

  using CompoundCell = typename Planner_::CompoundCell;
  using CompoundCellSP = typename Planner_::CompoundCellSP;

  using SimplePath = typename Planner_::Path;
  using SimplePathSP = typename Planner_::PathSP;

  using CCString = typename Planner_::CCString;
  using CCLabel = typename Planner_::CCLabel;

private:
  using CompoundCellPtr = CompoundCell*;

  using Level = std::list< CompoundCellSP >;
  using LevelList = std::list< Level >;

private:
  LevelList _str;
  CCLabel _label;

public:
  ExtendedPath();

private:
  void _compute_label();
  CompoundCellSP _get_first_active_occell();
  typename LevelList::iterator _get_first_active_occell_lvl();
  void _replace_active(typename LevelList::iterator lvl,
                       const std::vector< CompoundCellSP > &cells);
  void _make_active(typename LevelList::iterator lvl, CompoundCellSP cell);

public:
  size_t length() const;
  size_t engaged_cells_amount() const;

  void push_front_lvl(CompoundCellSP cell);
  void push_front_lvl(const Level &lvl);
  void push_back_lvl(CompoundCellSP cell);
  void push_back_lvl(const Level &lvl);

  // Level& front_lvl();
  // Level& back_lvl();
  typename LevelList::iterator front_lvl();
  typename LevelList::iterator back_lvl();

  void pop_last_lvl();
  CompoundCellSP pop_last_active();

  CompoundCellSP front_active();
  CompoundCellSP back_active();

  void split_at(CompoundCellSP cell,
                ExtendedPathSP &prefix,
                ExtendedPathSP &suffix) const;
  void split_at(typename LevelList::const_iterator lvl,
                 ExtendedPathSP &prefix,
                 ExtendedPathSP &suffix) const;


  bool admissible_p() const;
  bool obscure_p() const;
  bool inadmissible_p() const;

  bool prefix_admissible_p(typename LevelList::iterator lvl) const;

  bool contains_cell_p(CompoundCellSP cell) const;
  bool contains_active_cell_p(CompoundCellSP cell) const;

  SimplePathSP to_simple_path() const;

public:
  friend Planner_;

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "extended-path") const;


};

} // mrnav::planner_6124e1a8685f
