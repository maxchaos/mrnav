#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>
#include <mrnav/planner_6124e1a8685f/aux/stateful_ptr.hpp>

namespace mrnav::planner_6124e1a8685f
{

template< class Kernel >
class Planner< Kernel >::CompoundCell
{
public:
  using Planner_ = Planner< Kernel >;
  using PlannerPtr = typename Planner_::PlannerPtr;
  using SimpleCell = typename Planner_::SimpleCell;
  using SimpleCellSP = typename Planner_::SimpleCellSP;
  using CCComponents = typename Planner_::CCComponents;
  using CCLabel = typename Planner_::CCLabel;
  using PolygonRegionSet = typename Planner_::PolygonRegionSet;

public:
  using CompoundCellPtr = stateful_ptr< CompoundCell >;
  using CompoundCellSP = typename Planner_::CompoundCellSP;

private:
  CompoundCellPtr _self;

private:
  const PlannerPtr _planner;
  const CCComponents _components;
  const size_t _amount_of_robots;
  CCID _id;
  // std::vector< CompoundCellPtr > _adjacent;

  CCLabel _label;
  std::vector< bool > _oa_conflict_p;
  std::vector< bool > _ua_conflict_p;

  bool _expanded_p;
  CompoundCellPtr _parent;
  std::vector< CompoundCellSP > _expansion;

  double _heuristic;

private:
  // CompoundCell(PlannerPtr planner,
  //              CCComponents components,
  //              std::vector< CompoundCellPtr > adjacent);
  CompoundCell(PlannerPtr planner,
               CCComponents components,
               CompoundCellPtr parent = nullptr);

public:
  ~CompoundCell();

private:
  void _compute_label();

public:
  const CCID& get_id() const;
  const CCLabel& get_label() const;
  void get_conflicting_oascells(std::vector< size_t > &indices);

public:
  bool admissible_p() const;
  bool obscure_p() const;
  bool inadmissible_p() const;

  bool expanded_p();
  bool contains_p(const Configuration &p) const;
  bool adjacent_p(CompoundCellSP other) const;

public:
  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "compound-cell") const;

public:
  friend Planner_;

};

} // mrnav::planner_6124e1a8685f
