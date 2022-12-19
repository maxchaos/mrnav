#pragma once

#include <array>
#include <vector>
#include <list>
#include <optional>
#include <memory>

#include <cstdint>
#include <functional>

#include <unordered_map>
// #include <boost/container_hash/hash_fwd.hpp>
#include <boost/container_hash/hash.hpp>

#include <mrnav/planner_6124e1a8685f/aux/stateful_ptr.hpp>

#include <tinyxml2.h>



namespace std
{

template<> struct hash< std::vector< uintptr_t > >
{
  typedef std::vector< uintptr_t > argument_type;
  typedef std::size_t result_type;
  result_type operator()(argument_type const& s) const noexcept
  {
    return boost::hash_range(s.cbegin(), s.cend());
  }
};

} // std



namespace mrnav::planner_6124e1a8685f
{

using CCID = std::vector< uintptr_t >;
// size_t hash_value(CCID const& key) {
//   return boost::hash_range(key.cbegin(), key.cend());
// }



template< class Kernel_ >
class Planner
{

public:
  using Kernel = Kernel_;
  using GeometryKernel = typename Kernel::GeometryKernel;

public:
  using CoordNT = typename GeometryKernel::CoordNT;
  using DistNT = typename GeometryKernel::DistNT;
  using Point = typename GeometryKernel::Point;
  // using Polygon = typename GeometryKernel::Polygon;
  // using PolygonRegion = typename GeometryKernel::PolygonRegion;
  using PolygonRegionSet = typename GeometryKernel::PolygonRegionSet;
  using PolygonRegionSetSP = typename GeometryKernel::PolygonRegionSetSP;
  using PolygonRegionSetCSP = typename GeometryKernel::PolygonRegionSetCSP;

public:
  class PlanningParameters;

  class Problem;
  class Robot;
  class Configuration;
  class Path;
  class ExtendedPath;

  class SimpleSliceTree;
  class SimpleSlice;
  class SimpleCell;
  // class CompoundSlice;
  class CompoundCell;

public:
  using ProblemSP = std::shared_ptr< Problem >;
  using ProblemCSP = std::shared_ptr< const Problem >;
  using PathSP = std::shared_ptr< Path >;
  using PathCSP = std::shared_ptr< const Path >;
  using ExtendedPathSP = std::shared_ptr< ExtendedPath >;
  using ExtendedPathCSP = std::shared_ptr< const ExtendedPath >;

  using SimpleSliceTreeSP = std::shared_ptr< SimpleSliceTree >;
  using SimpleSliceSP = std::shared_ptr< SimpleSlice >;
  using SimpleCellSP = std::shared_ptr< SimpleCell >;
  using CompoundCellSP = std::shared_ptr< CompoundCell >;

public:
  // Auxiliary data types
  using Rectangle = typename GeometryKernel::Rectangle;
  // using CCID = std::vector< SimpleCell* >;
  using CCComponents = std::vector< SimpleCellSP >;
  using CCBuildingBlocks = std::vector< std::vector< SimpleCellSP > >;
  using CCString = std::vector< CompoundCellSP >;
  // class PlannerPtr;
  using PlannerPtr = stateful_ptr< Planner< Kernel > >;

public:
  enum class CCLabel
    {
     ADMISSIBLE,
     OBSCURE,
     INADMISSIBLE
    };

private:
  PlannerPtr _self;

private:
  // std::vector< Robot<GeometryKernel> > robots;
  // std::array< Robot, amount_of_robots > robots;
  // std::array< typename GeometryKernel::Point, amount_of_robots > pos_init;
  // std::array< typename GeometryKernel::Point, amount_of_robots > pos_goal;
  const Problem _problem;

  const PlanningParameters _parameters;

private:
  const size_t _amount_of_robots;
  std::vector< SimpleSliceTreeSP > _hierarchy;
  std::unordered_map< CCID,
                      CompoundCellSP,
                      // boost::hash< CCID > > _ccells;
                      std::hash< CCID > > _ccells;
  // std::list< CompoundCellSP > _ccells_frontier;
  std::unordered_map< CCID,
                      CompoundCellSP,
                      // boost::hash< CCID > > _ccells_frontier;
                      std::hash< CCID > > _ccells_frontier;
  CompoundCellSP _cc_init;
  CompoundCellSP _cc_goal;
  ExtendedPathSP _path;

  size_t _iteration_counter;

  bool _flag_hierarchy_initialized_p;
  bool _flag_ccells_initialized_p;
  bool _flag_frontier_initialized_p;
  bool _flag_ccinit_initialized_p;
  bool _flag_ccgoal_initialized_p;
  bool _flag_path_initialized_p;

public:
  Planner(const Problem &problem, const PlanningParameters &params);
  ~Planner();

public:
  size_t amount_of_robots() const;
  const Robot& get_robot(size_t idx) const;
  PolygonRegionSet get_robot_cs(size_t idx) const;

  PolygonRegionSetSP
  get_augmented_workspace(DistNT radius) const;

  size_t get_ccell_amount() const;
  size_t get_frontier_size() const;

public:
  void initialize();
  void initialize_hierarchy();
  void initialize_ccells();
  void initialize_ccells_frontier();
  void initialize_ccinit();
  void initialize_ccgoal();
  void initialize_path();

  PathSP find_apath();
  void find_apath_step();

private:
  CompoundCellSP _find_or_build_enclosing_accell(const Configuration &p);
  void _expand_path();
  CompoundCellSP _get_enclosing_ccell(const Configuration &p);
  CompoundCellSP _get_enclosing_ccell(
    const Configuration &p,
    const std::vector< CompoundCellSP > &set);
  void _connect_cell(CompoundCellSP cell);
  void _expand_occell(CompoundCellSP ccell,
                      std::vector< CompoundCellSP > &expansion);
  void _collect_ccell_unexpanded_expansions(
    CompoundCellSP ccell,
    std::list< CompoundCellSP > &expansion);
  void _expand_ccell_at_scell(CompoundCellSP ccell, size_t scell_idx,
                              std::vector< CompoundCellSP > &expansion);
  void _expand_scell(SimpleCellSP scell,
                     std::vector< SimpleCellSP > &expansion);
  void _find_or_build_ccells(const CCBuildingBlocks &bb,
                             std::vector< CompoundCellSP > &ccells);
  CompoundCellSP _find_or_build_ccell(const CCComponents &components);
  // void _sort_ccells(std::vector<CompoundCellSP> &ccells) const;
  // void _sort_ccells(std::list<CompoundCellSP> &ccells) const;
  template< class IT >
  void _sort_ccells(IT &ccells) const;
  void _compute_heuristic(CompoundCellSP cell);

public:
  tinyxml2::XMLElement*
  hierarchy_to_xml_element(tinyxml2::XMLDocument *doc,
                           std::string elt_name = "hierarchy") const;

  tinyxml2::XMLElement*
  path_to_xml_element(tinyxml2::XMLDocument *doc,
                      std::string elt_name = "paths") const;

  tinyxml2::XMLElement*
  ccells_to_xml_element(tinyxml2::XMLDocument *doc,
                        std::string elt_name = "compound-cells") const;

  tinyxml2::XMLElement*
  to_xml_element(tinyxml2::XMLDocument *doc,
                 std::string elt_name = "planner") const;

private:
  bool _ccell_exists_p(const CCID &id) const;

public:
  friend SimpleSliceTree;
  friend SimpleSlice;
  friend SimpleCell;
  friend CompoundCell;

};



template< class Kernel >
class Planner< Kernel >::PlanningParameters
{
public:
  using Planner_ = Planner< Kernel >;
  using DistNT = Planner_::DistNT;

public:
  DistNT minimum_slice_size;
  bool heuristic_sort_admissible_first;
  double heuristic_admissible_bonus;
  double slice_augmentation_factor;

public:
  PlanningParameters()
    : minimum_slice_size{}
    , heuristic_sort_admissible_first{true}
    , heuristic_admissible_bonus{1.0}
    , slice_augmentation_factor{0.0}
  {}

};



} // mrnav::planner_6124e1a8685f
