#pragma once

#include <mrnav/planner_6124e1a8685f/planner.hpp>
#include <mrnav/planner_6124e1a8685f/config.hpp>
#include <mrnav/planner_6124e1a8685f/compound_cell.hpp>

#include <cassert>

#include <iostream>

namespace mrnav::planner_6124e1a8685f
{

template< class BP >
Planner< BP >::Planner(const Problem &problem,
                       const PlanningParameters &parameters)
  : _self{this}
  , _problem{problem}
  , _parameters{parameters}
  , _amount_of_robots{problem.robots.size()}
  , _hierarchy{_amount_of_robots}
  , _ccells{}
  , _ccells_frontier{}
  , _cc_init{nullptr}
  , _cc_goal{nullptr}
  , _path{}
  , _iteration_counter{0}
  , _flag_hierarchy_initialized_p{false}
  , _flag_ccells_initialized_p{false}
  , _flag_frontier_initialized_p{false}
  , _flag_ccinit_initialized_p{false}
  , _flag_ccgoal_initialized_p{false}
  , _flag_path_initialized_p{false}
{
  // Validate given problem.
  if(not this->_problem.valid_p())
    throw std::invalid_argument("given problem is invalid");
  // Initialize internal state.
  // this->_initialize();
}

template< class BP >
Planner< BP >::~Planner()
{
  this->_self.invalidate();
}



template< class BP >
void
Planner< BP >::initialize()
{
  this->initialize_hierarchy();
  this->initialize_ccells();
  this->initialize_ccells_frontier();
  // Find admissible initial configuration.
  this->initialize_ccinit();
  // Find admissible goal configuration.
  this->initialize_ccgoal();
  // Initialize list of paths.
  this->initialize_path();
}

template< class BP >
void
Planner< BP >::initialize_hierarchy()
{
  // If already initialized, do nothing.
  if(this->_flag_hierarchy_initialized_p)
    return;
  // Assert that the container of trees has the appropriate size.
  assert(this->_hierarchy.size() == this->_amount_of_robots);
  // Build trees.
  for(size_t k = 0; k < this->_amount_of_robots; k++)
    this->_hierarchy[k] = SimpleSliceTreeSP(
      new SimpleSliceTree{this->_self, k});
  // Mark this initalization step as done.
  this->_flag_hierarchy_initialized_p = true;
}

template< class BP >
void
Planner< BP >::initialize_ccells()
{
  // If already initialized, do nothing.
  if(this->_flag_ccells_initialized_p)
    return;
  // If required initialization steps have not been performed, do them now.
  this->initialize_hierarchy();
  // Ensure set of compound cells is empty.
  this->_ccells.clear();
  // Gather building blocks, i.e., simple cells of each robot.
  CCBuildingBlocks blocks{this->_amount_of_robots};
  for(size_t k = 0; k < this->_amount_of_robots; k++) {
    blocks[k].reserve(this->_hierarchy[k]->_root->_scells.size());
    blocks[k].insert(blocks[k].begin(),
                     this->_hierarchy[k]->_root->_scells.cbegin(),
                     this->_hierarchy[k]->_root->_scells.cend());
  }
  // Construct compound cells.
  std::vector< CompoundCellSP > ccells;
  this->_find_or_build_ccells(blocks, ccells);
  // Mark this initalization step as done.
  this->_flag_ccells_initialized_p = true;
}

template< class BP >
void
Planner< BP >::initialize_ccells_frontier()
{
  // If already initialized, do nothing.
  if(this->_flag_frontier_initialized_p)
    return;
  // If required initialization steps have not been performed, do them now.
  this->initialize_ccells();
  // Ensure frontier is empty.
  this->_ccells_frontier.clear();
  // Put every initial compound cell that is not inadmissible into
  // the frontier.
  for(auto kv: this->_ccells)
    {
      // Extract compound cell id.
      auto key = kv.first;
      // Extract compound cell itself.
      auto val = kv.second;
      // If not inadmissible, add it to the frontier.
      if(not val->inadmissible_p())
        this->_ccells_frontier[key] = val;
      // this->_ccells_frontier.push_front(cc);
    }
  // Mark this initalization step as done.
  this->_flag_frontier_initialized_p = true;
}

template< class BP >
void
Planner< BP >::initialize_ccinit()
{
  // If already initialized, do nothing.
  if(this->_flag_ccinit_initialized_p)
    return;
  // If required initialization steps have not been performed, do them now.
  this->initialize_ccells_frontier();
  // Find or build an admissible enclosing compound cell.
  this->_cc_init =
    this->_find_or_build_enclosing_accell(this->_problem.pos_init);
  // If no such cell exists, raise an error.
  if(this->_cc_init == nullptr)
    throw std::runtime_error("no admissible initial configuration found");
  // Mark this initalization step as done.
  this->_flag_ccinit_initialized_p = true;
}

template< class BP >
void
Planner< BP >::initialize_ccgoal()
{
  // If already initialized, do nothing.
  if(this->_flag_ccgoal_initialized_p)
    return;
  // If required initialization steps have not been performed, do them now.
  this->initialize_ccells_frontier();
  // Find or build an admissible enclosing compound cell.
  this->_cc_goal =
    this->_find_or_build_enclosing_accell(this->_problem.pos_goal);
  // If no such cell exists, raise an error.
  if(this->_cc_goal == nullptr)
    throw std::runtime_error("no admissible goal configuration found");
  // Mark this initalization step as done.
  this->_flag_ccgoal_initialized_p = true;
}

template< class BP >
void
Planner< BP >::initialize_path()
{
  // If already initialized, do nothing.
  if(this->_flag_path_initialized_p)
    return;
  // If required initialization steps have not been performed, do them now.
  this->initialize_ccinit();
  this->initialize_ccgoal();
  // Allocate extended path.
  this->_path = ExtendedPathSP{ new ExtendedPath{} };
  this->_path->push_front_lvl(this->_cc_init);
  // Try to connect initial and goal configurations.
  this->_connect_cell(this->_cc_goal);
  // Mark this initalization step as done.
  this->_flag_path_initialized_p = true;
}



template< class BP >
typename Planner< BP >::CompoundCellSP
Planner< BP >::_find_or_build_enclosing_accell(const Configuration &p)
{
  // Get the compound cell belonging to the frontier that contains
  // the given configuration.
  CompoundCellSP cc = this->_get_enclosing_ccell(p);
  // If no such compound cell exists, return null.
  if(cc == nullptr)
    return nullptr;
  // If the cell found is obscure, expand it until either
  // an enclosing admissible cell is found or none exists to begin with.
  while(cc->obscure_p()) {
    // Expand the enclosing cell.
    std::vector< CompoundCellSP > expansion;
    this->_expand_occell(cc, expansion);
    // Get the compound cell in the expansion that contains
    // the given configuration.
    cc = this->_get_enclosing_ccell(p, expansion);
    // If no enlosing cell exists, return null.
    if(cc == nullptr)
      return nullptr;
  }
  // Assert the compound cell that was found is admissible.
  assert(cc->admissible_p());
  return cc;
}

template< class BP >
typename Planner< BP >::Planner::PathSP
Planner< BP >::find_apath()
{
  // Make sure that path have been initialized.
  this->initialize_path();
  // Iteratively expand path until it becomes admissible.
  while(true)
    {
      // Assert that path is not nullptr.
      assert(this->_path != nullptr);
      // If path's length is zero, there is nothing to do.
      if(this->_path->length() == 0)
        return nullptr;
      // Check if a solution was found. If true, return it.
      if(this->_path->admissible_p())
        if(this->_path->back_active() == this->_cc_goal)
          return this->_path->to_simple_path();
      // Execute search step.
      this->find_apath_step();
    }
  return this->_path->to_simple_path();
}

template< class BP >
void
Planner< BP >::find_apath_step()
{
  // Make sure that path have been initialized.
  this->initialize_path();
  // Assert that path is not nullptr.
  assert(this->_path != nullptr);
  // If path's length is zero, there is nothing to do.
  if(this->_path->length() == 0)
    return;
  // Check if a solution was found. If true, return it.
  if(this->_path->admissible_p())
    if(this->_path->back_active() == this->_cc_goal)
      return;
  // Expand path at that cell.
  this->_expand_path();
  // Increment iteration counter.
  this->_iteration_counter++;
}



template< class BP >
void
Planner< BP >::_expand_path()
{
  using LevelList = typename ExtendedPath::LevelList;
  // Find the first obscure active cell in that path.
  // CompoundCellSP ccell = this->_path->_get_first_active_occell();
  typename LevelList::iterator lvl_crit =
    this->_path->_get_first_active_occell_lvl();
  if(lvl_crit != this->_path->_str.end())
    {
      // Assert that this is not the first level not the list's end.
      assert(lvl_crit != this->_path->_str.begin());
      assert(lvl_crit != this->_path->_str.end());
      CompoundCellSP ccell = lvl_crit->front();
      // Assert that compound cell returned is not null,
      // since its impossible a obscure path to not have an obscure cell.
      assert(ccell != nullptr);
      assert(ccell->obscure_p());
      // Split path into a prefix and a suffix.
      ExtendedPathSP prefix{nullptr}, suffix{nullptr};
      this->_path->split_at(lvl_crit, prefix, suffix);
      // Assert that prefix and suffix are not null.
      assert(prefix != nullptr);
      assert(suffix != nullptr);
      assert(prefix->length() + suffix->length() == this->_path->length());
      assert(prefix->engaged_cells_amount() + suffix->engaged_cells_amount() ==
             this->_path->engaged_cells_amount());
      // Compute the expansion of the given ccell.
      std::vector< CompoundCellSP > expansion;
      this->_expand_occell(ccell, expansion);
      // Get last level of prefix, which must be equal to path's critical level.
      typename LevelList::iterator lvl_last =
        prefix->_get_first_active_occell_lvl();
      assert(lvl_last->front() == lvl_crit->front());
      // Get level and active cell before crititcal.
      typename LevelList::iterator lvl_before = lvl_last; lvl_before--;
      CompoundCellSP ccell_before = lvl_before->front();
      // Assert that active cell before critical is not null.
      assert(ccell_before != nullptr);
      // Filter expansion cells that are adjacent to active cell before critical.
      std::vector< CompoundCellSP > adjacent;
      adjacent.reserve(expansion.size());
      for(auto cc: expansion) {
        if(ccell_before->adjacent_p(cc))
          adjacent.push_back(cc);
      }
      // TODO: Sort adjacent expansion cells.
      this->_sort_ccells(adjacent);
      // Replace active cell at the prefix's end its with adjacent expansion.
      // lvl_crit->pop_front();
      // for(auto cc: adjacent)
      //   lvl_crit->push_front(cc);
      prefix->_replace_active(lvl_last, adjacent);
      // TODO (OPTIONAL): Try to connect prefix and suffix instead of
      //                  directly connecting prefix to goal.
      // Try to connect prefix with goal.
      this->_path = prefix;
    }
  this->_connect_cell(this->_cc_goal);
}

template< class BP >
typename Planner< BP >::CompoundCellSP
Planner< BP >::_get_enclosing_ccell(const Configuration &p)
{
  for(auto cc: this->_ccells_frontier)
    if(cc.second->contains_p(p))
      return cc.second;
  return nullptr;
}

template< class BP >
typename Planner< BP >::CompoundCellSP
Planner< BP >::_get_enclosing_ccell(const Configuration &p,
                                    const std::vector< CompoundCellSP > &set)
{
  for(auto cc: set)
    if(cc->contains_p(p))
      return cc;
  return nullptr;
}

template< class BP >
void
Planner< BP >::_connect_cell(CompoundCellSP cell)
{
  using Level = typename ExtendedPath::Level;
  using LevelList = typename ExtendedPath::LevelList;
  // Get path's last level.
  // Level &lvl_end = this->_path->back_lvl();
  typename LevelList::iterator lvl_end = this->_path->back_lvl();
  // Check if the last level is empty.
  // If true, then we need to back-track.
  if(lvl_end->empty())
    {
      // Pop current last level, which is empty.
      this->_path->pop_last_lvl();
      // If path's length is 0, then we lacked out since this means that
      // the problem is infeasible.
      if(this->_path->length() == 0)
        return;
      // Get reference to the previous level,
      // which is, once again, the end of the path.
      lvl_end = this->_path->back_lvl();
      // Since the last level was empty, this means that
      // the active cell of the level before that must be popped.
      // Also, if the path's prefix before the last level is admissible,
      // this cell can be safely removed from the frontier.
      assert(lvl_end->size() > 0);
      bool flag_admissible_prefix =
        this->_path->prefix_admissible_p(lvl_end);
      auto cc = this->_path->pop_last_active();
      if(flag_admissible_prefix) {
        auto cc_loc = this->_ccells_frontier.find(cc->_id);
        // Check if cell is part of the frontier before removing it because
        // it may have been already dropped.
        if(cc_loc != this->_ccells_frontier.end())
          this->_ccells_frontier.erase(cc_loc);
      }
      // Now, try to connect cells anew.
      if(lvl_end->empty())
        this->_connect_cell(cell);
      // Nothing left to do, so move on.
      return;
    }
  // Check if path's end contains the goal cell.
  // If so, make it the active cell and simply return.
  // for(auto it = lvl_end->begin(); it != lvl_end->end(); it++) {
  //   auto cc = *it;
  //   if(cc == cell) {
  //     lvl_end->erase(it);
  //     lvl_end->push_front(cc);
  //     return;
  //   }
  // }
  for(auto cc: *lvl_end) {
    if(cc == cell) {
      this->_path->_make_active(lvl_end, cell);
      return;
    }
  }
  // Get the last level's active cell.
  CompoundCellSP cc_init = lvl_end->front();
  assert(cc_init != nullptr);
  // Get frontier cells that are adjacent to this initial cell and
  // are not already part of the path.
  Level adjacent;
  for(auto kv: this->_ccells_frontier)
    {
      auto cc = kv.second;
      // If frontier cell is already part of the path
      // as an active cell, just move on.
      // This is done to avoid cycles without affecting completeness.
      if(this->_path->contains_active_cell_p(cc))
      // if(this->_path->contains_cell_p(cc))
        continue;
      // Otherwise, add it to the adjacent set.
      if(cc_init->adjacent_p(cc))
        adjacent.push_back(cc);
    }
  // TODO: Sort adjacent cells using some heuristic.
  this->_sort_ccells(adjacent);
  // Add adjacent cells as a new level to the path's end.
  this->_path->push_back_lvl(adjacent);
  // Try to connect new path to goal, only if path is not valid (i.e.,
  // a level is empty).
  if(adjacent.empty())
    this->_connect_cell(cell);
}

template< class BP >
void
Planner< BP >::_expand_occell(CompoundCellSP ccell,
                              std::vector< CompoundCellSP > &expansion)
{
  // Assert that given cell is obscure.
  assert(ccell->obscure_p());
  // If cell has already been expanded, return the previous expansion.
  if(ccell->expanded_p()) {
    //// >>>> Alternative #1:
    // expansion.clear();
    // expansion.reserve(ccell->_expansion.size());
    // expansion.insert(expansion.begin(),
    //                  ccell->_expansion.cbegin(),
    //                  ccell->_expansion.cend());
    //// >>>> Alternative #2:
    // Expanding each "readily expanded" cell of
    // the expansion recursively, until only unexpanded cells are left.
    // Finally, return the unexpanded cells directly.
    std::list< CompoundCellSP > expansion_list;
    this->_collect_ccell_unexpanded_expansions(ccell, expansion_list);
    expansion.reserve(expansion_list.size());
    expansion.insert(expansion.end(),
                     expansion_list.begin(),
                     expansion_list.end());
    return;
  }
  // Get conflicting simple cells.
  std::vector< size_t > indices;
  ccell->get_conflicting_oascells(indices);
  // Assert that there are indeed conflicting simple cells.
  assert(indices.size() > 0);
  // Get compound cell's components.
  const CCComponents &components = ccell->_components;
  // Choose component with largest slice for expansion.
  DistNT max_area = 0.0;
  size_t max_area_idx = 0;
  for(size_t k = 0; k < indices.size(); k++) {
    DistNT area = components[indices[k]]->_slice->get_area();
    if(area > max_area) {
      max_area = area;
      max_area_idx = indices[k];
    }
  }
  // Expand compound cell at the specified simple cell.
  this->_expand_ccell_at_scell(ccell, max_area_idx, expansion);
}

template< class BP >
void
Planner< BP >::_collect_ccell_unexpanded_expansions(
  CompoundCellSP ccell,
  std::list< CompoundCellSP > &expansion)
{
  // Assert that the given compound cell is expanded.
  assert(ccell->expanded_p());
  // Iterate over given ccell's expansion.
  for(auto cc : ccell->_expansion)
    {
      // If cc has already been expanded, recursively process its expansion.
      // Otherwise, add it to the result.
      if(cc->expanded_p())
        this->_collect_ccell_unexpanded_expansions(cc, expansion);
      else
        expansion.push_back(cc);
    }
}

template< class BP >
void
Planner< BP >::_expand_ccell_at_scell(CompoundCellSP ccell,
                                      size_t scell_idx,
                                      std::vector< CompoundCellSP > &expansion)
{
  // Assert that this cell has not been expanded already.
  assert(not ccell->expanded_p());
  // Get the specified simple cell.
  SimpleCellSP scell = ccell->_components[scell_idx];
  // Expand the specified simple cell and get its children.
  std::vector< SimpleCellSP > scell_expansion;
  this->_expand_scell(scell, scell_expansion);
  // assert(scell_expansion.size() <= 4);
  // Assemble compound cell building blocks.
  CCBuildingBlocks bb{this->_amount_of_robots};
  for(size_t k = 0; k < this->_amount_of_robots; k++) {
    if(k == scell_idx)
      continue;
    bb[k].push_back(ccell->_components[k]);
  }
  bb[scell_idx].insert(bb[scell_idx].begin(),
                       scell_expansion.begin(),
                       scell_expansion.end());
  // Get the compound cell's expansion.
  std::vector< CompoundCellSP > ccell_expansion;
  this->_find_or_build_ccells(bb, ccell_expansion);
  assert(ccell_expansion.size() == scell_expansion.size());
  // Assign parent to new cells.
  for(auto cc: ccell_expansion)
    if(cc->_parent == nullptr)
      cc->_parent = ccell->_self;
  // Filter out inadmissible cells from the expansion.
  expansion.clear();
  expansion.reserve(ccell_expansion.size());
  for(auto cc: ccell_expansion)
    if(not cc->inadmissible_p())
      expansion.push_back(cc);
  // Mark the compound cell as expanded.
  ccell->_expanded_p = true;
  // Also, store the expansion to compound cell's cache.
  ccell->_expansion.insert(ccell->_expansion.end(),
                           expansion.begin(),
                           expansion.end());
  // Assert that the compound cell has been marked as expanded and
  // the expansion consists of non-inadmissible cells.
  assert(ccell->expanded_p());
  for(auto ex: expansion)
    assert(not ex->inadmissible_p());
  // Remove the given compound cell from the frontier.
  // Note that, not being already expanded and all,
  // this cell must belong to the frontier.
  auto ccell_loc = this->_ccells_frontier.find(ccell->_id);
  assert(ccell_loc != this->_ccells_frontier.end());
  this->_ccells_frontier.erase(ccell_loc);
  // Put the expansion to the frontier.
  // Note that, not all cells may have been created just now, so be careful.
  for(auto ex: expansion) {
    auto loc = this->_ccells_frontier.find(ex->_id);
    if(not ex->expanded_p()) {
      // Being unexpanded does not imply that this cell was created just now,
      // so it may already be registered to the frontier.
      if(loc == this->_ccells_frontier.end())
        this->_ccells_frontier[ex->_id] = ex;
    }
    else
      // Assert that an readily expanded cell is not a member of the frontier.
      assert(loc == this->_ccells_frontier.end());
  }
}

template< class BP >
void
Planner< BP >::_expand_scell(SimpleCellSP scell,
                             std::vector< SimpleCellSP > &expansion)
{
  // If simple cell has not been expanded already, expand it now.
  if(not scell->expanded_p())
    scell->_expand();
  // Get simple cell's children whose size is not less than the threshold.
  expansion.clear();
  expansion.reserve(scell->_children.size());
  for(auto child: scell->_children)
    {
      if(child->_slice->get_smallest_dimension() <
         this->_parameters.minimum_slice_size)
        continue;
      expansion.push_back(child);
    }
  // expansion.clear();
  // expansion.insert(expansion.begin(),
  //                  scell->_children.begin(),
  //                  scell->_children.end());
}



template< class BP >
size_t
Planner< BP >::amount_of_robots() const
{
  return this->_amount_of_robots;
}

template< class BP >
const typename Planner< BP >::Robot&
Planner< BP >::get_robot(size_t idx) const
{
  if(idx < this->_amount_of_robots)
    return this->_problem.robots[idx];
  throw std::out_of_range("robot index out of range");
}

template< class BP >
typename Planner< BP >::PolygonRegionSet
Planner< BP >::get_robot_cs(size_t idx) const
{
  // Make sure that given robot index is valid.
  if(idx >= this->_amount_of_robots)
    throw std::out_of_range("robot index out of range");
  // If hierarchy has been computed already,
  // grab the configuration space from there.
  if(this->_flag_hierarchy_initialized_p)
    return this->_hierarchy[idx]->get_ws_aug();
  // Otherwise, build one now.
  else
    return *(this->get_augmented_workspace(
               this->get_robot(idx).get_radius()));
}

template< class BP >
typename Planner< BP >::PolygonRegionSetSP
Planner< BP >::get_augmented_workspace(DistNT radius) const
{
  return this->_problem.ws.get_buffer(-radius);
}

template< class BP >
size_t
Planner< BP >::get_ccell_amount() const
{
  return this->_ccells.size();
}

template< class BP >
size_t
Planner< BP >::get_frontier_size() const
{
  return this->_ccells_frontier.size();
}



template< class BP >
void
Planner< BP >::_find_or_build_ccells(const CCBuildingBlocks &bb,
                                     std::vector< CompoundCellSP > &ccells)
{
  // Assert that the number of building block bins is valid.
  assert(bb.size() == this->_amount_of_robots);
  // Compute the amount of compound cells that will be returned.
  size_t amount = 1;
  for(size_t k = 0; k < this->_amount_of_robots; k++)
    amount *= bb[k].size();
  // If given list of ccells is empty,
  // reserve enough space for the results cells.
  ccells.reserve(amount);
  // Handle each case.
  if(amount == 0)
    {
      // No cells can be produced, because at least one indice has no blocks.
    }
  else if(amount == 1)
    {
      // Just a single compound cell can be produced,
      // because all indices have exactly one simple cell.
      CCComponents components{this->_amount_of_robots};
      for(size_t k = 0; k < this->_amount_of_robots; k++)
        components[k] = bb[k][0];
      ccells.push_back(this->_find_or_build_ccell(components));
    }
  else
    {
      // Find the first index that has more than one blocks.
      size_t idx;
      for(size_t k = 0; k < this->_amount_of_robots; k++) {
        if(bb[k].size() > 1) { idx = k; break; }
      }
      // Recursively reduce that the building blocks at that index.
      for(size_t k = 0; k < bb[idx].size(); k++) {
        CCBuildingBlocks bb_new = bb;
        bb_new[idx].clear();
        bb_new[idx].push_back(bb[idx].at(k));
        this->_find_or_build_ccells(bb_new, ccells);
      }
    }
}

template< class BP >
typename Planner< BP >::CompoundCellSP
Planner< BP >::_find_or_build_ccell(const CCComponents &components)
{
  // Assert that the amount of components given is valid.
  assert(components.size() == this->_amount_of_robots);
  // Compute compound cell's id.
  CCID id{};
  id.resize(this->_amount_of_robots);
  for(size_t k = 0; k < this->_amount_of_robots; k++)
    id[k] = reinterpret_cast< uintptr_t >(components[k].get());
  // Check if compound cell already exists, in which case simply return that,
  // otherwise, build and return a new one.
  auto sr = this->_ccells.find(id);
  if(sr == this->_ccells.end()) {
    CompoundCellSP cc_new{ new CompoundCell{this->_self, components} };
    assert(cc_new->_components == components);
    assert(cc_new->_id == id);
    this->_compute_heuristic(cc_new);
    this->_ccells[id] = cc_new;
    return cc_new;
  } else {
    return sr->second;
  }
}



// template< class BP >
// void
// Planner< BP >::_sort_ccells(std::vector<CompoundCellSP> &ccells)
//   const
// {
//   std::vector< CompoundCellSP > ccells_admissible, ccells_obscure;
//   ccells_admissible.reserve(ccells.size());
//   ccells_obscure.reserve(ccells.size());
//   for(auto cc: ccells)
//     {
//       if(cc->admissible_p())
//         ccells_admissible.push_back(cc);
//       else
//         ccells_obscure.push_back(cc);
//     }
//   // Sort admissible cells based on heuristic.
//   std::sort(ccells_admissible.begin(), ccells_admissible.end(),
//             [](CompoundCellSP a, CompoundCellSP b) {
//               return a->_heuristic < b->_heuristic;
//             });
//   // Sort obscure cells based on heuristic.
//   std::sort(ccells_obscure.begin(), ccells_obscure.end(),
//             [](CompoundCellSP a, CompoundCellSP b) {
//               return a->_heuristic < b->_heuristic;
//             });
//   ccells.clear();
//   ccells.insert(ccells.end(),
//                 ccells_admissible.begin(), ccells_admissible.end());
//   ccells.insert(ccells.end(),
//                 ccells_obscure.begin(), ccells_obscure.end());
// }

// template< class BP >
// void
// Planner< BP >::_sort_ccells(std::list<CompoundCellSP> &ccells)
//   const
// {
//   std::vector< CompoundCellSP > ccells_admissible, ccells_obscure;
//   ccells_admissible.reserve(ccells.size());
//   ccells_obscure.reserve(ccells.size());
//   for(auto cc: ccells)
//     {
//       if(cc->admissible_p())
//         ccells_admissible.push_back(cc);
//       else
//         ccells_obscure.push_back(cc);
//     }
//   // Sort admissible cells based on heuristic.
//   std::sort(ccells_admissible.begin(), ccells_admissible.end(),
//             [](CompoundCellSP a, CompoundCellSP b) {
//               return a->_heuristic < b->_heuristic;
//             });
//   // Sort obscure cells based on heuristic.
//   std::sort(ccells_obscure.begin(), ccells_obscure.end(),
//             [](CompoundCellSP a, CompoundCellSP b) {
//               return a->_heuristic < b->_heuristic;
//             });
//   ccells.clear();
//   ccells.insert(ccells.end(),
//                 ccells_admissible.begin(), ccells_admissible.end());
//   ccells.insert(ccells.end(),
//                 ccells_obscure.begin(), ccells_obscure.end());
// }

template< class BP >
template< class IT >
void
Planner< BP >::_sort_ccells(IT &ccells)
  const
{
  if(this->_parameters.heuristic_sort_admissible_first)
    {
      std::vector< CompoundCellSP > ccells_admissible, ccells_obscure;
      ccells_admissible.reserve(ccells.size());
      ccells_obscure.reserve(ccells.size());
      for(auto cc: ccells)
        {
          if(cc->admissible_p())
            ccells_admissible.push_back(cc);
          else
            ccells_obscure.push_back(cc);
        }
      // Sort admissible cells based on heuristic.
      std::sort(ccells_admissible.begin(), ccells_admissible.end(),
                [](CompoundCellSP a, CompoundCellSP b) {
                  return a->_heuristic < b->_heuristic;
                });
      // Sort obscure cells based on heuristic.
      std::sort(ccells_obscure.begin(), ccells_obscure.end(),
                [](CompoundCellSP a, CompoundCellSP b) {
                  return a->_heuristic < b->_heuristic;
                });
      ccells.clear();
      ccells.insert(ccells.end(),
                    ccells_admissible.begin(), ccells_admissible.end());
      ccells.insert(ccells.end(),
                    ccells_obscure.begin(), ccells_obscure.end());
      return;
    }
  else
    {
      std::vector< CompoundCellSP > ccells_sorted;
      ccells_sorted.reserve(ccells.size());
      ccells_sorted.insert(ccells_sorted.end(),
                           ccells.cbegin(), ccells.cend());
      std::sort(ccells_sorted.begin(), ccells_sorted.end(),
                [](CompoundCellSP a, CompoundCellSP b) {
                  return a->_heuristic < b->_heuristic;
                });
      ccells.clear();
      ccells.insert(ccells.end(),
                    ccells_sorted.begin(), ccells_sorted.end());
      return;
    }
}

template< class BP >
void
Planner< BP >::_compute_heuristic(CompoundCellSP cell)
{
  double heuristic = 0.0;
  // for(auto sc: cell->_components)
  for(size_t k = 0; k < this->_amount_of_robots; k++)
    {
      auto sc = cell->_components[k];
      auto dist = sc->_shape.distance(this->_problem.pos_goal[k]);
      if(not this->_parameters.heuristic_sort_admissible_first)
        if(cell->admissible_p())
          dist *= this->_parameters.heuristic_admissible_bonus;
      heuristic += dist;
    }
  cell->_heuristic = heuristic;
}



template< class BP >
bool
Planner< BP >::_ccell_exists_p(const CCID &id)
  const
{
  return this->_ccells.find(id) == this->_ccells.end();
}



template< class BP >
tinyxml2::XMLElement*
Planner< BP >::hierarchy_to_xml_element(tinyxml2::XMLDocument *doc,
                                        std::string elt_name)
  const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  for(size_t k = 0; k < this->_amount_of_robots; k++)
    {
      auto xml_hier = this->_hierarchy[k]->to_xml_element(doc);
      xml_elt->InsertEndChild(xml_hier);
    }
  return xml_elt;
}

template< class BP >
tinyxml2::XMLElement*
Planner< BP >::path_to_xml_element(tinyxml2::XMLDocument *doc,
                                   std::string elt_name)
  const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  auto xml_path = this->_path->to_xml_element(doc);
  xml_elt->InsertEndChild(xml_path);
  return xml_elt;
}

template< class BP >
tinyxml2::XMLElement*
Planner< BP >::ccells_to_xml_element(tinyxml2::XMLDocument *doc,
                                     std::string elt_name)
  const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  // Store id of initial cell.
  xml_elt->SetAttribute(
    "cc_init",
    std::to_string(reinterpret_cast< uintptr_t >(this->_cc_init.get())).c_str()
  );
  // Store id of goal cell.
  xml_elt->SetAttribute(
    "cc_goal",
    std::to_string(reinterpret_cast< uintptr_t >(this->_cc_goal.get())).c_str()
  );
  // Iterate over all compound cells.
  for(auto elt: this->_ccells) {
    auto cc = elt.second;
    auto xml_cc = cc->to_xml_element(doc, "cell");
    xml_elt->InsertEndChild(xml_cc);
  }
  // Iterate over frontier cells.
  auto xml_frontier = doc->NewElement("frontier");
  xml_elt->InsertEndChild(xml_frontier);
  for(auto elt: this->_ccells_frontier) {
    auto cc = elt.second;
    auto xml_cc = doc->NewElement("cell");
    xml_cc->SetAttribute(
      "refid",
      std::to_string(reinterpret_cast< uintptr_t >(cc.get())).c_str());
    xml_frontier->InsertEndChild(xml_cc);
  }
  return xml_elt;
}

template< class BP >
tinyxml2::XMLElement*
Planner< BP >::to_xml_element(tinyxml2::XMLDocument *doc,
                              std::string elt_name)
  const
{
  tinyxml2::XMLElement *xml_elt = doc->NewElement(elt_name.c_str());
  auto xml_problem = this->_problem.to_xml_element(doc);
  auto xml_hier = this->hierarchy_to_xml_element(doc);
  auto xml_ccells = this->ccells_to_xml_element(doc);
  auto xml_path = this->path_to_xml_element(doc);
  xml_elt->InsertEndChild(xml_problem);
  xml_elt->InsertEndChild(xml_hier);
  xml_elt->InsertEndChild(xml_ccells);
  xml_elt->InsertEndChild(xml_path);
  return xml_elt;
}



} // mrnav::planner_6124e1a8685f
