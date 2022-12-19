#include <mrnav/planner_6124e1a8685f/models/boost_geom_double.hpp>

#include <cstring>
#include <iostream>
#include <tinyxml2.h>

using Planner = mrnav::planner_6124e1a8685f::PlannerBGD;
using GK = Planner::GeometryKernel;

int
main (int argc, char **argv)
{
  if(argc < 2)
    throw std::runtime_error("expected path to setup.xml");
  if(argc < 3)
    throw std::runtime_error("expected maximum amount of iterations");
  size_t amount_of_iterations = std::stoi(argv[2]);
  // Load setup file.
  std::cout << "Loading setup file: " << argv[1] << std::endl;
  tinyxml2::XMLDocument xml_doc_setup;
  xml_doc_setup.LoadFile(argv[1]);
  // Get setup's root element.
  tinyxml2::XMLElement *xml_setup =
    xml_doc_setup.FirstChildElement("setup");
  assert(xml_setup != nullptr);
  // Get problem definition
  tinyxml2::XMLElement *xml_problem =
    xml_setup->FirstChildElement("problem");
  assert(xml_problem != nullptr);
  const size_t amount_of_robots =
    static_cast<size_t>(xml_problem->IntAttribute("dim"));
  std::cout << "Amount of robots: " << amount_of_robots << std::endl;
  // Allocate problem, initial and final configurations
  Planner::Problem problem{amount_of_robots};
  Planner::Configuration
    conf_init{amount_of_robots},
    conf_goal{amount_of_robots};
  // Load robot specifications.
  std::vector< Planner::Robot > robots{};
  auto xml_robot = xml_problem->FirstChildElement("robot");
  for(size_t k = 0; k < amount_of_robots; k++)
    {
      std::cout << "Loading robot #" << k << std::endl;
      assert(xml_robot != nullptr);
      double radius = xml_robot->DoubleAttribute("radius");
      std::cout << "    Radius = " << radius << std::endl;
      robots.emplace_back(radius);
      auto xml_pos_init = xml_robot->FirstChildElement("init");
      assert(xml_pos_init != nullptr);
      auto xml_pos_goal = xml_robot->FirstChildElement("goal");
      assert(xml_pos_goal != nullptr);
      conf_init[k] = GK::Point{ xml_pos_init->DoubleAttribute("x"),
                                xml_pos_init->DoubleAttribute("y") };
      std::cout << "    Initial position: [" <<
        conf_init[k].x() << ", " << conf_init[k].y() << "]" <<
        std::endl;
      conf_goal[k] = GK::Point{ xml_pos_goal->DoubleAttribute("x"),
                                xml_pos_goal->DoubleAttribute("y") };
      std::cout << "    Goal position:    [" <<
        conf_goal[k].x() << ", " << conf_goal[k].y() << "]" <<
        std::endl;
      xml_robot = xml_robot->NextSiblingElement("robot");
    }
  // Load workspace.
  tinyxml2::XMLDocument xml_doc_workspace;
  std::string xml_doc_workspace_path =
    xml_problem->FirstChildElement("workspace")->Attribute("url");
  std::cout << "Loading workspace from file \"" << xml_doc_workspace_path <<
    "\"" << std::endl;
  xml_doc_workspace.LoadFile(xml_doc_workspace_path.c_str());
  GK::PolygonRegionSet ws{};
  ws.from_xml_element(xml_doc_workspace.FirstChildElement());
  // Formulate problem.
  std::cout << "Formulating problem" << std::endl;
  problem.set_ws(ws);
  for(size_t k = 0; k < amount_of_robots; k++)
    problem.set_robot(k, robots[k]);
  problem.set_conf_init(conf_init);
  problem.set_conf_goal(conf_goal);
  // Load planning parameters.
  std::cout << "Loading planning parameters." << std::endl;
  tinyxml2::XMLElement *xml_params =
    xml_setup->FirstChildElement("planning-parameters");
  Planner::PlanningParameters params{};
  params.minimum_slice_size =
    xml_params->DoubleAttribute("minimum_slice_size");
  params.slice_augmentation_factor =
    xml_params->DoubleAttribute("slice_augmentation_factor");
  params.heuristic_sort_admissible_first =
    xml_params->BoolAttribute("heuristic_sort_admissible_first");
  params.heuristic_admissible_bonus =
    xml_params->DoubleAttribute("heuristic_admissible_bonus");
  // Initialize planner.
  std::cout << "Initializing planner." << std::endl;
  Planner planner{problem, params};
  planner.initialize_hierarchy();
  {
    tinyxml2::XMLDocument xml_doc_hier{};
    auto xml_hier = planner.hierarchy_to_xml_element(&xml_doc_hier);
    xml_doc_hier.InsertFirstChild(xml_hier);
    xml_doc_hier.SaveFile("hierarchy-init.xml");
  }
  planner.initialize_ccells();
  planner.initialize_ccells_frontier();
  {
    tinyxml2::XMLDocument xml_doc_cells{};
    auto xml_cells = planner.ccells_to_xml_element(&xml_doc_cells);
    xml_doc_cells.InsertFirstChild(xml_cells);
    xml_doc_cells.SaveFile("cells-init.xml");
  }
  try {
    planner.initialize_ccinit();
    planner.initialize_ccgoal();
  }
  catch(std::runtime_error)
    {
      tinyxml2::XMLDocument xml_doc_hier{};
      auto xml_hier = planner.hierarchy_to_xml_element(&xml_doc_hier);
      xml_doc_hier.InsertFirstChild(xml_hier);
      xml_doc_hier.SaveFile("hierarchy-failure.xml");
      tinyxml2::XMLDocument xml_doc_cells{};
      auto xml_cells = planner.ccells_to_xml_element(&xml_doc_cells);
      xml_doc_cells.InsertFirstChild(xml_cells);
      xml_doc_cells.SaveFile("cells-failure.xml");
    }
  planner.initialize_path();
  // Save planner's initial state.
  tinyxml2::XMLDocument xml_doc_init{};
  auto xml_planner_init = planner.to_xml_element(&xml_doc_init);
  xml_doc_init.InsertFirstChild(xml_planner_init);
  xml_doc_init.SaveFile("planner-init.xml");
  std::cout << "Amount of Compound Cells: " <<
    planner.get_ccell_amount() << std::endl;
  std::cout << "Size of Frontier: " <<
    planner.get_frontier_size() << std::endl;
  // Iterate.
  for(size_t k = 1; k < amount_of_iterations; k++)
    {
      planner.find_apath_step();
    }
  // Save planner's final state.
  tinyxml2::XMLDocument xml_doc{};
  auto xml_planner = planner.to_xml_element(&xml_doc);
  xml_doc.InsertFirstChild(xml_planner);
  xml_doc.SaveFile("planner-end.xml");
  std::cout << "Amount of Compound Cells: " <<
    planner.get_ccell_amount() << std::endl;
  std::cout << "Size of Frontier: " <<
    planner.get_frontier_size() << std::endl;
}
