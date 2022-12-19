#include <gtest/gtest.h>

#include <mrnav/planner_6124e1a8685f/models/boost_geom_double.hpp>

using Planner = mrnav::planner_6124e1a8685f::PlannerBGD;
using GK = Planner::GeometryKernel;

TEST(robot, unit_circle)
{
  Planner::Robot r{1};
  EXPECT_DOUBLE_EQ(r.get_radius(), 1.0);
  auto pgn1 = r.get_body(GK::Point{0, 0});
  EXPECT_TRUE(pgn1->contains_p(GK::Point{0, 0}));
  EXPECT_FALSE(pgn1->contains_p(GK::Point{1, 1}));
  auto pgn2 = r.get_body(GK::Point{1, 0});
  EXPECT_TRUE(pgn2->contains_p(GK::Point{1, 0}));
  EXPECT_FALSE(pgn2->contains_p(GK::Point{2, 1}));
  auto pgn3 = r.get_body(GK::Point{0, 1});
  EXPECT_TRUE(pgn3->contains_p(GK::Point{0, 1}));
  EXPECT_FALSE(pgn3->contains_p(GK::Point{1, 2}));
  // for(size_t k = 0; k < pgn3->size(); k++)
  //   {
  //     auto p = pgn3->get_vtx(k);
  //     std::cout << p->x() << " " << p->y() << std::endl;
  //   }
}

TEST(robot, circle_1)
{
  Planner::Robot r{5, 128};
  EXPECT_DOUBLE_EQ(r.get_radius(), 5.0);
  auto pgn1 = r.get_body(GK::Point{0, 0});
  EXPECT_TRUE(pgn1->contains_p(GK::Point{0, 0}));
  EXPECT_TRUE(pgn1->contains_p(GK::Point{4.8, 0}));
  EXPECT_FALSE(pgn1->contains_p(GK::Point{5.2, 0}));
  EXPECT_TRUE(pgn1->contains_p(GK::Point{0, 4.8}));
  EXPECT_FALSE(pgn1->contains_p(GK::Point{0, 5.2}));
  auto pgn2 = r.get_body(GK::Point{1, 0});
  EXPECT_TRUE(pgn2->contains_p(GK::Point{1, 0}));
  EXPECT_TRUE(pgn2->contains_p(GK::Point{5.8, 0}));
  EXPECT_FALSE(pgn2->contains_p(GK::Point{6.2, 0}));
  EXPECT_TRUE(pgn2->contains_p(GK::Point{1, 4.8}));
  EXPECT_FALSE(pgn2->contains_p(GK::Point{1, 5.2}));
  auto pgn3 = r.get_body(GK::Point{0, 1});
  EXPECT_TRUE(pgn3->contains_p(GK::Point{0, 1}));
  EXPECT_TRUE(pgn3->contains_p(GK::Point{4.8, 1}));
  EXPECT_FALSE(pgn3->contains_p(GK::Point{5.2, 1}));
  EXPECT_TRUE(pgn3->contains_p(GK::Point{0, 5.8}));
  EXPECT_FALSE(pgn3->contains_p(GK::Point{0, 6.2}));
}
