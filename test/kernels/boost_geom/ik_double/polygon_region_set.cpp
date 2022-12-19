#include <gtest/gtest.h>

#include <mrnav/kernels/boost_geom/ik/double.hpp>

using kern = mrnav::kernels::boost_geom::Kernel_double;

TEST(polygon_region_set, default_)
{
  kern::PolygonRegionSet dflt;
  EXPECT_EQ(dflt.size(), 0);
  EXPECT_ANY_THROW(dflt.get_region(0));
  EXPECT_ANY_THROW(dflt.get_region(1));
  EXPECT_ANY_THROW(dflt.get_region(-1));
  EXPECT_ANY_THROW(dflt.set_region(0, kern::PolygonRegion{}));
  EXPECT_ANY_THROW(dflt.set_region(1, kern::PolygonRegion{}));
  EXPECT_ANY_THROW(dflt.set_region(-1, kern::PolygonRegion{}));
  EXPECT_ANY_THROW(dflt.remove_region(0));
  EXPECT_ANY_THROW(dflt.remove_region(1));
  EXPECT_ANY_THROW(dflt.remove_region(-1));
}

TEST(polygon_region_set, accessors)
{
  kern::PolygonRegionSet set;
  set.insert_region(0, kern::PolygonRegion{});
  // TODO
}

TEST(polygon_region_set, union_1)
{
  kern::Polygon pgn1;
  pgn1.append_vtx(kern::Point{0, 0});
  pgn1.append_vtx(kern::Point{3, 0});
  pgn1.append_vtx(kern::Point{3, 3});
  pgn1.append_vtx(kern::Point{0, 3});
  kern::Polygon pgn2;
  pgn2.append_vtx(kern::Point{2, 2});
  pgn2.append_vtx(kern::Point{5, 2});
  pgn2.append_vtx(kern::Point{5, 5});
  pgn2.append_vtx(kern::Point{2, 5});
  kern::PolygonRegionSet set;
  set.unite(pgn1);
  set.unite(pgn2);
  ASSERT_EQ(set.size(), 1);
  EXPECT_TRUE(set.contains_p(kern::Point{1, 1}));
  EXPECT_TRUE(set.contains_p(kern::Point{2.5, 2.5}));
  EXPECT_TRUE(set.contains_p(kern::Point{4, 4}));
  EXPECT_FALSE(set.contains_p(kern::Point{5.1, 5.1}));
  EXPECT_FALSE(set.contains_p(kern::Point{-.1, -.1}));
}

TEST(polygon_region_set, union_2)
{
  kern::Polygon pgn1;
  pgn1.append_vtx(kern::Point{0, 0});
  pgn1.append_vtx(kern::Point{1, 0});
  pgn1.append_vtx(kern::Point{1, 1});
  pgn1.append_vtx(kern::Point{0, 1});
  kern::Polygon pgn2;
  pgn2.append_vtx(kern::Point{2, 2});
  pgn2.append_vtx(kern::Point{3, 2});
  pgn2.append_vtx(kern::Point{3, 3});
  pgn2.append_vtx(kern::Point{2, 3});
  kern::PolygonRegionSet set;
  set.unite(pgn1);
  set.unite(pgn2);
  ASSERT_EQ(set.size(), 2);
  EXPECT_TRUE(set.contains_p(kern::Point{0.5, 0.5}));
  EXPECT_TRUE(set.contains_p(kern::Point{2.5, 2.5}));
  EXPECT_FALSE(set.contains_p(kern::Point{1.5, 1.5}));
}

TEST(polygon_region_set, envelope1)
{
  kern::Polygon pgn1;
  pgn1.append_vtx(kern::Point{-1, -2});
  pgn1.append_vtx(kern::Point{+3, -2});
  pgn1.append_vtx(kern::Point{+3, +3});
  pgn1.append_vtx(kern::Point{-1, +3});
  kern::Polygon pgn2;
  pgn2.append_vtx(kern::Point{+2, +2});
  pgn2.append_vtx(kern::Point{+6, +2});
  pgn2.append_vtx(kern::Point{+6, +5});
  pgn2.append_vtx(kern::Point{+2, +5});
  kern::PolygonRegionSet set;
  set.unite(pgn1);
  set.unite(pgn2);
  kern::CoordNT x_min, y_min, x_max, y_max;
  set.get_envelope(x_min, x_max, y_min, y_max);
  ASSERT_DOUBLE_EQ(x_min, -1.0);
  ASSERT_DOUBLE_EQ(x_max, +6.0);
  ASSERT_DOUBLE_EQ(y_min, -2.0);
  ASSERT_DOUBLE_EQ(y_max, +5.0);
}

TEST(polygon_region_set, xml1)
{
  kern::Polygon pgn1;
  pgn1.append_vtx(kern::Point{10, 10});
  pgn1.append_vtx(kern::Point{20, 10});
  pgn1.append_vtx(kern::Point{20, 20});
  pgn1.append_vtx(kern::Point{10, 20});
  kern::Polygon pgn2;
  pgn2.append_vtx(kern::Point{30, 30});
  pgn2.append_vtx(kern::Point{40, 30});
  pgn2.append_vtx(kern::Point{40, 40});
  pgn2.append_vtx(kern::Point{30, 40});
  kern::PolygonRegionSet set_write;
  set_write.unite(pgn1);
  set_write.unite(pgn2);
  tinyxml2::XMLDocument *xml_doc = new tinyxml2::XMLDocument();
  tinyxml2::XMLElement *xml_elt = nullptr;
  ASSERT_NO_THROW(xml_elt = set_write.to_xml_element(xml_doc));
  // xml_doc->InsertFirstChild(xml_elt);
  // xml_doc->SaveFile("polygon_region_set_xml1.xml");
  kern::PolygonRegionSet set_read{};
  set_read.from_xml_element(xml_elt);
  ASSERT_EQ(set_read.size(), set_write.size());
  // 1st region
  auto set_read_reg1 = set_read.get_region(0);
  auto set_read_outer1 = set_read_reg1.get_outer();
  auto set_read_inners1 = set_read_reg1.get_inners();
  ASSERT_EQ(set_read_outer1.size(), pgn1.size());
  EXPECT_EQ(*(set_read_outer1.get_vtx(0)), *(pgn1.get_vtx(0)));
  EXPECT_EQ(*(set_read_outer1.get_vtx(1)), *(pgn1.get_vtx(1)));
  EXPECT_EQ(*(set_read_outer1.get_vtx(2)), *(pgn1.get_vtx(2)));
  EXPECT_EQ(*(set_read_outer1.get_vtx(3)), *(pgn1.get_vtx(3)));
  ASSERT_EQ(set_read_inners1.size(), 0);
  // 2nd region
  auto set_read_reg2 = set_read.get_region(1);
  auto set_read_outer2 = set_read_reg2.get_outer();
  auto set_read_inners2 = set_read_reg2.get_inners();
  ASSERT_EQ(set_read_outer2.size(), pgn2.size());
  EXPECT_EQ(*(set_read_outer2.get_vtx(0)), *(pgn2.get_vtx(0)));
  EXPECT_EQ(*(set_read_outer2.get_vtx(1)), *(pgn2.get_vtx(1)));
  EXPECT_EQ(*(set_read_outer2.get_vtx(2)), *(pgn2.get_vtx(2)));
  EXPECT_EQ(*(set_read_outer2.get_vtx(3)), *(pgn2.get_vtx(3)));
  ASSERT_EQ(set_read_inners2.size(), 0);
}
