#include <gtest/gtest.h>

#include <mrnav/kernels/boost_geom/ik/double.hpp>

using kern = mrnav::kernels::boost_geom::Kernel_double;

TEST(polygon, default_)
{
  kern::Polygon pgn{};          // not simple, not valid
  EXPECT_EQ(pgn.size(), 0);
  EXPECT_FALSE(pgn.simple_p());
  EXPECT_FALSE(pgn.valid_p());
  EXPECT_ANY_THROW(pgn.get_vtx(0));
  pgn.append_vtx(kern::Point{0, 0}); // not simple, not valid
  EXPECT_FALSE(pgn.simple_p());
  EXPECT_FALSE(pgn.valid_p());
  pgn.append_vtx(kern::Point{1, 0}); // simple, not valid
  EXPECT_TRUE(pgn.simple_p());
  EXPECT_FALSE(pgn.valid_p());
  pgn.append_vtx(kern::Point{1, 1}); // simple, valid
  EXPECT_TRUE(pgn.simple_p());
  EXPECT_TRUE(pgn.valid_p());
  pgn.append_vtx(kern::Point{0, 1}); // simple, valid
  EXPECT_TRUE(pgn.simple_p());
  EXPECT_TRUE(pgn.valid_p());
}

TEST(polygon, simple1)
{
  // In this test, points are added in a counter-clock-wise manner,
  // and the evolution of the polygon's simplicity is tested.
  kern::Polygon pgn{};
  // not simple
  EXPECT_FALSE(pgn.simple_p());
  // not simple
  pgn.append_vtx(kern::Point{0, 0});
  EXPECT_FALSE(pgn.simple_p());
  // simple
  pgn.append_vtx(kern::Point{1, 0});
  EXPECT_TRUE(pgn.simple_p());
  // simple
  pgn.append_vtx(kern::Point{1, 1});
  EXPECT_TRUE(pgn.simple_p());
  // simple
  pgn.append_vtx(kern::Point{0, 1});
  EXPECT_TRUE(pgn.simple_p());
}

TEST(polygon, simple2)
{
  // In this test, points are added in a clock-wise manner,
  // and the evolution of the polygon's simplicity is tested.
  kern::Polygon pgn{};
  // not simple
  EXPECT_FALSE(pgn.simple_p());
  // not simple
  pgn.append_vtx(kern::Point{0, 0});
  EXPECT_FALSE(pgn.simple_p());
  // simple
  pgn.append_vtx(kern::Point{0, 1});
  EXPECT_TRUE(pgn.simple_p());
  // simple
  pgn.append_vtx(kern::Point{1, 1});
  EXPECT_TRUE(pgn.simple_p());
  // simple
  pgn.append_vtx(kern::Point{1, 0});
  EXPECT_TRUE(pgn.simple_p());
}

TEST(polygon, valid1)
{
  // In this test, points are added in a counter-clock-wise manner,
  // and the evolution of the polygon's validity is tested.
  kern::Polygon pgn{};
  // not valid
  EXPECT_FALSE(pgn.valid_p());
  // not valid
  pgn.append_vtx(kern::Point{0, 0});
  EXPECT_FALSE(pgn.valid_p());
  // not valid
  pgn.append_vtx(kern::Point{1, 0});
  EXPECT_FALSE(pgn.valid_p());
  // valid
  pgn.append_vtx(kern::Point{1, 1});
  EXPECT_TRUE(pgn.valid_p());
  // valid
  pgn.append_vtx(kern::Point{0, 1});
  EXPECT_TRUE(pgn.valid_p());
}

TEST(polygon, valid2)
{
  // In this test, points are added in a clock-wise manner,
  // and the evolution of the polygon's validity is tested.
  kern::Polygon pgn{};
  // not valid
  EXPECT_FALSE(pgn.valid_p());
  // not valid
  pgn.append_vtx(kern::Point{0, 0});
  EXPECT_FALSE(pgn.valid_p());
  // not valid
  pgn.append_vtx(kern::Point{0, 1});
  EXPECT_FALSE(pgn.valid_p());
  // not valid
  pgn.append_vtx(kern::Point{1, 1});
  EXPECT_FALSE(pgn.valid_p());
  // not valid
  pgn.append_vtx(kern::Point{1, 0});
  EXPECT_FALSE(pgn.valid_p());
}

TEST(polygon, modify1)
{
  kern::Polygon pgn{};
  kern::Point pnt1{0, 0}, pnt2{1, 1}, pnt3{2, 2};
  EXPECT_NO_THROW(pgn.insert_vtx(0, pnt1)); // Insert new vertex at the end (permissible)
  EXPECT_NO_THROW(pgn.append_vtx(pnt2));    // Append new vertex at the end (permissible)
  ASSERT_NO_THROW(pgn.get_vtx(1));          // Get 2nd vertex (exists)
  EXPECT_EQ((*pgn.get_vtx(0)), pnt1);
  EXPECT_EQ((*pgn.get_vtx(1)), pnt2);
  EXPECT_NE((*pgn.get_vtx(0)), pnt2);
  EXPECT_NE((*pgn.get_vtx(1)), pnt1);
  ASSERT_ANY_THROW(pgn.get_vtx(2)); // Get 3rd vertex (not exists)
  EXPECT_ANY_THROW(pgn.insert_vtx(10, pnt3)); // Insert new vertex beyond the end (impermissible)
  EXPECT_NO_THROW(pgn.insert_vtx(0, pnt3)); // Insert new vertex at the beginning (permissible)
  ASSERT_NO_THROW(pgn.remove_vtx(2));       // Remove 3rd vertex (exists)
  ASSERT_ANY_THROW(pgn.remove_vtx(2));      // Remove 3rd vertex (not exists)
  EXPECT_EQ(pgn.size(), 2);
  ASSERT_NO_THROW(pgn.remove_vtx(0)); // Remove 1st vertex (exists)
  EXPECT_EQ((*pgn.get_vtx(0)), pnt1);
  pgn.clear();
  ASSERT_EQ(pgn.size(), 0);
}

TEST(polygon, modify2)
{
  kern::Polygon pgn1{}, pgn2{};
  kern::Point pnt{1, 1};
  pgn1.append_vtx(kern::Point{0, 0});
  pgn2.append_vtx(kern::Point{0, 0});
  EXPECT_NO_THROW(pgn1.insert_vtx(0, pnt)); // Insert new vertex at the end (permissible)
  EXPECT_NO_THROW(pgn2.insert_vtx(1, pnt)); // Insert new vertex at the end (permissible)
  EXPECT_EQ(*(pgn1.get_vtx(0)), pnt);
  EXPECT_EQ(*(pgn2.get_vtx(1)), pnt);
}

TEST(polygon, contains_point_1)
{
  // Simple, counter-clock-wise oriented, square.
  kern::Polygon pgn{};
  pgn.append_vtx(kern::Point{0, 0});
  pgn.append_vtx(kern::Point{1, 0});
  pgn.append_vtx(kern::Point{1, 1});
  pgn.append_vtx(kern::Point{0, 1});
  EXPECT_TRUE(pgn.contains_p(kern::Point{0.5, 0.5}));
  EXPECT_FALSE(pgn.contains_p(kern::Point{1.5, 0.5}));
  EXPECT_FALSE(pgn.contains_p(kern::Point{0.5, 1.5}));
}

TEST(polygon, contains_point_2)
{
  // Simple, clock-wise oriented, square.
  kern::Polygon pgn{};
  pgn.append_vtx(kern::Point{0, 0});
  pgn.append_vtx(kern::Point{0, 1});
  pgn.append_vtx(kern::Point{1, 1});
  pgn.append_vtx(kern::Point{1, 0});
  EXPECT_FALSE(pgn.contains_p(kern::Point{0.5, 0.5}));
  EXPECT_TRUE(pgn.contains_p(kern::Point{1.5, 0.5}));
  EXPECT_TRUE(pgn.contains_p(kern::Point{0.5, 1.5}));
}

TEST(polygon, intersects_1)
{
  // Two overlapping squares.
  kern::Polygon pgn1{}, pgn2{};
  pgn1.append_vtx(kern::Point{0, 0});
  pgn1.append_vtx(kern::Point{2, 0});
  pgn1.append_vtx(kern::Point{2, 2});
  pgn1.append_vtx(kern::Point{0, 2});
  pgn2.append_vtx(kern::Point{1, 1});
  pgn2.append_vtx(kern::Point{3, 1});
  pgn2.append_vtx(kern::Point{3, 3});
  pgn2.append_vtx(kern::Point{1, 3});
  EXPECT_TRUE(pgn1.intersects_p(pgn2));
}


TEST(polygon, intersects_2)
{
  // Two non-overlapping squares.
  kern::Polygon pgn1{}, pgn2{};
  pgn1.append_vtx(kern::Point{0, 0});
  pgn1.append_vtx(kern::Point{2, 0});
  pgn1.append_vtx(kern::Point{2, 2});
  pgn1.append_vtx(kern::Point{0, 2});
  pgn2.append_vtx(kern::Point{4, 4});
  pgn2.append_vtx(kern::Point{6, 4});
  pgn2.append_vtx(kern::Point{6, 6});
  pgn2.append_vtx(kern::Point{4, 6});
  EXPECT_FALSE(pgn1.intersects_p(pgn2));
}

TEST(polygon, translate1)
{
  kern::Polygon pgn{};
  pgn.append_vtx(kern::Point{0, 0});
  pgn.append_vtx(kern::Point{1, 0});
  pgn.append_vtx(kern::Point{1, 1});
  pgn.append_vtx(kern::Point{0, 1});
  pgn.translate(kern::Point{2, 2});
  ASSERT_EQ(pgn.size(), 4);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(0)->x(), 2);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(0)->y(), 2);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(1)->x(), 3);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(1)->y(), 2);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(2)->x(), 3);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(2)->y(), 3);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(3)->x(), 2);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(3)->y(), 3);
}

TEST(polygon, reverse1)
{
  kern::Polygon pgn{};
  pgn.append_vtx(kern::Point{0, 0});
  pgn.append_vtx(kern::Point{1, 0});
  pgn.append_vtx(kern::Point{1, 1});
  pgn.append_vtx(kern::Point{0, 1});
  pgn.reverse();
  ASSERT_EQ(pgn.size(), 4);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(0)->x(), 0);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(0)->y(), 1);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(1)->x(), 1);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(1)->y(), 1);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(2)->x(), 1);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(2)->y(), 0);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(3)->x(), 0);
  EXPECT_DOUBLE_EQ(pgn.get_vtx(3)->y(), 0);
}

TEST(polygon, xml1)
{
  kern::Polygon pgn_write{};
  pgn_write.append_vtx(kern::Point{0, 0});
  pgn_write.append_vtx(kern::Point{1, 0});
  pgn_write.append_vtx(kern::Point{1, 1});
  pgn_write.append_vtx(kern::Point{0, 1});
  tinyxml2::XMLDocument *xml_doc = new tinyxml2::XMLDocument();
  tinyxml2::XMLElement *xml_elt = nullptr;
  ASSERT_NO_THROW(xml_elt = pgn_write.to_xml_element(xml_doc));
  // xml_doc->InsertFirstChild(xml_elt);
  // xml_doc->SaveFile("polygon_xml1.xml");
  kern::Polygon pgn_read{};
  ASSERT_NO_THROW(pgn_read.from_xml_element(xml_elt));
  ASSERT_EQ(pgn_read.size(), pgn_write.size());
  EXPECT_EQ(*(pgn_read.get_vtx(0)), *(pgn_write.get_vtx(0)));
  EXPECT_EQ(*(pgn_read.get_vtx(1)), *(pgn_write.get_vtx(1)));
  EXPECT_EQ(*(pgn_read.get_vtx(2)), *(pgn_write.get_vtx(2)));
  EXPECT_EQ(*(pgn_read.get_vtx(3)), *(pgn_write.get_vtx(3)));
}
