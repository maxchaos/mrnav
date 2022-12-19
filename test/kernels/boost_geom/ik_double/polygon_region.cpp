#include <gtest/gtest.h>

#include <mrnav/kernels/boost_geom/ik/double.hpp>

using kern = mrnav::kernels::boost_geom::Kernel_double;

TEST(polygon_region, default_)
{
  kern::PolygonRegion reg{};
  EXPECT_EQ(reg.get_outer().size(), 0);
  EXPECT_EQ(reg.get_inners().size(), 0);
  EXPECT_FALSE(reg.simple_p());
  EXPECT_FALSE(reg.valid_p());
}

TEST(polygon_region, outer_1)
{
  // Unit box, valid and simple.
  kern::PolygonRegion reg{};
  auto outer = reg.get_outer();
  outer.append_vtx(kern::Point{0, 0});
  outer.append_vtx(kern::Point{1, 0});
  outer.append_vtx(kern::Point{1, 1});
  outer.append_vtx(kern::Point{0, 1});
  // Assert size
  EXPECT_EQ(reg.get_outer().size(), 4);
  // Assert validity
  EXPECT_TRUE(reg.valid_p());
  // Assert simplicity
  EXPECT_TRUE(reg.simple_p());
  // Assert point inside.
  EXPECT_TRUE(reg.contains_p(kern::Point{0.5, 0.5}));
  // Assert points outside.
  EXPECT_FALSE(reg.contains_p(kern::Point{1.5, 0.5}));
  EXPECT_FALSE(reg.contains_p(kern::Point{0.5, 1.5}));
  EXPECT_FALSE(reg.contains_p(kern::Point{1.5, 1.5}));
}

TEST(polygon_region, outer_inner_1)
{
  // Unit box with a square hole.
  kern::PolygonRegion reg{};
  auto outer = reg.get_outer();
  outer.append_vtx(kern::Point{0, 0});
  outer.append_vtx(kern::Point{1, 0});
  outer.append_vtx(kern::Point{1, 1});
  outer.append_vtx(kern::Point{0, 1});
  auto inners = reg.get_inners();
  reg.get_inners().resize(1);
  ASSERT_EQ(inners.size(), 1);
  ASSERT_EQ(reg.get_inners().size(), 1);
  auto inner = reg.get_inners().get_ring(0);
  inner.append_vtx(kern::Point{0.25, 0.25});
  inner.append_vtx(kern::Point{0.25, 0.75});
  inner.append_vtx(kern::Point{0.75, 0.75});
  inner.append_vtx(kern::Point{0.75, 0.25});
  // Assert validity and simplicity.
  EXPECT_TRUE(reg.valid_p());
  EXPECT_TRUE(reg.simple_p());
  // Assert point inside.
  EXPECT_TRUE(reg.contains_p(kern::Point{0.1, 0.1}));
  EXPECT_TRUE(reg.contains_p(kern::Point{0.9, 0.9}));
  // Assert point inside.
  EXPECT_FALSE(reg.contains_p(kern::Point{0.5, 0.5}));
  EXPECT_FALSE(reg.contains_p(kern::Point{-0.1, -0.1}));
  EXPECT_FALSE(reg.contains_p(kern::Point{1.1, 1.1}));
}

TEST(polygon_region, xml1)
{
  // Unit box with a square hole.
  kern::PolygonRegion reg_write{};
  auto outer = reg_write.get_outer();
  outer.append_vtx(kern::Point{0, 0});
  outer.append_vtx(kern::Point{1, 0});
  outer.append_vtx(kern::Point{1, 1});
  outer.append_vtx(kern::Point{0, 1});
  auto inners = reg_write.get_inners();
  inners.resize(1);
  auto inner = reg_write.get_inners().get_ring(0);
  inner.append_vtx(kern::Point{0.25, 0.25});
  inner.append_vtx(kern::Point{0.25, 0.75});
  inner.append_vtx(kern::Point{0.75, 0.75});
  inner.append_vtx(kern::Point{0.75, 0.25});
  tinyxml2::XMLDocument *xml_doc = new tinyxml2::XMLDocument();
  tinyxml2::XMLElement *xml_elt = nullptr;
  ASSERT_NO_THROW(xml_elt = reg_write.to_xml_element(xml_doc));
  // xml_doc->InsertFirstChild(xml_elt);
  // xml_doc->SaveFile("polygon_region_xml1.xml");
  kern::PolygonRegion reg_read{};
  reg_read.from_xml_element(xml_elt);
  auto reg_read_outer = reg_read.get_outer();
  ASSERT_EQ(reg_read_outer.size(), outer.size());
  EXPECT_EQ(*(reg_read_outer.get_vtx(0)), *(outer.get_vtx(0)));
  EXPECT_EQ(*(reg_read_outer.get_vtx(1)), *(outer.get_vtx(1)));
  EXPECT_EQ(*(reg_read_outer.get_vtx(2)), *(outer.get_vtx(2)));
  EXPECT_EQ(*(reg_read_outer.get_vtx(3)), *(outer.get_vtx(3)));
  auto reg_read_inners = reg_read.get_inners();
  ASSERT_EQ(reg_read_inners.size(), inners.size());
  auto reg_read_inner = inners.get_ring(0);
  ASSERT_EQ(reg_read_inner.size(), inner.size());
  EXPECT_EQ(*(reg_read_inner.get_vtx(0)), *(inner.get_vtx(0)));
  EXPECT_EQ(*(reg_read_inner.get_vtx(1)), *(inner.get_vtx(1)));
  EXPECT_EQ(*(reg_read_inner.get_vtx(2)), *(inner.get_vtx(2)));
  EXPECT_EQ(*(reg_read_inner.get_vtx(3)), *(inner.get_vtx(3)));
}
