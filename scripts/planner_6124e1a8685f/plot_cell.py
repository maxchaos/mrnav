#!/usr/bin/python3

import sys

import numpy
import matplotlib.pyplot as plt
import matplotlib.patches
import matplotlib.collections
import matplotlib.colors
import lxml.etree as xml

COLORMAP = plt.get_cmap('tab10')

def plot_polygon(ax, xml_pgn, color, return_patch=False):
    """TODO"""
    vertices = numpy.array(tuple((xml_vtx.get('x'), xml_vtx.get('y'))
                                 for xml_vtx in xml_pgn.xpath("vertex")),
                           dtype=float)
    patch = matplotlib.patches.Polygon(vertices, closed=True, color=color)
    if return_patch:
        return patch
    else:
        pc = matplotlib.collections.PatchCollection((patch,),
                                                    match_original=True)
        ax.add_collection(pc)

def plot_polygon_region(ax, xml_reg, color, return_patches=False):
    """TODO"""
    xml_outer = xml_reg.xpath('outer/polygon')[0]
    patch_outer = plot_polygon(ax, xml_outer, color=color, return_patch=True)
    patch_inners = []
    for xml_inner in xml_reg.xpath('inners/polygon'):
        patch_inners.append(
            plot_polygon(ax, xml_inner, color='white', return_patch=True))
    patches = []
    patches.append(patch_outer)
    patches.extend(patch_inners)
    if return_patches:
        return patches
    else:
        pc = matplotlib.collections.PatchCollection(patches, match_original=True)
        ax.add_collection(pc)

def plot_polygon_region_set(ax, xml_set, color):
    """TODO"""
    patches = []
    for xml_reg in xml_set.xpath("polygon-region"):
        patches.extend(
            plot_polygon_region(ax, xml_reg, color, return_patches=True))
    pc = matplotlib.collections.PatchCollection(patches, match_original=True)
    ax.add_collection(pc)

def get_robot_color(idx):
    """TODO"""
    # return matplotlib.colors.hsv_to_rgb(
    #     numpy.array((float(COLORMAP.colors[idx]), 1.0, 1.0)))
    color = numpy.ones(4)
    color[:-1] = COLORMAP.colors[idx]
    color[-1] = 0.5
    return color

def xml_ccell_get(refid, xml_ccells):
    """TODO"""
    return xml_ccells.xpath("cell[@id='{}']".format(refid))[0]

def xml_ccell_components_get(xml_cc, xml_hier):
    """TODO"""
    xml_components = []
    for refid in xml_cc.xpath("components/simple-cell/@refid"):
        xml_components.append(
            xml_hier.xpath("//simple-cell[@id='{}']".format(refid))[0])
    return xml_components

def plot_scell(ax, xml_sc, color):
    """TODO"""
    xml_shape = xml_sc.xpath("shape")[0]
    xml_rfpoa = xml_sc.xpath("rfpoa")[0]
    color_oa = color
    color_oa[3] = 0.2
    plot_polygon_region_set(ax, xml_rfpoa, color)
    plot_polygon_region_set(ax, xml_shape, color)

def plot_ccell(ax, xml_cc, xml_hier):
    """TODO"""
    xml_components = xml_ccell_components_get(xml_cc, xml_hier)
    for idx, xml_comp in enumerate(xml_components):
        color = get_robot_color(idx)
        a = ax[idx] if isinstance(ax, numpy.ndarray) else ax
        plot_scell(a, xml_comp, color=color)

if __name__ == "__main__":
    with open(sys.argv[1], 'r') as f:
        xml_hier = xml.fromstring(f.read().encode("utf-8"))
    with open(sys.argv[2], 'r') as f:
        xml_ccells = xml.fromstring(f.read().encode("utf-8"))
    # xml_hier = xml_tree.xpath("hierarchy")[0]
    # xml_ccells = xml_tree.xpath("compound-cells")[0]
    dim = len(xml_hier.xpath("simple-hierarchy"))
    # fix, ax = plt.subplots(1, dim)
    # for a in ax:
    #     a.axis([0, 100, -100, 0], option='auto')
    fix, ax = plt.subplots()
    ax.axis([0, 100, -100, 0], option='auto')
    xml_cc = xml_ccell_get(sys.argv[3], xml_ccells)
    plot_ccell(ax, xml_cc, xml_hier)
    plt.show()
