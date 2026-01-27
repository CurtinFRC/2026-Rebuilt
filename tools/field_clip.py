import trimesh

mesh = trimesh.load("C:\\Users\\paulh\\Downloads\\FE-2026- REBUILT™ Playing Field.stl")  # in meters

# Original bounding box extents and center
extents = mesh.extents          # [Lx, Ly, Lz]
center = mesh.centroid          # [cx, cy, cz]

Lx, Ly, Lz = extents

# Identify longer vs shorter horizontal side (assume horizontal axes are x,y)
if Lx >= Ly:
    long_axis = 0  # x
    short_axis = 1 # y
else:
    long_axis = 1  # y
    short_axis = 0 # x

crop_long = 3.005487
crop_short = 0.717554

# Start from original min/max in each axis
mins = mesh.bounds[0].copy()
maxs = mesh.bounds[1].copy()

# Cut from both ends along the long axis
mins[long_axis] += crop_long
maxs[long_axis] -= crop_long

# Cut from both sides along the short axis
mins[short_axis] += crop_short
maxs[short_axis] -= crop_short

# Create a box mesh with the desired bounds
box = trimesh.creation.box(extents=maxs - mins)
box.apply_translation((mins + maxs) / 2.0)

# Use its facet origins/normals as clipping planes
origins = box.facets_origin
normals = -box.facets_normal  # negative so you keep the inside

clipped = mesh.slice_plane(origins, normals)

clipped.export("o.stl")
