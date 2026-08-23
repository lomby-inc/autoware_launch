# `compare_map_segmentation` on the real robot

`pointcloud_map_filter.param.yaml` is the config the **robot** uses. It almost
certainly is not masking anything today, and the failure is silent. This note
says how to confirm that, what to change, and what the change costs.

Nothing here has been applied to the robot. The sim runs its own copy,
`pointcloud_map_filter_sim.param.yaml`, and the robot's file is untouched.

## What this filter is for

It deletes lidar points that coincide with the pointcloud map, so permanent
structure — walls, facades, fences — never reaches `euclidean_cluster` and never
becomes an object. Without it, and with a rule-based detector that labels
everything UNKNOWN, walls arrive as objects indistinguishable from a pedestrian.
Measured in the sim before the fix: ~35 tracked objects of pure scenery noise
with the robot parked. After: ~1.

## Why it silently does nothing

The z leaf of the per-map-cell voxel grid is derived, not configured:

```
z leaf size       = down_sample_voxel_size * downsize_ratio_z_axis
z match tolerance = distance_threshold     * downsize_ratio_z_axis
```

At `down_sample_voxel_size: 0.05` and `downsize_ratio_z_axis: 0.05` the z leaf is
**2.5 mm**. PCL's `VoxelGrid::applyFilter` then refuses the job: when
`dx*dy*dz` for a cell's bounding box would overflow int32 it prints a warning,
does `output = *input_`, and **saves no leaf layout**. Every subsequent
`getCentroidIndexAt()` returns −1, so `is_close_to_map()` is false for every
point and nothing is ever removed.

The node still starts, still logs `VoxelGridDynamicMapLoader initialized.`, still
publishes at the right rate on the right frame. It just passes everything
through. That is the whole reason this went unnoticed.

## Confirm it on the robot first

Do not change anything until you have seen the no-op. Compare the filter's input
and output point counts:

```
ros2 topic hz /perception/obstacle_segmentation/pointcloud_map_filtered/downsampled/pointcloud
ros2 topic hz /perception/object_recognition/detection/pointcloud_map_filtered/pointcloud
```

Rates being equal proves nothing — compare `width`. If the output count is
identical to the input, the filter is a pass-through. In the sim the two read
`2805` and `2805`; after the fix the output dropped to `3`.

Also worth grepping the launch output for PCL's own words:

```
Leaf size is too small for the input dataset
```

## What has to change

Two things, and **one alone is not enough**.

### 1. The map must be tiled

`VoxelGridDynamicMapLoader` derives its cell pitch from a cell's own extent
(`map_grid_size_x_ = metadata.max_x - metadata.min_x`), so a single-file map
yields a pitch as wide as the whole map and the cell lookup degenerates. It also
voxelizes each cell whole, so one large cell is what overflows int32.

The robot's map needs `pointcloud_map_metadata.yaml` plus a directory of tiles.
If it already has them, check the pitch is small enough with the budget below.
If it does not, `sim_test/tools/tile_pointcloud_map.py` in the `lomby_nav2` repo
will generate both:

```
sim_test/tools/tile_pointcloud_map.py <robot_map>.pcd --dry-run --check-leaf 0.05,0.6
```

`--dry-run` writes nothing and prints the budget. Drop it to generate. The
original `.pcd` is never modified, and every input point is accounted for.

### 2. `downsize_ratio_z_axis` must be workable

```yaml
downsize_ratio_z_axis: 0.6      # was 0.05
```

`0.6` is Autoware's own default: a 30 mm z leaf and a 324 mm z tolerance.

Keep `down_sample_voxel_size: 0.05` and `distance_threshold: 0.54` as they are.

## The budget you have to satisfy

For each tile, with `E` the tile's z extent and `T` the tile pitch:

```
(T / leaf) * (T / leaf) * (E / (leaf * ratio_z))  <  2^31
```

Worked example from the sim's map — 906 x 1095 x 33.6 m, 4.64 M points:

| tile | leaf | ratio_z | z leaf | dx*dy*dz | |
|---|---|---|---|---|---|
| untiled | 0.05 | 0.05 | 2.5 mm | 5,329,218,824,622 | 2481x over |
| untiled | 0.1 | 0.6 | 60 mm | 55,552,959,840 | 26x over |
| 50 m | 0.05 | 0.6 | 30 mm | 782,562,781 | ok, 2.7x margin |

Note row two: **Autoware's stock defaults also overflow** on a map this size.
Tiling is not optional, and neither is checking the number for your own map.

## What the change costs

Anything within `distance_threshold` of mapped structure is deleted —
**54 cm horizontally, 32 cm vertically** with these values. A pedestrian standing
against a facade can be erased along with the facade.

In the sim, a pedestrian 6 m ahead in open space survives and is tracked to
0.01 m. The against-a-wall case is **not yet characterised** — test it on the
robot before trusting the filter near buildings. If it does erase people,
lowering `distance_threshold` trades masking quality for that margin.

Also note the filter only removes what is *in the map*. Parked cars, bins, road
works and anything else added since the survey still arrive as objects. This
reduces noise; it does not classify.

## Rollback

Restore `downsize_ratio_z_axis: 0.05` and the filter reverts to a no-op, i.e.
exactly today's behaviour. No map changes need undoing — a tiled map with the old
ratio behaves the same as an untiled one, because both overflow.
