include "multi_lidar_quality.lua"

-- Keep historical geometry until at least two fresher finished submaps cover
-- it. Zero area threshold keeps any old submap with unique observed cells.
-- No pure-localization trimmer: unvisited old areas must not be discarded.
POSE_GRAPH.overlapping_submaps_trimmer_2d = {
  fresh_submaps_count = 2,
  min_covered_area = 0.0,
  min_added_submaps_count = 2,
}
return options
