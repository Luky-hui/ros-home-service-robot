include "bobac3_2d.lua"


TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.global_sampling_ratio = 0.0003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03

return options