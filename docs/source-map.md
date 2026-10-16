# Source and cleanup notes

The 2025 course work is preserved in `archive/2025-course-scripts/`. The `.mlx` files were exported from their stored code and headings into plain `.m` files, without embedded figures, execution output, or Live Editor metadata. The maintained code in `matlab/` reorganizes these stages for portable paths and explicit configuration.

| Original source                 | Role                                                                                                                                       |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| `CoinProcessing.mlx`            | Baseline subtraction, filtering, CAD sampling, PCA initialization, point-to-plane ICP, residual analysis. Latest integrated course script. |
| `CoinProcessingSurfBased.mlx`   | Alternative exploratory surface-processing route. Preserved as historical source.                                                          |
| `ConProcessingReconstruction.m` | Missing-region extraction, convex-hull boundary, extrusion, and STL export.                                                                |
| `homogenomatrix.m`              | Checkerboard detection and SVD homography for XY calibration.                                                                              |
| `ICP.m`                         | Earlier combined processing/registration script; superseded by the integrated live script.                                                 |
| `final_script.mlx`              | Smaller prepared-PLY registration demonstration; the solver and error-analysis stages are represented in the maintained workflow.          |

## Cleanup decisions

- The maintained workflow writes generated files to `outputs/`, separates configuration from data, and uses exact filenames on case-sensitive filesystems.
- CAD surface sampling is seeded and bounded so reruns are repeatable and do not create unbounded point arrays.
- Registration uses one explicit transform convention, moving scan to fixed CAD. The original scripts mixed naming conventions and sometimes registered CAD to scan.
- PCA initialization is reorganized to avoid an original branch that referenced `alignedXYZ` before assignment. This is a code correction, not a claimed improvement in the historical experimental results.
- Repair geometry is an optional convex extrusion for the demonstrated class of shapes. Concave boundaries and internal holes require a more general reconstruction method.
- Full-resolution scans, duplicate archives, homework, autosaves, caches, and scratch exports are omitted. The one saved processed PLY is retained deliberately as a demo input.

## Original dependencies and attribution

The course folder included `xyzread.m` and `xyz2grid.m` by Chad A. Greene; `stlwrite2.m` credited Sven Holcombe and other contributors; and a G-code reader bundle by Tom Williamson. These helpers are not redistributed. The cleaned implementation uses MATLAB's native readers and STL routines. The Tom Williamson BSD license applies to that original G-code bundle and is not used as a license for this repository.

`final_script_Lisa.mlx` is a collaborator's variant and is not republished as Carter's individual work. Team credit is retained in the README and final presentation.

## Historical scripts

Files in `archive/2025-course-scripts/` are reference material, not the supported entry point. They retain interactive sections, dataset-specific constants, old output paths, and dependencies from the original coursework. Run the maintained entry point instead; in particular, the original homography script can overwrite a same-named XYZ file in its working directory.
