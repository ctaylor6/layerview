# Project data

`processed/filtered_scan.ply` is an unchanged copy of the saved `filteredPC1.ply` output in the course folder. It is a processed scan used to make the registration demonstration accessible without distributing the full raw dataset. It does not exercise raw calibration or filtering, and the course scripts do not establish a unique raw-input lineage for this saved file.

`cad/` contains the original feature, base, and filled-feature STL models. Their original uppercase `.STL` extension is preserved for case-sensitive filesystems. `calibration/checkerboardwarp.png` is the original checkerboard image used by the calibration script.

[`manifest.json`](manifest.json) records original relative filenames, published filenames, file sizes, and SHA-256 checksums for these inputs. Inputs are copied without rescaling or mesh modification. The course code and presentation label the analysis coordinates in millimeters; STL and PLY files themselves do not enforce physical units. Check coordinate scales when substituting scans.

## Full scans

The original folder contains baseline, defective-part, and repair XYZ datasets. Individual files exceed GitHub's 100 MB file limit, and several are duplicates. They are intentionally excluded from Git along with the duplicate ZIP archive. Keep full-resolution files under a local `data/raw/` directory or supply an external data directory to the pipeline.

The latest course script used `Repair Homog/2_1.xyz` as its baseline and `Repair Homog/2_2.xyz` as the part scan. These are already homography-transformed coordinates. The cleaned raw-input workflow expects finite numeric XYZ columns and does not repeat phase reconstruction.
