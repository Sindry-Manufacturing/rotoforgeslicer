# PyInstaller hooks

Drop `hook-<package>.py` files here for extra hidden-imports or data files that
`collect_all` misses on a given OS (trimesh/shapely/rtree/Qt are the usual
suspects). Referenced via `hookspath=["packaging/hooks"]` in
`rotoforge_slicer.spec`.
