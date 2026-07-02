"""Studio: the 3D build-plate GUI (M11 + kinematic simulation). SPEC §9.

A pyvista/pyvistaqt viewport on top of the existing, validated core: place and
transform meshes on the simulated build plate (``scene``), slice them through the
normal pipeline, view the tagged toolpath in 3D, and play back a time-parameterized
kinematic simulation with live process readouts (``simulate``).

Importing this package stays light — pyvista / PySide6 / trimesh are pulled lazily
inside the functions that use them (CLAUDE.md).
"""
