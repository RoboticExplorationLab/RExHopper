# How to Generate MJCF from the SolidWorks model
(Make sure the model is up-to-date and the URDF reference sketch is still correctly constrained)
1. Install SW2URDF
2. Tools > Export to URDF
3. Install Meshlab and Open
4. Import base_link.STL
5. Filters > Remeshing, Simplification and Reconstruction > Quadric Edge Collapse Decimation
6. Select:
   - Preserve Normal
   - Preserve Topology
7. Number of Faces <= 200000 (This will keep file size below 10 Mb)
8. Save as base_link.stl
    Repeat for all links
9. Put meshes into new folder called hopper_rev0X
10. Copy old MJCF xml file with new name
Or:  if making from scratch, you can convert from URDF: 
    ```
    cd ~/mujoco/build_dir/bin
    ./compile /path/to/model.urdf /path/to/model.xml
    ```

11. Copy CoM positions, quaternions, axes, etc. into the new MJCF
12. Go into SW and check the subassy inertia values. SW2URDF has a bug that makes the inertia matrices wrong so you will need to replace all of the inertia matrix values in diaginertia in the MJCF
   - Use the custom coordinate systems to make sure it matches your math "Report coordinate values taken relative to: Coordinate System 1"
   -  Use "taken at the center of mass and aligned with the output coordinate system"
   -  (Ixx, Iyy, Izz) 
   - Units: kg * m2
   - Also update base mass, CoM pos, etc.