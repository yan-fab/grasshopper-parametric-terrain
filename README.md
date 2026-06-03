# Grasshopper Parametric Terrain & Accessible Paths

A Grasshopper (RhinoCommon/Python) tool to generate parametric undulating terrains (mounds) with integrated accessible pathways (1:12 slope constraint) and automatic mesh/surface clipping to prevent z-fighting.

## 🚀 Features

- **Smooth Gaussian Falloff**: Generates organic, natural terrain mounds from control points (attractors).
- **Boundary Fade (Smoothstep)**: Ensures the terrain height smoothly transitions to zero exactly at the defined boundary, preventing sharp edges.
- **Accessible Ramp (Double-Pass Algorithm)**: Analyzes the centerline of the path and corrects heights step-by-step to strictly respect the maximum slope limit of **8.33%** (1:12 slope, NBR 9050 / ADA compliant).
- **Automatic 2D/3D Puzzle-Fit Clipping**: Trims the path area directly out of the terrain using stable 2D curve booleans, completely eliminating mesh overlap (z-fighting).

## 📁 File Structure

- `grasshopper_terrain_path_base.py`: Main Python source code to paste inside the GhPython component.

## 🛠️ How to Use in Grasshopper

1. Add a **Python 3** or **GhPython** component to the Grasshopper canvas.
2. Configure the following parameters as **Inputs** (right-click each input to set the Type Hint and Access):
   - `C` (Type Hint: `Curve`, Item Access): Boundary curve of the lawn area.
   - `P` (Type Hint: `Point3d`, List Access): Peak/attractor points.
   - `H` (Type Hint: `float`, Item/List Access): Maximum peak height.
   - `R` (Type Hint: `float`, Item/List Access): Influence radius for each peak.
   - `F` (Type Hint: `float`, Item Access): Fade/transition distance at the boundary.
   - `N` (Type Hint: `int`, Item Access): Grid resolution (e.g., 50).
   - `CRV` (Type Hint: `Curve`, Item Access, Optional): Central axis curve of the path.
   - `W` (Type Hint: `float`, Item Access, Optional): Total path width (e.g., 2.0).
   - `B` (Type Hint: `float`, Item Access, Optional): Buffer/side slope transition width.
   - `MAX_S` (Type Hint: `float`, Item Access, Optional): Maximum allowed slope (Default: 0.0833).
3. Configure the following parameters as **Outputs**:
   - `S`: Trimmed NURBS surface of the terrain (Brep).
   - `M`: Trimmed mesh of the terrain.
   - `PM`: Trimmed 3D mesh of the path.
   - `PC`: Trimmed boundary curves of the path.
4. Paste the code from `grasshopper_terrain_path_base.py` into the editor.
