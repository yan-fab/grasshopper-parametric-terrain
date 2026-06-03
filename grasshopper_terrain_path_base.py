"""
TEMPLATE BASE PARA TERRENOS PARAMÉTRICOS COM CAMINHOS ACESSÍVEIS (GhPython)
==========================================================================
Configure o componente GhPython com os seguintes parâmetros:

Inputs:
    C: Curva de base do limite (Type hint: Curve, Item Access)
    P: Pontos dos picos/atratores (Type hint: Point3d, List Access)
    H: Altura(s) dos picos (Type hint: float, Item/List Access)
    R: Raio(s) de influência (Type hint: float, Item/List Access)
    F: Distância de suavização na borda (Type hint: float, Item Access)
    N: Resolução da grade (Type hint: int, Item Access)
    CRV: Eixo do caminho (Type hint: Curve, Item Access, Opcional)
    W: Largura total do caminho (Type hint: float, Item Access, Opcional)
    B: Largura da transição/talude lateral (Type hint: float, Item Access, Opcional)
    MAX_S: Inclinação máxima permitida (Type hint: float, Item Access, Opcional - Ex: 0.0833 para 8.33%)

Outputs:
    S: Superfície(s) NURBS do terreno (Brep)
    M: Malha (Mesh) do terreno recortada
    PM: Malha (Mesh) 3D do caminho recortada
    PC: Curvas 3D das bordas do caminho recortadas
"""

import Rhino.Geometry as rg
import math
from System.Collections.Generic import List

# ==========================================
# 1. FUNÇÕES AUXILIARES DE GEOMETRIA
# ==========================================

def trim_curve_to_boundary(curve_3d, flat_boundary, tolerance=0.001):
    """Recorta uma curva 3D mantendo apenas os trechos dentro do limite 2D."""
    flat_c = rg.Curve.Duplicate(curve_3d)
    xform = rg.Transform.PlanarProjection(rg.Plane.WorldXY)
    flat_c.Transform(xform)
    
    events = rg.Intersect.Intersection.CurveCurve(flat_c, flat_boundary, tolerance, tolerance)
    if not events or len(events) == 0:
        mid_pt = curve_3d.PointAt(curve_3d.Domain.Mid)
        mid_pt_xy = rg.Point3d(mid_pt.X, mid_pt.Y, 0.0)
        if flat_boundary.Contains(mid_pt_xy, rg.Plane.WorldXY, tolerance) in [rg.PointContainment.Inside, rg.PointContainment.Coincident]:
            return curve_3d
        return None
        
    split_params = [ev.ParameterA for ev in events]
    segments = curve_3d.Split(split_params)
    if not segments:
        return curve_3d
        
    inside_segments = []
    for seg in segments:
        mid_pt = seg.PointAt(seg.Domain.Mid)
        mid_pt_xy = rg.Point3d(mid_pt.X, mid_pt.Y, 0.0)
        if flat_boundary.Contains(mid_pt_xy, rg.Plane.WorldXY, tolerance) in [rg.PointContainment.Inside, rg.PointContainment.Coincident]:
            inside_segments.append(seg)
            
    if not inside_segments:
        return None
    elif len(inside_segments) == 1:
        return inside_segments[0]
    else:
        joined = rg.Curve.JoinCurves(inside_segments, tolerance)
        return joined[0] if (joined and len(joined) > 0) else inside_segments[0]


def get_terrain_z(x, y, bbox_min_z, flat_curve, peaks, h_list, r_list, fade_dist):
    """Calcula a altura Gaussiana com atenuação suave (smoothstep) na borda."""
    z_val = bbox_min_z
    raw_z = 0.0
    for pk, h, r in zip(peaks, h_list, r_list):
        dist = math.sqrt((x - pk.X)**2 + (y - pk.Y)**2)
        raw_z += h * math.exp(-((dist / r) ** 2))
        
    containment = flat_curve.Contains(rg.Point3d(x, y, 0.0), rg.Plane.WorldXY, 0.001)
    if containment in [rg.PointContainment.Inside, rg.PointContainment.Coincident]:
        success, t = flat_curve.ClosestPoint(rg.Point3d(x, y, 0.0))
        if success:
            closest_pt = flat_curve.PointAt(t)
            dist_to_edge = rg.Point3d(x, y, 0.0).DistanceTo(closest_pt)
            if fade_dist > 0.0:
                t_fade = min(1.0, dist_to_edge / fade_dist)
                weight = 3 * (t_fade ** 2) - 2 * (t_fade ** 3)  # Smoothstep
            else:
                weight = 1.0
            z_val += raw_z * weight
        else:
            z_val += raw_z
    return z_val

# ==========================================
# 2. ALGORITMO PRINCIPAL DE MODELAGEM
# ==========================================

def gerar_modelo(base_curve, peaks, heights, radii, fade_dist, res, path_curve, path_width, buffer_width, max_slope=0.0833):
    bbox = base_curve.GetBoundingBox(True)
    min_pt, max_pt = bbox.Min, bbox.Max
    dx, dy = max_pt.X - min_pt.X, max_pt.Y - min_pt.Y
    num_peaks = len(peaks)
    
    # 2.1. Normalização de Listas de Altura (H) e Raio (R)
    if not isinstance(heights, list):
        h_in = [heights]
    else:
        h_in = list(heights)
    if len(h_in) == 1:
        h_max = h_in[0]
        h_list = [h_max * s for s in [1.0, 0.72, 0.48]]  # Escalonamento automático
        h_list += [h_list[-1]] * (num_peaks - len(h_list))
    else:
        h_list = h_in + [h_in[-1]] * (num_peaks - len(h_in))
        
    if not isinstance(radii, list):
        r_in = [radii]
    else:
        r_in = list(radii)
    if len(r_in) == 1:
        r_max = r_in[0]
        r_list = [r_max * s for s in [1.0, 0.85, 0.70]]  # Escalonamento automático
        r_list += [r_list[-1]] * (num_peaks - len(r_list))
    else:
        r_list = r_in + [r_in[-1]] * (num_peaks - len(r_in))

    flat_curve = rg.Curve.Duplicate(base_curve)
    flat_curve.Translate(rg.Vector3d(0, 0, -min_pt.Z))

    graded_path_curve = None
    path_mesh_out = rg.Mesh()
    path_borders_out = []
    left_pts_3d, right_pts_3d = [], []
    
    # 2.2. Processar o Caminho e Limitar Inclinação (Double Pass)
    if path_curve:
        num_path_pts = 150
        ts = path_curve.DivideByCount(num_path_pts, True)
        path_pts = [path_curve.PointAt(t) for t in ts]
        
        path_z = [get_terrain_z(pt.X, pt.Y, min_pt.Z, flat_curve, peaks, h_list, r_list, fade_dist) for pt in path_pts]
        
        # Ida (Forward Pass)
        for i in range(1, num_path_pts):
            ds = math.sqrt((path_pts[i].X - path_pts[i-1].X)**2 + (path_pts[i].Y - path_pts[i-1].Y)**2)
            dz_max = ds * max_slope
            path_z[i] = max(path_z[i-1] - dz_max, min(path_z[i-1] + dz_max, path_z[i]))
            
        # Volta (Backward Pass)
        for i in range(num_path_pts - 2, -1, -1):
            ds = math.sqrt((path_pts[i].X - path_pts[i+1].X)**2 + (path_pts[i].Y - path_pts[i+1].Y)**2)
            dz_max = ds * max_slope
            path_z[i] = max(path_z[i+1] - dz_max, min(path_z[i+1] + dz_max, path_z[i]))
            
        path_pts_3d = [rg.Point3d(path_pts[i].X, path_pts[i].Y, path_z[i]) for i in range(num_path_pts)]
        graded_path_curve = rg.Curve.CreateInterpolatedCurve(path_pts_3d, 3)
        
        # Gerar pontos das laterais
        half_w = path_width / 2.0
        for i in range(num_path_pts):
            tangent = path_pts_3d[1] - path_pts_3d[0] if i == 0 else (path_pts_3d[i] - path_pts_3d[i-1] if i == num_path_pts - 1 else path_pts_3d[i+1] - path_pts_3d[i-1])
            tangent.Z = 0.0
            tangent.Unitize()
            normal = rg.Vector3d(-tangent.Y, tangent.X, 0.0)
            
            left_pts_3d.append(path_pts_3d[i] + normal * half_w)
            right_pts_3d.append(path_pts_3d[i] - normal * half_w)
            
        left_border = rg.Curve.CreateInterpolatedCurve(left_pts_3d, 3)
        right_border = rg.Curve.CreateInterpolatedCurve(right_pts_3d, 3)
        
        left_trimmed = trim_curve_to_boundary(left_border, flat_curve)
        right_trimmed = trim_curve_to_boundary(right_border, flat_curve)
        if left_trimmed: path_borders_out.append(left_trimmed)
        if right_trimmed: path_borders_out.append(right_trimmed)
        
        # Gerar Malha do Caminho recortada no limite
        path_loft = rg.Brep.CreateFromLoft([left_border, right_border], rg.Point3d.Unset, rg.Point3d.Unset, rg.LoftType.Normal, False)
        if path_loft:
            brep_path = path_loft[0]
            proj_path = rg.Curve.ProjectToBrep(base_curve, brep_path, rg.Vector3d.ZAxis, 0.001)
            if proj_path:
                cutters_path = List[rg.Curve]()
                for c in proj_path: cutters_path.Add(c)
                split_path = brep_path.Split(cutters_path, 0.001)
                if split_path:
                    for bp in split_path:
                        centroid = bp.GetBoundingBox(True).Center
                        if flat_curve.Contains(rg.Point3d(centroid.X, centroid.Y, 0.0), rg.Plane.WorldXY, 0.001) in [rg.PointContainment.Inside, rg.PointContainment.Coincident]:
                            m_parts = rg.Mesh.CreateFromBrep(bp, rg.MeshingParameters.Default)
                            if m_parts:
                                for m in m_parts: path_mesh_out.Append(m)
                            break

    # 2.3. Gerar Grade de Pontos (Terreno nivelado com rampa)
    grid_pts = []
    u_count, v_count = res + 1, res + 1
    for i in range(u_count):
        x = min_pt.X + dx * (i / float(res))
        for j in range(v_count):
            y = min_pt.Y + dy * (j / float(res))
            z_orig = get_terrain_z(x, y, min_pt.Z, flat_curve, peaks, h_list, r_list, fade_dist)
            pt = rg.Point3d(x, y, z_orig)
            
            if graded_path_curve:
                success, t_path = graded_path_curve.ClosestPoint(pt)
                if success:
                    closest_pt = graded_path_curve.PointAt(t_path)
                    dist = math.sqrt((pt.X - closest_pt.X)**2 + (pt.Y - closest_pt.Y)**2)
                    half_w = path_width / 2.0
                    if dist <= half_w:
                        pt.Z = closest_pt.Z
                    elif dist <= half_w + buffer_width:
                        t_blend = (dist - half_w) / buffer_width
                        weight = 3 * (t_blend ** 2) - 2 * (t_blend ** 3)
                        pt.Z = closest_pt.Z * (1.0 - weight) + z_orig * weight
            grid_pts.append(pt)
            
    # 2.4. Superfície Base e Diferença Booleana 2D
    srf = rg.NurbsSurface.CreateFromPoints(grid_pts, u_count, v_count, 3, 3)
    brep = rg.Brep.CreateFromSurface(srf)
    
    terrain_2d_curves = [flat_curve]
    if graded_path_curve and left_pts_3d:
        path_outline_pts = [rg.Point3d(p.X, p.Y, 0.0) for p in left_pts_3d] + [rg.Point3d(p.X, p.Y, 0.0) for p in reversed(right_pts_3d)] + [rg.Point3d(left_pts_3d[0].X, left_pts_3d[0].Y, 0.0)]
        path_outline_curve = rg.Polyline(path_outline_pts).ToNurbsCurve()
        subtracted = rg.Curve.CreateBooleanDifference(flat_curve, path_outline_curve)
        if subtracted: terrain_2d_curves = subtracted

    # 2.5. Projetar e Recortar o Terreno Final (com o furo para o caminho)
    proj_curves = []
    for tc in terrain_2d_curves:
        proj_tc = rg.Curve.ProjectToBrep(tc, brep, rg.Vector3d.ZAxis, 0.001)
        if proj_tc: proj_curves.extend(proj_tc)
            
    srf_final = brep
    if proj_curves:
        cutters = List[rg.Curve]()
        for c in proj_curves: cutters.Add(c)
        split_breps = brep.Split(cutters, 0.001)
        if split_breps:
            kept_pieces = []
            for b in split_breps:
                centroid = b.GetBoundingBox(True).Center
                centroid_xy = rg.Point3d(centroid.X, centroid.Y, 0.0)
                is_inside = any(tc.Contains(centroid_xy, rg.Plane.WorldXY, 0.001) in [rg.PointContainment.Inside, rg.PointContainment.Coincident] for tc in terrain_2d_curves)
                if is_inside: kept_pieces.append(b)
            if kept_pieces:
                srf_final = rg.Brep()
                for kp in kept_pieces: srf_final.Append(kp)

    # 2.6. Criar Malha (Mesh) Recortada
    mesh = rg.Mesh()
    mesh_parts = rg.Mesh.CreateFromBrep(srf_final, rg.MeshingParameters.Default)
    if mesh_parts:
        for m in mesh_parts: mesh.Append(m)
            
    return srf_final, mesh, path_mesh_out, path_borders_out

# ==========================================
# 3. EXECUÇÃO
# ==========================================
if 'C' in globals() and 'P' in globals() and C is not None and P is not None:
    res = N if ('N' in globals() and N is not None) else 50
    h_vals = H if ('H' in globals() and H is not None) else 5.0
    r_vals = R if ('R' in globals() and R is not None) else 15.0
    f_val = F if ('F' in globals() and F is not None) else 8.0
    p_list = P if isinstance(P, list) else [P]
    
    path_crv = CRV if ('CRV' in globals() and CRV is not None) else None
    p_width = W if ('W' in globals() and W is not None) else 2.0
    b_width = B if ('B' in globals() and B is not None) else 3.0
    slope_limit = MAX_S if ('MAX_S' in globals() and MAX_S is not None) else 0.0833
    
    S, M, PM, PC = gerar_modelo(C, p_list, h_vals, r_vals, f_val, res, path_crv, p_width, b_width, slope_limit)
