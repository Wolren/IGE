"""
liriap — Inscribed Geometry Engine

Fast axis-aligned and oriented largest-inscribed-rectangle solvers
powered by a native Rust backend (Daniels et al. 1997).

Usage
-----
    >>> from liriap import solve_axis_aligned
    >>> rect = solve_axis_aligned([(0, 0), (10, 0), (10, 10), (0, 10)])
    >>> rect.area
    100.0

    >>> from liriap import solve_oriented_lir
    >>> rect = solve_oriented_lir([(0,0), (8,1), (7,7), (2,8), (-1,4)])

Install
-------
    pip install liriap

QGIS Plugin
-----------
    Zip the ``qgis_plugin/LIRiAP/`` folder and install via
    Plugin Manager → Install from ZIP.
    Two processing algorithms appear under "LIRiAP".

    If the native ``liriap`` wheel is not yet built for your platform,
    the algorithms display a helpful install message instead of crashing.
"""

from __future__ import annotations

# Re-export the native Rust functions with a clean doc API.
# The actual implementations live in liriap._native (the compiled cdylib).
from liriap._native import (
    solve_axis_aligned_py,
    solve_oriented_lir_py,
    solve_bcrs_py,
    solve_mic_py,
    axis_aligned_demo,
    oriented_lir_demo,
)

from typing import List, Optional, Tuple


class AxisAlignedResult:
    """Result of an axis-aligned rectangle solve.

    Attributes
    ----------
    x_min, y_min, x_max, y_max : float
        Bounding coordinates of the inscribed rectangle.
    area : float
        Area of the rectangle (width × height).
    """

    def __init__(self, x_min: float, y_min: float, x_max: float, y_max: float, area: float):
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max
        self.area = area

    def __repr__(self) -> str:
        return (
            f"AxisAlignedResult(area={self.area:.4f}, "
            f"rect=[{self.x_min:.4f}, {self.y_min:.4f}, {self.x_max:.4f}, {self.y_max:.4f}])"
        )


class OrientedLirResult:
    """Result of an oriented rectangle solve.

    Attributes
    ----------
    x_min, y_min, x_max, y_max : float
        Axis-aligned bounding box of the inscribed rectangle.
    area : float
        Area of the rectangle.
    """

    def __init__(self, x_min: float, y_min: float, x_max: float, y_max: float, area: float):
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max
        self.area = area

    def __repr__(self) -> str:
        return (
            f"OrientedLirResult(area={self.area:.4f}, "
            f"bounds=[{self.x_min:.4f}, {self.y_min:.4f}, {self.x_max:.4f}, {self.y_max:.4f}])"
        )


class BcrsResult:
    """Result of a BCRS oriented rectangle solve.

    Attributes
    ----------
    x_min, y_min, x_max, y_max : float
        Axis-aligned bounding box of the inscribed rectangle.
    area : float
        Area of the rectangle.
    angle_deg : float
        Rotation angle that produced this rectangle.
    """

    def __init__(self, x_min: float, y_min: float, x_max: float, y_max: float, area: float, angle_deg: float):
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max
        self.area = area
        self.angle_deg = angle_deg

    def __repr__(self) -> str:
        return (
            f"BcrsResult(area={self.area:.4f}, angle={self.angle_deg:.1f}°, "
            f"rect=[{self.x_min:.4f}, {self.y_min:.4f}, {self.x_max:.4f}, {self.y_max:.4f}])"
        )


class MicResult:
    """Result of a maximum-inscribed-circle solve."""

    def __init__(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        radius_sq: float,
        used_engine: str,
        candidate_count: int,
    ):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        self.radius_sq = radius_sq
        self.used_engine = used_engine
        self.candidate_count = candidate_count

    def __repr__(self) -> str:
        return (
            f"MicResult(center=({self.center_x:.4f}, {self.center_y:.4f}), "
            f"radius={self.radius:.4f}, engine={self.used_engine}, "
            f"candidates={self.candidate_count})"
        )


def solve_axis_aligned(
    exterior: List[Tuple[float, float]],
    max_aspect_ratio: Optional[float] = None,
) -> AxisAlignedResult:
    """Find the largest axis-aligned rectangle inscribed in a polygon.

    Uses the Daniels et al. (1997) vertex-grid algorithm with adaptive
    subdivision and per-side contraction verification.

    Parameters
    ----------
    exterior : list of (float, float)
        Polygon exterior ring coordinates, listed clockwise or counter-clockwise.
        The ring is closed automatically (last point does not need to repeat first).
        At least 3 points are required.
    max_aspect_ratio : float, optional
        Maximum allowed aspect ratio (longer ÷ shorter).  ``0`` or ``None`` means
        unlimited.  When constrained, the rectangle is shrunk symmetrically to
        satisfy the limit.

    Returns
    -------
    AxisAlignedResult
        The largest axis-aligned rectangle that fits entirely inside the polygon.

    Raises
    ------
    ValueError
        If the polygon has fewer than 3 vertices or no valid rectangle is found.
    """
    py_result = solve_axis_aligned_py(exterior, max_aspect_ratio=max_aspect_ratio)
    return AxisAlignedResult(
        x_min=py_result.x_min,
        y_min=py_result.y_min,
        x_max=py_result.x_max,
        y_max=py_result.y_max,
        area=py_result.area,
    )


def solve_oriented_lir(
    exterior: List[Tuple[float, float]],
    rotation_degrees: Optional[float] = None,
) -> OrientedLirResult:
    """Find the largest oriented (rotated) rectangle inscribed in a polygon.

    Parameters
    ----------
    exterior : list of (float, float)
        Polygon exterior ring coordinates.
    rotation_degrees : float, optional
        Rotation angle for oriented search (degrees). ``None`` sweeps 0–90°.

    Returns
    -------
    OrientedLirResult
        The largest inscribed rectangle.

    Raises
    ------
    ValueError
        If the polygon has fewer than 3 vertices or no rectangle is found.
    """
    py_result = solve_oriented_lir_py(exterior, rotation_degrees=rotation_degrees)
    return OrientedLirResult(
        x_min=py_result.x_min,
        y_min=py_result.y_min,
        x_max=py_result.x_max,
        y_max=py_result.y_max,
        area=py_result.area,
    )


def solve_bcrs(
    exterior: List[Tuple[float, float]],
    max_aspect_ratio: Optional[float] = None,
    use_parallel_field: bool = False,
    use_simulated_annealing: bool = False,
    use_bootstrap_seeds: bool = False,
    use_pca_axes: bool = False,
) -> BcrsResult:
    """Find the largest oriented rectangle using BCRS (Boundary-Coordinate
    Rectangle Solve).

    Parameters
    ----------
    exterior : list of (float, float)
        Polygon exterior ring coordinates.
    max_aspect_ratio : float, optional
        Maximum aspect ratio (0 or None = unlimited).
    use_parallel_field : bool, optional
        Enable the parallel ray-shooting candidate-field refinement.
    use_simulated_annealing : bool, optional
        Enable stochastic center/angle basin escape over top candidates.
    use_bootstrap_seeds : bool, optional
        Enable deterministic multi-seed bootstrap per angle
        (vertex-snapped valid seed + center-only promotion seeds).
    use_pca_axes : bool, optional
        Enable Principal Component Analysis for angle candidate guidance.

    Returns
    -------
    BcrsResult
        The largest inscribed oriented rectangle.

    Raises
    ------
    ValueError
        If the polygon has fewer than 3 vertices or no rectangle is found.
    """
    py_result = solve_bcrs_py(
        exterior,
        max_aspect_ratio=max_aspect_ratio,
        use_parallel_field=use_parallel_field,
        use_simulated_annealing=use_simulated_annealing,
        use_bootstrap_seeds=use_bootstrap_seeds,
        use_pca_axes=use_pca_axes,
    )
    return BcrsResult(
        x_min=py_result.x_min,
        y_min=py_result.y_min,
        x_max=py_result.x_max,
        y_max=py_result.y_max,
        area=py_result.area,
        angle_deg=py_result.angle_deg,
    )


def solve_maximum_inscribed_circle(
    exterior: List[Tuple[float, float]],
    engine: Optional[str] = None,
    robust_mode: Optional[str] = None,
) -> MicResult:
    """Find the maximum inscribed circle in a polygon.

    Parameters
    ----------
    exterior : list of (float, float)
        Polygon exterior ring coordinates.
    engine : {"exact_only", "fallback_only", "exact_then_geos"}, optional
        Solver engine selection.
    robust_mode : {"fast_f64", "filtered"}, optional
        Robustness mode for candidate filtering/certification.
    """
    py_result = solve_mic_py(exterior, engine=engine, robust_mode=robust_mode)
    return MicResult(
        center_x=py_result.center_x,
        center_y=py_result.center_y,
        radius=py_result.radius,
        radius_sq=py_result.radius_sq,
        used_engine=py_result.used_engine,
        candidate_count=py_result.candidate_count,
    )


def demo() -> None:
    """Run a quick demo on built-in test cases."""
    print(axis_aligned_demo())
    print(oriented_lir_demo())
