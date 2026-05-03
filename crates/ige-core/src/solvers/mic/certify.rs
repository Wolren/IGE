use super::index::{NearestBoundaryIndex, PipIndex};

/// Certify a candidate center/radius against shared predicates.
pub(crate) fn certify_candidate(
    pip_index: &PipIndex,
    nb_index: &NearestBoundaryIndex,
    x: f64,
    y: f64,
    radius_sq: f64,
) -> bool {
    if !pip_index.contains_strict_xy(x, y) {
        return false;
    }

    match nb_index.nearest_distance_sq(x, y) {
        Some((nearest_sq, _)) => {
            let tol = nearest_sq.abs() * 1e-9 + 1e-20;
            nearest_sq + tol >= radius_sq
        }
        None => false,
    }
}
