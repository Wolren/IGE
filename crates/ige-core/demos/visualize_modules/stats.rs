//! Statistics aggregation helpers for LIR and MIC modes.

/// MIC-specific result summary after parallel computation.
#[derive(Default)]
pub struct MicStats {
    pub exact_ok: usize,
    pub geos_ok: usize,
    pub both_ok: usize,
    pub rel_errs: Vec<f64>,
    pub exact_ms_acc: f64,
    pub geos_ms_acc: f64,
    pub per_polygon_errors: Vec<(String, f64, f64, f64, &'static str)>,
    pub exact_larger_count: usize,
    pub exact_smaller_count: usize,
}

impl MicStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&mut self, idx: usize, _card: &str, exact_r: Option<f64>, geos_r: Option<f64>, exact_ms: f64, geos_ms: f64, polygon_ids: &[String]) {
        self.exact_ms_acc += exact_ms;
        self.geos_ms_acc += geos_ms;
        if exact_r.is_some() { self.exact_ok += 1; }
        if geos_r.is_some() { self.geos_ok += 1; }
        if let (Some(e), Some(g)) = (exact_r, geos_r) {
            if g > 0.0 {
                self.both_ok += 1;
                let abs_pct = (e - g).abs() / g * 100.0;
                self.rel_errs.push(abs_pct);
                let dir = if e > g { 
                    self.exact_larger_count += 1;
                    "exact_larger" 
                } else { 
                    self.exact_smaller_count += 1;
                    "exact_smaller" 
                };
                self.per_polygon_errors.push((polygon_ids[idx].clone(), e, g, abs_pct, dir));
            }
        }
    }

    pub fn finalize(self, results_len: usize) -> MicSummary {
        let mut sorted_rel_errs = self.rel_errs;
        sorted_rel_errs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let rel_mean = if sorted_rel_errs.is_empty() {
            0.0
        } else {
            sorted_rel_errs.iter().sum::<f64>() / sorted_rel_errs.len() as f64
        };
        let rel_median = if sorted_rel_errs.is_empty() {
            0.0
        } else {
            sorted_rel_errs[sorted_rel_errs.len() / 2]
        };

        let avg_exact_ms = if results_len > 0 { self.exact_ms_acc / results_len as f64 } else { 0.0 };
        let avg_geos_ms = if results_len > 0 { self.geos_ms_acc / results_len as f64 } else { 0.0 };

        // Top 10 errors by absolute percentage error.
        let mut top_errors = self.per_polygon_errors;
        top_errors.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap_or(std::cmp::Ordering::Equal));
        let top_errors: Vec<serde_json::Value> = top_errors.iter()
            .take(10)
            .map(|(id, e, g, pct, dir)| {
                serde_json::json!({
                    "id": id, "exact_radius": e, "geos_radius": g,
                    "err_pct": pct, "direction": dir,
                })
            })
            .collect();

        MicSummary {
            exact_ok: self.exact_ok,
            geos_ok: self.geos_ok,
            both_ok: self.both_ok,
            rel_median,
            rel_mean,
            top_errors,
            avg_exact_ms,
            avg_geos_ms,
            exact_larger_count: self.exact_larger_count,
            exact_smaller_count: self.exact_smaller_count,
        }
    }
}

/// Summary metrics available after processing LIR results.
#[derive(Debug, Clone)]
pub struct LirStats {
    pub success: usize,
    pub total_poly_area: f64,
    pub total_rect_area: f64,
    pub per_shape_pcts: Vec<f64>,
}

impl FromIterator<(f64, f64)> for LirStats {
    fn from_iter<T: IntoIterator<Item = (f64, f64)>>(iter: T) -> Self {
        let mut success = 0usize;
        let mut total_poly_area = 0.0;
        let mut total_rect_area = 0.0;
        let mut per_shape_pcts = Vec::new();

        for (ra, pa) in iter {
            if ra > 0.0 { success += 1; }
            total_rect_area += ra;
            total_poly_area += pa;
            let pct = if pa > 0.0 { ra / pa * 100.0 } else { 0.0 };
            per_shape_pcts.push(pct);
        }

        Self { success, total_poly_area, total_rect_area, per_shape_pcts }
    }
}

impl LirStats {
    /// Compute fill percentage across all polygons.
    pub fn overall_fill_pct(&self) -> f64 {
        if self.total_poly_area > 0.0 {
            self.total_rect_area / self.total_poly_area * 100.0
        } else {
            0.0
        }
    }

    /// Median per-shape fill percentage.
    pub fn median_pct(&self) -> f64 {
        let mut sorted = self.per_shape_pcts.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = sorted.len();
        if n > 0 { sorted[n / 2] } else { 0.0 }
    }

    /// Mean per-shape fill percentage.
    pub fn mean_pct(&self) -> f64 {
        let n = self.per_shape_pcts.len();
        if n > 0 {
            self.per_shape_pcts.iter().sum::<f64>() / n as f64
        } else {
            0.0
        }
    }
}

/// Aggregated MIC results for JSON/HTML summary.
pub struct MicSummary {
    pub exact_ok: usize,
    pub geos_ok: usize,
    pub both_ok: usize,
    pub rel_median: f64,
    pub rel_mean: f64,
    pub top_errors: Vec<serde_json::Value>,
    pub avg_exact_ms: f64,
    pub avg_geos_ms: f64,
    pub exact_larger_count: usize,
    pub exact_smaller_count: usize,
}

impl MicSummary {
    pub fn speed_ratio(&self) -> f64 {
        if self.avg_geos_ms > 0.0 {
            self.avg_exact_ms / self.avg_geos_ms
        } else {
            0.0
        }
    }
}
