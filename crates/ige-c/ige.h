/**
 * @file ige.h
 * @brief C API for Inscribed Geometry Engine (IGE)
 *
 * This header provides a C-compatible interface for computing the largest
 * axis-aligned rectangle inscribed in a polygon.
 */

#ifndef IGE_H
#define IGE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Rectangle result structure
 */
typedef struct {
    double x_min;  ///< Minimum X coordinate
    double y_min;  ///< Minimum Y coordinate
    double x_max;  ///< Maximum X coordinate
    double y_max;  ///< Maximum Y coordinate
} IgeRectangle;

/**
 * @brief Oriented solver configuration options
 */
typedef struct {
    double rotation_degrees;        ///< Rotation angle (0 = axis-aligned)
    int prefer_gpu;                ///< Prefer GPU when available (1=yes, 0=no)
    int force_cpu;                 ///< Force CPU solver (1=yes, 0=no)
    double max_aspect_ratio;       ///< Maximum aspect ratio (0 = unlimited)
    double min_aspect_ratio;       ///< Minimum aspect ratio (0 = unlimited)
    int use_parallel_field;        ///< Enable parallel candidate-field refinement (1=yes, 0=no)
    int use_simulated_annealing;   ///< Enable SA basin-escape search (1=yes, 0=no)
    int use_bootstrap_seeds;       ///< Enable deterministic bootstrap multi-seed stage (1=yes, 0=no)
    int use_pca_axes;              ///< Enable PCA-guided angle candidates (1=yes, 0=no)
} IgeOptions;

/**
 * @brief Axis-aligned solver configuration options
 */
typedef struct {
    double max_aspect_ratio;  ///< Maximum aspect ratio (0 = unlimited)
} IgeAxisAlignedOptions;

/**
 * @brief Get default oriented solver options
 *
 * @return Default IgeOptions structure
 */
IgeOptions ige_options_default(void);

/**
 * @brief Solve for the largest oriented inscribed rectangle
 *
 * @param coords Array of polygon coordinates [x0, y0, x1, y1, ...]
 * @param coords_len Number of elements in coords array (must be even, >= 6)
 * @param options Solver options (NULL for defaults)
 * @param result Output rectangle result
 * @return 0 on success, -1 on error
 */
int ige_solve(
    const double *coords,
    size_t coords_len,
    const IgeOptions *options,
    IgeRectangle *result
);

/**
 * @brief Get default axis-aligned solver options
 *
 * @return Default IgeAxisAlignedOptions structure
 */
IgeAxisAlignedOptions ige_axis_aligned_options_default(void);

/**
 * @brief Solve for the largest axis-aligned rectangle
 *
 * @param coords Array of polygon coordinates [x0, y0, x1, y1, ...]
 * @param coords_len Number of elements in coords array (must be even, >= 6)
 * @param options Axis-aligned solver options (NULL for defaults)
 * @param result Output rectangle result
 * @return 0 on success, -1 on error
 */
int ige_solve_axis_aligned(
    const double *coords,
    size_t coords_len,
    const IgeAxisAlignedOptions *options,
    IgeRectangle *result
);

/**
 * @brief Calculate the area of a rectangle
 *
 * @param rect Rectangle to measure
 * @return Area (width * height)
 */
double ige_rectangle_area(const IgeRectangle *rect);

#ifdef __cplusplus
}
#endif

#endif /* IGE_H */
