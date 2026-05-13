"""
LIRiAP Oriented LIR algorithm wrapper.

Computes the largest oriented rectangle inscribed in each polygon.
"""

import importlib.util
import os
import subprocess
import sys

from qgis.PyQt.QtCore import QCoreApplication, QVariant
from qgis.core import (
    QgsFeature,
    QgsFeatureSink,
    QgsField,
    QgsFields,
    QgsGeometry,
    QgsProcessing,
    QgsProcessingAlgorithm,
    QgsProcessingException,
    QgsProcessingParameterBoolean,
    QgsProcessingParameterFeatureSink,
    QgsProcessingParameterNumber,
    QgsProcessingParameterVectorLayer,
    QgsWkbTypes,
)


def _get_ige():
    try:
        return importlib.import_module("ige")
    except ImportError:
        return None


def _find_python():
    if sys.platform != "win32":
        return sys.executable
    base = os.path.dirname(sys.executable)
    candidates = [
        os.path.join(base, "python.exe"),
        os.path.join(os.path.dirname(base), "python.exe"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def _try_install_ige():
    python = _find_python()
    if not python:
        return None
    cmd = [python, "-m", "pip", "install", "--user", "--index-url", "https://test.pypi.org/simple/", "ige"]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False, timeout=180)
        if proc.returncode == 0:
            importlib.invalidate_caches()
            return _get_ige()
    except:
        pass
    return None


IGE = None


RESULT_FIELDS = QgsFields()
RESULT_FIELDS.append(QgsField("area", QVariant.Double))
RESULT_FIELDS.append(QgsField("src_fid", QVariant.Int))
RESULT_FIELDS.append(QgsField("center_x", QVariant.Double))
RESULT_FIELDS.append(QgsField("center_y", QVariant.Double))
RESULT_FIELDS.append(QgsField("width", QVariant.Double))
RESULT_FIELDS.append(QgsField("height", QVariant.Double))
RESULT_FIELDS.append(QgsField("angle_deg", QVariant.Double))
RESULT_FIELDS.append(QgsField("aspect_ratio", QVariant.Double))
RESULT_FIELDS.append(QgsField("best_effort", QVariant.Int))


class OrientedLirAlgorithm(QgsProcessingAlgorithm):

    INPUT = "INPUT"
    OUTPUT = "OUTPUT"
    ROTATION = "ROTATION"
    MAX_RATIO = "MAX_RATIO"
    MIN_RATIO = "MIN_RATIO"
    GRID_COARSE = "GRID_COARSE"
    GRID_FINE = "GRID_FINE"
    TOP_K = "TOP_K"
    USE_PARALLEL = "USE_PARALLEL"
    USE_SA = "USE_SA"
    USE_BOOTSTRAP = "USE_BOOTSTRAP"
    USE_PCA = "USE_PCA"
    USE_EDGE_ANCHORED = "USE_EDGE_ANCHORED"

    def tr(self, text):
        return QCoreApplication.translate("Processing", text)

    def createInstance(self):
        return OrientedLirAlgorithm()

    def name(self):
        return "oriented_lir"

    def displayName(self):
        return "Oriented LIR (BCRS)"

    def group(self):
        return "LIRiAP"

    def groupId(self):
        return "liriap"

    def shortHelpString(self):
        return (
            "Computes the largest oriented (rotated) rectangle inscribed in each "
            "polygon, using the BCRS algorithm with SDF-guided expansion.\n\n"
            "Advanced options:\n"
            "  - Parallel field: local angle polish\n"
            "  - Simulated annealing: rescue from local minima\n"
            "  - Bootstrap seeds: vertex-snapped + center seeds\n"
            "  - PCA axes: use principal component analysis\n"
            "  - Edge-anchored: generate candidates from boundary"
        )

    def initAlgorithm(self, config=None):
        self.addParameter(
            QgsProcessingParameterVectorLayer(
                self.INPUT, "Input polygons", [QgsProcessing.TypeVectorPolygon]
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.ROTATION, "Rotation angle (degrees, blank = auto)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=None, optional=True
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.MAX_RATIO, "Max aspect ratio (0 = unlimited)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=0.0, minValue=0.0
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.MIN_RATIO, "Min aspect ratio (0 = unlimited)",
                type=QgsProcessingParameterNumber.Double,
                defaultValue=0.0, minValue=0.0
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.GRID_COARSE, "Coarse grid resolution",
                type=QgsProcessingParameterNumber.Integer,
                defaultValue=32, minValue=4, maxValue=1000
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.GRID_FINE, "Fine grid resolution",
                type=QgsProcessingParameterNumber.Integer,
                defaultValue=64, minValue=10, maxValue=1000
            )
        )
        self.addParameter(
            QgsProcessingParameterNumber(
                self.TOP_K, "Top K candidates",
                type=QgsProcessingParameterNumber.Integer,
                defaultValue=20, minValue=1, maxValue=100
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.USE_PARALLEL, "Use parallel field (local angle polish)",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.USE_SA, "Use simulated annealing rescue",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.USE_BOOTSTRAP, "Use bootstrap seeds",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.USE_PCA, "Use PCA axes",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.USE_EDGE_ANCHORED, "Use edge-anchored candidates",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterBoolean(
                self.USE_EARLY_STOP, "Use early stopping",
                defaultValue=False
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT, "Inscribed rectangles", QgsProcessing.TypeVectorPolygon
            )
        )

    def processAlgorithm(self, parameters, context, feedback):
        global IGE
        if IGE is None:
            IGE = _get_ige()
        if IGE is None:
            feedback.pushInfo("Installing ige from TestPyPI...")
            IGE = _try_install_ige()
        if IGE is None:
            raise QgsProcessingException("Failed to install ige. Install manually in QGIS Python.")

        source = self.parameterAsSource(parameters, self.INPUT, context)
        rot = self.parameterAsDouble(parameters, self.ROTATION, context)
        max_ratio = self.parameterAsDouble(parameters, self.MAX_RATIO, context)
        min_ratio = self.parameterAsDouble(parameters, self.MIN_RATIO, context)
        grid_coarse = self.parameterAsInt(parameters, self.GRID_COARSE, context)
        grid_fine = self.parameterAsInt(parameters, self.GRID_FINE, context)
        top_k = self.parameterAsInt(parameters, self.TOP_K, context)
        use_parallel = self.parameterAsBool(parameters, self.USE_PARALLEL, context)
        use_sa = self.parameterAsBool(parameters, self.USE_SA, context)
        use_bootstrap = self.parameterAsBool(parameters, self.USE_BOOTSTRAP, context)
        use_pca = self.parameterAsBool(parameters, self.USE_PCA, context)
        use_edge_anchored = self.parameterAsBool(parameters, self.USE_EDGE_ANCHORED, context)

        crs = source.sourceCrs()

        sink, dest_id = self.parameterAsSink(
            parameters, self.OUTPUT, context, RESULT_FIELDS, QgsWkbTypes.Polygon, crs
        )

        total = source.featureCount() if source else 0

        for i, feature in enumerate(source.getFeatures()):
            if feedback.isCanceled():
                break
            feedback.setProgress(i * 100 // max(total, 1))

            geom = feature.geometry()
            if not geom or geom.isEmpty():
                continue
            poly = geom.asPolygon()
            if not poly:
                continue
            exterior = [(pt.x(), pt.y()) for pt in poly[0]]
            holes = [[(pt.x(), pt.y()) for pt in ring] for ring in poly[1:] if len(ring) >= 3]

            try:
                rect = IGE.solve_oriented_lir_py(
                    exterior,
                    holes=holes,
                    rotation_degrees=rot if rot != 0.0 else None,
                    max_aspect_ratio=max_ratio if max_ratio > 0 else None,
                    min_aspect_ratio=min_ratio if min_ratio > 0 else None,
                    grid_coarse=grid_coarse,
                    grid_fine=grid_fine,
                    top_k=top_k,
                    use_parallel_field=use_parallel,
                    use_simulated_annealing=use_sa,
                    use_bootstrap_seeds=use_bootstrap,
                    use_pca_axes=use_pca,
                    use_edge_anchored=use_edge_anchored,
                )
                out_feat = QgsFeature(RESULT_FIELDS)
                out_feat.setGeometry(QgsGeometry.fromWkt(rect.polygon_wkt))
                out_feat.setAttribute("area", rect.area)
                out_feat.setAttribute("src_fid", int(feature.id()))
                out_feat.setAttribute("center_x", rect.center_x)
                out_feat.setAttribute("center_y", rect.center_y)
                out_feat.setAttribute("width", rect.width)
                out_feat.setAttribute("height", rect.height)
                out_feat.setAttribute("angle_deg", rect.angle_deg)
                out_feat.setAttribute("aspect_ratio", rect.aspect_ratio)
                out_feat.setAttribute("best_effort", 1 if rect.best_effort else 0)
                sink.addFeature(out_feat, QgsFeatureSink.FastInsert)
            except Exception as e:
                feedback.reportError(f"Feature {feature.id()}: {e}")

        return {self.OUTPUT: dest_id}