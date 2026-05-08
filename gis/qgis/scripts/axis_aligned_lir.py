"""
LIRiAP Axis-Aligned LIR algorithm wrapper.

Computes the largest axis-aligned rectangle inscribed in each polygon.
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
RESULT_FIELDS.append(QgsField("x_min", QVariant.Double))
RESULT_FIELDS.append(QgsField("y_min", QVariant.Double))
RESULT_FIELDS.append(QgsField("x_max", QVariant.Double))
RESULT_FIELDS.append(QgsField("y_max", QVariant.Double))
RESULT_FIELDS.append(QgsField("width", QVariant.Double))
RESULT_FIELDS.append(QgsField("height", QVariant.Double))


class LargestAxisAlignedRectAlgorithm(QgsProcessingAlgorithm):

    INPUT = "INPUT"
    OUTPUT = "OUTPUT"
    MAX_RATIO = "MAX_RATIO"
    MIN_RATIO = "MIN_RATIO"
    MAX_GRID = "MAX_GRID"

    def tr(self, text):
        return QCoreApplication.translate("Processing", text)

    def createInstance(self):
        return LargestAxisAlignedRectAlgorithm()

    def name(self):
        return "largest_axis_aligned_rect"

    def displayName(self):
        return "Largest Axis-Aligned Rectangle"

    def group(self):
        return "LIRiAP"

    def groupId(self):
        return "liriap"

    def shortHelpString(self):
        return (
            "Computes the largest axis-aligned rectangle inscribed in each "
            "polygon, using an exact vertex-grid algorithm "
            "(Daniels et al. 1997).\n\n"
            "Parameters:\n"
            "  Max aspect ratio — clamp the longer/shorter ratio (0=unlimited)\n"
            "  Min aspect ratio — require minimum elongation\n"
            "  Max grid resolution — vertex grid density (default=32)"
        )

    def initAlgorithm(self, config=None):
        self.addParameter(
            QgsProcessingParameterVectorLayer(
                self.INPUT, "Input polygons", [QgsProcessing.TypeVectorPolygon]
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
                self.MAX_GRID, "Max grid resolution",
                type=QgsProcessingParameterNumber.Integer,
                defaultValue=32, minValue=10, maxValue=1000
            )
        )
        self.addParameter(
            QgsProcessingParameterFeatureSink(
                self.OUTPUT, "Output rectangles",
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
            raise QgsProcessingException("Failed to install ige. Install manually in Qgs Python.")

        source = self.parameterAsSource(parameters, self.INPUT, context)
        max_ratio = self.parameterAsDouble(parameters, self.MAX_RATIO, context)
        min_ratio = self.parameterAsDouble(parameters, self.MIN_RATIO, context)
        max_grid = self.parameterAsInt(parameters, self.MAX_GRID, context)

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
                rect = IGE.solve_axis_aligned_py(
                    exterior,
                    holes=holes,
                    max_aspect_ratio=max_ratio if max_ratio > 0 else None,
                    min_aspect_ratio=min_ratio if min_ratio > 0 else None,
                    max_grid=max_grid,
                )
                out_feat = QgsFeature(RESULT_FIELDS)
                out_feat.setGeometry(QgsGeometry.fromRect(
                    QgsGeometry.fromRect(rect.x_min, rect.y_min, rect.x_max, rect.y_max)
                ))
                out_feat.setAttribute("area", rect.area)
                out_feat.setAttribute("src_fid", int(feature.id()))
                out_feat.setAttribute("x_min", rect.x_min)
                out_feat.setAttribute("y_min", rect.y_min)
                out_feat.setAttribute("x_max", rect.x_max)
                out_feat.setAttribute("y_max", rect.y_max)
                out_feat.setAttribute("width", rect.x_max - rect.x_min)
                out_feat.setAttribute("height", rect.y_max - rect.y_min)
                sink.addFeature(out_feat, QgsFeatureSink.FastInsert)
            except Exception as e:
                feedback.reportError(f"Feature {feature.id()}: {e}")

        return {self.OUTPUT: dest_id}