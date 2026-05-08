"""
Standalone QGIS Processing Algorithm — LIRiAP

Drop THIS FILE into your QGIS processing scripts folder:
  Windows: %APPDATA%/QGIS/QGIS3/profiles/default/processing/scripts/
  Linux:   ~/.local/share/QGIS/QGIS3/profiles/default/processing/scripts/

It will appear in the Processing Toolbox under "LIRiAP".

Requires the ``liriap`` pip package:
  pip install liriap

After installing, restart QGIS or refresh the Processing Toolbox.
"""

from qgis_plugin.LIRiAP.algorithms import (
    LargestAxisAlignedRectAlgorithm,
    OrientedLirAlgorithm,
)
