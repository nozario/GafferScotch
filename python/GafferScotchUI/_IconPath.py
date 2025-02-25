import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_RESOURCES_DIR = os.path.abspath(
    os.path.join(_THIS_DIR, "..", "..", "resources", "icons")
)
ICON_PATH = os.path.join(_RESOURCES_DIR, "scotch.png")
ICON_NAME = "scotch.png"
