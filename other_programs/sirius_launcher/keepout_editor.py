"""Compatibility entry point for the shared standalone editor."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'keepout-editor' / 'src'))
from keepout_editor_app import editor
sys.modules[__name__] = editor
