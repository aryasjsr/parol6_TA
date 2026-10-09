import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from backend.ui_theme import (
    DEFAULT_UI_THEME,
    UI_THEME_NAMES,
    customtkinter_appearance_mode,
    get_ui_palette,
    normalize_ui_theme,
)


def test_theme_names_are_normalized_case_insensitively():
    assert normalize_ui_theme("light") == "Light"
    assert normalize_ui_theme(" GRAY ") == "Gray"
    assert normalize_ui_theme("unsupported") == DEFAULT_UI_THEME
    assert normalize_ui_theme(None) == DEFAULT_UI_THEME


def test_all_themes_define_the_same_distinct_palette_roles():
    palettes = [get_ui_palette(name) for name in UI_THEME_NAMES]
    assert all(palette.keys() == palettes[0].keys() for palette in palettes)
    assert len({palette["APP_BG"] for palette in palettes}) == len(UI_THEME_NAMES)
    assert len({palette["SURFACE"] for palette in palettes}) == len(UI_THEME_NAMES)


def test_gray_uses_dark_customtkinter_base_mode():
    assert customtkinter_appearance_mode("Light") == "Light"
    assert customtkinter_appearance_mode("Dark") == "Dark"
    assert customtkinter_appearance_mode("Gray") == "Dark"


def test_palette_callers_receive_a_copy():
    palette = get_ui_palette("Dark")
    palette["APP_BG"] = "#ffffff"
    assert get_ui_palette("Dark")["APP_BG"] != "#ffffff"
