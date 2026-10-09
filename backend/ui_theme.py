"""Shared colour palettes and helpers for the PAROL6 desktop interface."""

from __future__ import annotations

from typing import Final


UI_THEME_NAMES: Final[tuple[str, ...]] = ("Dark", "Light", "Gray")
DEFAULT_UI_THEME: Final[str] = "Dark"

UI_THEME_PALETTES: Final[dict[str, dict[str, str]]] = {
    "Dark": {
        "APP_BG": "#000000",
        "SURFACE": "#0a0a0a",
        "SURFACE_ALT": "#111111",
        "SURFACE_LOW": "#080808",
        "SURFACE_HIGH": "#1a1a1a",
        "BORDER": "#1f1f1f",
        "OUTLINE": "#2a2a2a",
        "HANDLE": "#252525",
        "ACCENT": "#3366ff",
        "ACCENT_DEEP": "#5588ff",
        "ACCENT_SOFT": "#0d1a33",
        "GOLD": "#d4a843",
        "ON_SURFACE": "#f0f0f0",
        "ON_SURFACE_MUTE": "#9a9a9a",
        "SUCCESS": "#2ecc71",
        "DANGER": "#e74c3c",
        "WARN": "#f39c12",
    },
    "Light": {
        "APP_BG": "#edf1f5",
        "SURFACE": "#ffffff",
        "SURFACE_ALT": "#f3f6f9",
        "SURFACE_LOW": "#f7f9fb",
        "SURFACE_HIGH": "#e5ebf1",
        "BORDER": "#cbd5df",
        "OUTLINE": "#b6c3d0",
        "HANDLE": "#98a8b8",
        "ACCENT": "#2457d6",
        "ACCENT_DEEP": "#1747bd",
        "ACCENT_SOFT": "#dce7ff",
        "GOLD": "#8a6412",
        "ON_SURFACE": "#17212b",
        "ON_SURFACE_MUTE": "#5f6d7a",
        "SUCCESS": "#18864b",
        "DANGER": "#c9362b",
        "WARN": "#d98900",
    },
    "Gray": {
        "APP_BG": "#292c30",
        "SURFACE": "#393d42",
        "SURFACE_ALT": "#44494f",
        "SURFACE_LOW": "#303338",
        "SURFACE_HIGH": "#50565d",
        "BORDER": "#626971",
        "OUTLINE": "#747c85",
        "HANDLE": "#69717a",
        "ACCENT": "#727b85",
        "ACCENT_DEEP": "#929aa3",
        "ACCENT_SOFT": "#4b5056",
        "GOLD": "#d0b36f",
        "ON_SURFACE": "#f2f3f4",
        "ON_SURFACE_MUTE": "#c0c5ca",
        "SUCCESS": "#3bad6f",
        "DANGER": "#e05a50",
        "WARN": "#e3a534",
    },
}


def normalize_ui_theme(value: object) -> str:
    """Return one of the supported display names, defaulting safely to Dark."""
    candidate = str(value or "").strip().casefold()
    for name in UI_THEME_NAMES:
        if candidate == name.casefold():
            return name
    return DEFAULT_UI_THEME


def get_ui_palette(theme_name: object) -> dict[str, str]:
    """Return a detached palette so callers cannot mutate the shared constants."""
    return dict(UI_THEME_PALETTES[normalize_ui_theme(theme_name)])


def customtkinter_appearance_mode(theme_name: object) -> str:
    """Map the three application themes to CustomTkinter's two base modes."""
    return "Light" if normalize_ui_theme(theme_name) == "Light" else "Dark"
