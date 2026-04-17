#!/usr/bin/env python3
"""
Apply Apple Light tokens to interactive/*.html and app.html.
- Injects <link rel="stylesheet" href="../shared/theme.css"> (or "shared/..." for app.html).
- Replaces dark UI color hex values with Apple Light equivalents.
- Leaves data-series colors alone (they carry information).
Idempotent: running twice has no further effect.
"""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INTERACTIVE = ROOT / "interactive"
APP_HTML = ROOT / "app.html"

# (pattern, replacement). Order matters — longer/more-specific first.
# Hex is matched case-insensitively via re.IGNORECASE.
COLOR_MAP = [
    # ── Dark chrome backgrounds → white / f5f5f7 ──
    (r"#1a1a2e\b", "#ffffff"),       # main page bg
    (r"#0f0f23\b", "#f5f5f7"),       # deep bg / inset
    (r"#12122b\b", "#f5f5f7"),       # sidebar variant
    (r"#16213e\b", "#f5f5f7"),       # panel mid
    (r"#1e1e3a\b", "#f5f5f7"),       # tooltip / blockquote
    (r"#1e1e38\b", "#f5f5f7"),       # table even
    (r"#16162e\b", "#f5f5f7"),       # table odd
    (r"#252545\b", "#f5f5f7"),       # table header
    (r"#151528\b", "#f5f5f7"),       # sidebar bg (guide)
    (r"#0d1117\b", "#f5f5f7"),       # code bg
    (r"#0d1b3e\b", "#f5f5f7"),       # info box
    (r"#0d1b38\b", "#f5f5f7"),       # state-display (se2_odometry)
    (r"#0d0d20\b", "#fafafc"),       # dark image inset (feature_matching)
    (r"#2a2a4e\b", "#f5f5f7"),       # preset button bg
    (r"#1a5276\b", "#e8e8ed"),       # button hover
    (r"#1a4a8a\b", "rgba(0,113,227,0.12)"),   # secondary button hover
    (r"#1a2744\b", "rgba(0,0,0,0.05)"),       # dark navy grid (se2_odometry)
    (r"#334466\b", "rgba(0,0,0,0.12)"),       # mid-dark axis
    (r"#000000\b", "#ffffff"),       # pure black canvas → light (chrome only)

    # ── Borders (dark → subtle light) ──
    (r"#2a2a4a\b", "rgba(0,0,0,0.08)"),
    (r"#0f3460\b", "rgba(0,0,0,0.12)"),
    (r"#5a4aaa\b", "rgba(0,0,0,0.12)"),       # purple border (feature_matching)

    # ── Accents: unify to Apple Blue ──
    (r"#00d4ff\b", "#0071e3"),       # cyan
    (r"#e94560\b", "#0071e3"),       # hot pink
    (r"#6fa8dc\b", "#0066cc"),       # link blue
    (r"#4a6fa5\b", "#0071e3"),       # muted blue
    (r"#8fcbfa\b", "#2997ff"),       # gradient end
    (r"#8ab4f8\b", "#0066cc"),       # quote accent
    (r"#53d8fb\b", "#0071e3"),       # bright cyan (se2_odometry)
    (r"#7b68ee\b", "#0071e3"),       # purple accent
    (r"#3a2a7a\b", "#0071e3"),       # purple button bg
    (r"#4a3a9a\b", "#0077ed"),       # purple hover
    (r"#c73e55\b", "#0077ed"),       # stray red-pink hover
    (r"#7bf\b",    "#0071e3"),       # shorthand bright blue
    (r"#a0d0a0\b", "#34c759"),       # pastel green → Apple green

    # ── Generic light text on dark → dark on light ──
    (r"#e0e0e0\b", "#1d1d1f"),
    (r"#f0f0f0\b", "#1d1d1f"),

    # ── Muted greys (short + long hex) ──
    (r"#b0b0c8\b", "rgba(0,0,0,0.80)"),
    (r"#a0a0c0\b", "rgba(0,0,0,0.56)"),
    (r"#9a9ab0\b", "rgba(0,0,0,0.56)"),
    (r"#bbb\b",    "rgba(0,0,0,0.56)"),
    (r"#ccc\b",    "rgba(0,0,0,0.56)"),
    (r"#aaa\b",    "rgba(0,0,0,0.48)"),
    (r"#999\b",    "rgba(0,0,0,0.48)"),
    (r"#888\b",    "rgba(0,0,0,0.56)"),
    (r"#777\b",    "rgba(0,0,0,0.56)"),
    (r"#666\b",    "rgba(0,0,0,0.64)"),
    (r"#555\b",    "rgba(0,0,0,0.16)"),       # usually border on dark bg
    (r"#444\b",    "rgba(0,0,0,0.16)"),
    (r"#333\b",    "rgba(0,0,0,0.16)"),
    (r"#222\b",    "rgba(0,0,0,0.80)"),
    (r"#111\b",    "rgba(0,0,0,0.80)"),

    # ── Dark hover / glass alphas → light alpha ──
    (r"rgba\(\s*255\s*,\s*255\s*,\s*255\s*,\s*0\.0[3-9]\s*\)", "rgba(0,0,0,0.04)"),
    (r"rgba\(\s*255\s*,\s*255\s*,\s*255\s*,\s*0\.1\s*\)",      "rgba(0,0,0,0.04)"),
    (r"rgba\(\s*0\s*,\s*0\s*,\s*0\s*,\s*0\.5\s*\)",            "rgba(0,0,0,0.4)"),
    (r"rgba\(\s*16\s*,\s*16\s*,\s*40\s*,\s*0\.\d+\s*\)",       "rgba(255,255,255,0.72)"),

    # ── Old-accent alpha glows → Apple Blue alpha ──
    (r"rgba\(\s*233\s*,\s*69\s*,\s*96\s*,\s*0\.\d+\s*\)",      "rgba(0,113,227,0.08)"),
    (r"rgba\(\s*0\s*,\s*212\s*,\s*255\s*,\s*0\.\d+\s*\)",      "rgba(0,113,227,0.08)"),
    (r"rgba\(\s*111\s*,\s*168\s*,\s*220\s*,\s*0\.\d+\s*\)",    "rgba(0,113,227,0.08)"),
    (r"rgba\(\s*74\s*,\s*111\s*,\s*165\s*,\s*0\.\d+\s*\)",     "rgba(0,113,227,0.06)"),
    (r"rgba\(\s*83\s*,\s*216\s*,\s*251\s*,\s*0\.\d+\s*\)",     "rgba(0,113,227,0.30)"),
    (r"#00d4ff44\b",                                           "rgba(0,113,227,0.26)"),
    (r"#00d4ff33\b",                                           "rgba(0,113,227,0.20)"),

    # ── Fix text color that became transparent after earlier substitutions ──
    # (some dark-grey hex like #0f3460 / #333 / #555 were used as TEXT color,
    #  and generic border-tone mappings made them nearly invisible on light bg)
    (r"(color:\s*)rgba\(\s*0\s*,\s*0\s*,\s*0\s*,\s*0\.04\s*\)",  r"\1rgba(0, 0, 0, 0.56)"),
    (r"(color:\s*)rgba\(\s*0\s*,\s*0\s*,\s*0\s*,\s*0\.08\s*\)",  r"\1rgba(0, 0, 0, 0.56)"),
    (r"(color:\s*)rgba\(\s*0\s*,\s*0\s*,\s*0\s*,\s*0\.12\s*\)",  r"\1rgba(0, 0, 0, 0.56)"),
    (r"(color:\s*)rgba\(\s*0\s*,\s*0\s*,\s*0\s*,\s*0\.16\s*\)",  r"\1rgba(0, 0, 0, 0.56)"),

    # ── Font family: swap Segoe-based stack → SF Pro / Apple stack ──
    (r"font-family:\s*['\"]?Segoe UI['\"]?\s*,\s*Tahoma\s*,\s*Geneva\s*,\s*Verdana\s*,\s*sans-serif",
     "font-family: 'SF Pro Text', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Pretendard', system-ui, sans-serif"),
    (r"font-family:\s*['\"]?Segoe UI['\"]?\s*,\s*Tahoma\s*,\s*sans-serif",
     "font-family: 'SF Pro Text', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Pretendard', system-ui, sans-serif"),
    (r"font-family:\s*['\"]?Segoe UI['\"]?\s*,\s*sans-serif",
     "font-family: 'SF Pro Text', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Pretendard', system-ui, sans-serif"),
    (r"font-family:\s*-apple-system\s*,\s*BlinkMacSystemFont\s*,\s*['\"]?Segoe UI['\"]?\s*,\s*Roboto\s*,\s*['\"]?Helvetica Neue['\"]?\s*,\s*Arial\s*,\s*sans-serif",
     "font-family: 'SF Pro Text', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Pretendard', system-ui, sans-serif"),
]


def apply_replacements(text: str) -> str:
    for pattern, replacement in COLOR_MAP:
        text = re.sub(pattern, replacement, text, flags=re.IGNORECASE)
    return text


def inject_theme_link(text: str, relpath: str) -> str:
    if f'href="{relpath}"' in text:
        return text
    link = f'<link rel="stylesheet" href="{relpath}">\n'
    m = re.search(r"(<meta[^>]*viewport[^>]*>\s*\n)", text)
    if m:
        return text[: m.end()] + link + text[m.end() :]
    m2 = re.search(r"(<head[^>]*>\s*\n)", text)
    if m2:
        return text[: m2.end()] + link + text[m2.end() :]
    return text


def process(path: Path, relpath: str) -> bool:
    original = path.read_text(encoding="utf-8")
    text = inject_theme_link(original, relpath)
    text = apply_replacements(text)
    if text != original:
        path.write_text(text, encoding="utf-8")
        return True
    return False


def main() -> None:
    changed = 0
    total = 0

    for f in sorted(INTERACTIVE.glob("*.html")):
        total += 1
        if process(f, "../shared/theme.css"):
            changed += 1
            print(f"  ✓ interactive/{f.name}")
        else:
            print(f"  · interactive/{f.name} (no change)")

    if APP_HTML.exists():
        total += 1
        if process(APP_HTML, "shared/theme.css"):
            changed += 1
            print(f"  ✓ app.html")
        else:
            print(f"  · app.html (no change)")

    print(f"\n{changed}/{total} files modified")


if __name__ == "__main__":
    main()
