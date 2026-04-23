#!/usr/bin/env python3
"""
Build script: concatenate chapter_*.md files into the two language textareas of
guide.template.html, producing guide.html.

Layout:
  chapter_NN_*.md         — Korean source (ko)
  en/chapter_NN_*.md      — English source (en, optional)
  guide.template.html     — hand-edited template with {{MD_KO}} / {{MD_EN}}
  guide.html              — generated

Usage: python3 scripts/build_guide.py
"""

import glob
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPLATE = os.path.join(PROJECT_ROOT, "guide.template.html")
OUTPUT = os.path.join(PROJECT_ROOT, "guide.html")
EN_DIR = os.path.join(PROJECT_ROOT, "en")

EN_PLACEHOLDER = (
    "# English edition — coming soon\n\n"
    "The English edition is being translated. "
    "Switch back to 한국어 to read the current content.\n"
)


def read_chapters(lang):
    if lang == "ko":
        pattern = os.path.join(PROJECT_ROOT, "chapter_*.md")
    else:
        pattern = os.path.join(EN_DIR, "chapter_*.md")
    files = sorted(glob.glob(pattern))
    parts = []
    for f in files:
        with open(f, "r", encoding="utf-8") as fh:
            parts.append(fh.read())
    return "\n\n---\n\n".join(parts)


def escape_textarea(s):
    return s.replace("</textarea>", "&lt;/textarea&gt;")


def main():
    with open(TEMPLATE, "r", encoding="utf-8") as f:
        template = f.read()

    md_ko = read_chapters("ko")
    md_en = read_chapters("en")
    if not md_en.strip():
        md_en = EN_PLACEHOLDER

    html = template.replace("{{MD_KO}}", escape_textarea(md_ko))
    html = html.replace("{{MD_EN}}", escape_textarea(md_en))

    with open(OUTPUT, "w", encoding="utf-8") as f:
        f.write(html)

    print(f"Built {OUTPUT}")
    print(f"  KO: {len(md_ko):,} chars")
    print(f"  EN: {len(md_en):,} chars")
    print(f"  HTML: {len(html):,} chars")


if __name__ == "__main__":
    main()
