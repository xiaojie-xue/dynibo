# Translating the dynibo documentation

English is the source language. Update the English page first, then update its
adjacent `.zh.md` translation in the same change when practical. If a Chinese
page is missing, the site deliberately falls back to English.

Keep API identifiers, code, equations, URDF names, and units unchanged. On
first use, technical terms may use the form `中文（English）`; later uses should
prefer the concise Chinese term. Keep headings and examples aligned between the
two files so readers land at the corresponding content when switching language.

The Rust, Python, C++, and C generated API references remain English-only. Their
source comments and docstrings are the single source of truth; do not create
translated copies of generated pages.

Before submitting documentation changes, run:

```bash
mkdocs build --strict
```
