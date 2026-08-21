"""Compatibility hooks for the dynibo documentation build."""

from pathlib import Path
import shutil

from mkdocs.plugins import event_priority

_command = "build"
_docs_root = None


def on_startup(*, command, dirty):
    """Remember whether generated copies must remain stable for live reload."""
    del dirty
    global _command
    _command = command


def _clean_generated_refs(config):
    global _docs_root
    docs_root = Path(config.docs_dir)
    _docs_root = docs_root
    for directory in ("c-api", "cpp-api"):
        shutil.rmtree(docs_root / directory, ignore_errors=True)


def on_config(config):
    """Remove generated reference copies left by an interrupted build."""
    _clean_generated_refs(config)
    return config


@event_priority(-50)
def on_files(files, config):
    """Make mkdoxy pages visible to the i18n plugin.

    mkdoxy constructs generated Markdown files with a relative source root.
    mkdocs-static-i18n classifies documentation using absolute source paths, so
    normalize only the generated C and C++ reference pages after mkdoxy runs
    and before i18n filters the file collection.
    """
    project_root = Path(config.config_file_path).parent
    docs_root = Path(config.docs_dir)
    generated_prefixes = ("c-api/", "cpp-api/")

    generated = {}
    for file in list(files):
        if file.src_uri.startswith(generated_prefixes):
            generated.setdefault(file.src_uri, []).append(file)

    for src_uri, candidates in generated.items():
        destination = (docs_root / src_uri).resolve()
        sources = []
        for file in candidates:
            source = Path(file.abs_src_path)
            if not source.is_absolute():
                source = (project_root / source).resolve()
            sources.append(source)

        # Prefer mkdoxy's cache as the source when a previous live build has
        # also made the destination discoverable by MkDocs.
        source = next((path for path in sources if path != destination), sources[0])
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.exists() or source.read_bytes() != destination.read_bytes():
            shutil.copyfile(source, destination)

        # A live rebuild can contain both the discovered copy and mkdoxy's
        # generated File. Keep one normalized entry so i18n sees no duplicates.
        keep = candidates[0]
        keep.abs_src_path = str(destination)
        for duplicate in candidates[1:]:
            files.remove(duplicate)

    return files


def on_post_build(config):
    """Keep generated reference copies out of the source tree after a build."""
    if _command != "serve":
        _clean_generated_refs(config)


@event_priority(0)
def on_page_context(context, page, config, nav):
    """Use prefix-safe relative language links on the two homepages."""
    del nav
    if getattr(page.file, "norm_src_uri", page.file.src_uri) == "index.md":
        config.extra.alternate = [
            {"name": "English", "link": "./", "lang": "en"},
            {"name": "简体中文", "link": "./zh/", "lang": "zh"},
        ]
    return context


def on_template_context(context, template_name, config):
    """Keep the shared 404 page's language links at the site root."""
    if template_name == "404.html":
        config.extra.alternate = [
            {"name": "English", "link": "./", "lang": "en"},
            {"name": "简体中文", "link": "./zh/", "lang": "zh"},
        ]
    return context


def on_shutdown():
    """Clean reference copies retained for the live server."""
    if _docs_root is not None:
        for directory in ("c-api", "cpp-api"):
            shutil.rmtree(_docs_root / directory, ignore_errors=True)
