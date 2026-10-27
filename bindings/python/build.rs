fn main() {
    // Embedded unit tests need to locate libpython for non-system interpreters.
    // This helper does nothing for extension-module builds.
    pyo3_build_config::add_libpython_rpath_link_args();
}
