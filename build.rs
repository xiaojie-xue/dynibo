#[cfg(feature = "pinocchio-tests")]
fn main() {
    println!("cargo:rerun-if-changed=tests/support/pinocchio_bridge.cpp");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");

    let pinocchio = pkg_config::Config::new()
        .cargo_metadata(true)
        .probe("pinocchio")
        .unwrap_or_else(|error| {
            panic!(
                "Pinocchio support requested, but pkg-config could not find pinocchio: {error}\n\
                 Set PKG_CONFIG_PATH to the directory containing pinocchio.pc."
            )
        });

    let mut bridge = cc::Build::new();
    bridge
        .cpp(true)
        .warnings(false)
        .file("tests/support/pinocchio_bridge.cpp")
        .flag_if_supported("-std=c++17")
        .flag_if_supported("-O3")
        .define("NDEBUG", None);
    for include in pinocchio.include_paths {
        bridge.include(include);
    }
    bridge.compile("dynibo_pinocchio_bridge");
}

#[cfg(not(feature = "pinocchio-tests"))]
fn main() {}
