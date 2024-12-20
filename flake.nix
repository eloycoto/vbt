{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };
  outputs = { self, nixpkgs, flake-utils, rust-overlay }:
    flake-utils.lib.eachDefaultSystem
      (system:
        let
          overlays = [ (import rust-overlay) ];
          pkgs = import nixpkgs {
            inherit system overlays;
          };
        in
        with pkgs;
        {
          devShells.default = mkShell {
            buildInputs = with pkgs; [
              (rust-bin.stable.latest.default.override {
                extensions = [ "rust-src" ];
                targets = [ "riscv32imc-unknown-none-elf" ];
              })
            ] ++  [
              openocd
              rust-analyzer
              rustfmt
              probe-run
              flip-link
              probe-rs
              espflash
              python3Packages.bleak
              python3Packages.ipython
              python3Packages.ipdb
              # bluez
              # kicad
              # kicadAddons.kikit-library
              # kicadAddons.kikit
              minicom
            ];
          shellHook = ''
            export GDK_BACKEND=x11
          '';
          };
        }
      );
}
