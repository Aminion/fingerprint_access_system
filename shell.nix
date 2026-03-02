let
  moz_overlay = import (builtins.fetchTarball "https://github.com/oxalica/rust-overlay/archive/master.tar.gz");
  pkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
  rustBuild = pkgs.rust-bin.nightly.latest.default.override {
    targets = [ "thumbv6m-none-eabi" ];
    extensions = [ "rust-src" "rust-analyzer" "llvm-tools"]; 
  };
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    rustBuild
    probe-rs-tools
    pkg-config
    libusb1
    cargo-binutils
    cargo-bloat
    
  ];
}