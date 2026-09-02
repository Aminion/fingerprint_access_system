let
  moz_overlay = import (builtins.fetchTarball {
    url = "https://github.com/oxalica/rust-overlay/archive/860d7c835ab91bfc8972b67092f5f2db8e9390a0.tar.gz";
    sha256 = "03qkyspcg5a8xwnlvhakk9xdp5k26kf38gghn91sb8bl52dnnc85";
  });
  pkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
  # Pinned to a known-good nightly: at opt-level='z' the 2026-09-01 nightly's
  # LLVM emits __aeabi_uread4/uwrite4/uread8/uwrite8 calls on thumbv6m that
  # compiler_builtins doesn't implement, breaking release linking. opt-level
  # is now 's' in Cargo.toml, which avoids the bug, but the toolchain is
  # pinned too so it can't silently drift back into it (or a new one).
  rustBuild = pkgs.rust-bin.nightly."2026-09-01".default.override {
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