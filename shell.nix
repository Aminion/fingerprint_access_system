let
  moz_overlay = import (builtins.fetchTarball {
    url = "https://github.com/oxalica/rust-overlay/archive/404b7a5537f0adc6e5c558a1b94612e683dec504.tar.gz";
    sha256 = "07n35cdbifgcnzsh66h7s4rsfg7ya7jjqvvh5pfykp8szxwvjq6p";
  });
  pkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
  # Pinned to a known-good nightly so the toolchain can't silently drift into
  # a codegen regression. Nightlies up to 2026-09-01 failed to link release at
  # opt-level='z' on thumbv6m (missing __aeabi_uread4/uwrite4/uread8/uwrite8 in
  # compiler_builtins); fixed by 2026-10-12, but 'z' is ~1.4 KB larger than 's'
  # here, so Cargo.toml stays on 's'.
  rustBuild = pkgs.rust-bin.nightly."2026-10-12".default.override {
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