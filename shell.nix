# { pkgs ? import <nixpkgs> {} }:
with import <nixpkgs> {};
let
  
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    openjdk17
    libGL
  ];

  shellHook = ''
    export JAVA_HOME=${pkgs.openjdk17}
  '';
}
