{ pkgs ? import <nixpkgs> {} }:
let
  
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    openjdk17
  ];

  shellHook = ''
    export JAVA_HOME=${pkgs.openjdk17}
  '';
}
