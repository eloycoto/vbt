# shell.nix
{ pkgs ? import <nixpkgs> {} }:

let
  python-env = pkgs.python3.withPackages (ps: with ps; [
    numpy
    ipython
    ipdb
    scipy
    requests
    matplotlib
    # ruff
  ]);
in
pkgs.mkShell {
  buildInputs = [
    python-env
  ];

  shellHook = ''
    export PYTHONPATH="${python-env}/lib/python${pkgs.python3.pythonVersion}/site-packages:$PYTHONPATH"
    echo "Python development environment loaded with numpy, ipython, ipdb, and ruff"
  '';
}
