{
  description = "Corba server for Humanoid Path Planner applications";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/refs/pull/362956/head";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        {
          pkgs,
          self',
          system,
          ...
        }:
        {
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [
              (final: prev: {
                hpp-core = prev.hpp-core.overrideAttrs (super: {
                  patches = (super.patches or [ ]) ++ [
                    (final.fetchpatch {
                      url = "https://github.com/humanoid-path-planner/hpp-core/commit/21c46cdcb870e5f28e95a0b5f6e01099d5f8fefb.patch";
                      hash = "sha256-NMLUkAY0mrr9ktn42iUGbAvZ8bGi9y39w5gQZiofmtA=";
                    })
                  ];
                });
              })
            ];
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.hpp-corbaserver;
            hpp-corbaserver = pkgs.python3Packages.hpp-corbaserver.overrideAttrs (_: {
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./cmake-modules
                  ./CMakeLists.txt
                  ./doc
                  ./idl
                  ./include
                  ./package.xml
                  ./src
                  ./tests
                ];
              };
            });
          };
        };
    };
}
