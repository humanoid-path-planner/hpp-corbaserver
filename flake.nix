{
  description = "Corba server for Humanoid Path Planner applications";

  nixConfig = {
    extra-substituters = [ "https://gepetto.cachix.org" ];
    extra-trusted-public-keys = [ "gepetto.cachix.org-1:toswMl31VewC0jGkN6+gOelO2Yom0SOHzPwJMY2XiDY=" ];
  };

  inputs = {
    nixpkgs.url = "github:nim65s/nixpkgs/gepetto";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    hpp-core = {
      url = "github:humanoid-path-planner/hpp-core";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-parts.follows = "flake-parts";
      };
    };
    hpp-template-corba = {
      url = "github:humanoid-path-planner/hpp-template-corba";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-parts.follows = "flake-parts";
        hpp-util.follows = "hpp-core/hpp-constraints/hpp-util";
      };
    };
  };

  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [ ];
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        {
          self',
          pkgs,
          system,
          ...
        }:
        {
          packages = {
            inherit (pkgs) cachix;
            default = pkgs.callPackage ./. {
              hpp-core = inputs.hpp-core.packages.${system}.default;
              hpp-template-corba = inputs.hpp-template-corba.packages.${system}.default;
            };
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
        };
    };
}
