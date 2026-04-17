{
  description = "Corba server for Humanoid Path Planner applications";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.hpp-corbaserver = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
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
        };
      }
    );
}
