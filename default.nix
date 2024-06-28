{
  lib,
  stdenv,
  cmake,
  hpp-core,
  hpp-template-corba,
  pkg-config,
  psmisc,
  python3Packages,
}:

python3Packages.buildPythonPackage {
  pname = "hpp-corbaserver";
  version = "5.0.0";
  pyproject = false;

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

  prePatch = ''
    substituteInPlace tests/hppcorbaserver.sh \
      --replace-fail /bin/bash ${stdenv.shell}
  '';

  nativeBuildInputs = [
    cmake
    pkg-config
  ];
  propagatedBuildInputs = [
    hpp-core
    hpp-template-corba
    python3Packages.omniorbpy
  ];
  checkInputs = [ psmisc ];

  enableParallelBuilding = false;

  doCheck = true;

  pythonImportsCheck = [ "hpp.corbaserver" ];

  meta = {
    description = "Corba server for Humanoid Path Planner applications";
    homepage = "https://github.com/humanoid-path-planner/hpp-corbaserver";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
