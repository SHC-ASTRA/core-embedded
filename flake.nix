{
  description = "Core embedded dev shell";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      ...
    }@inputs:
    inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import inputs.nixpkgs { inherit system; };
        treefmtEval = inputs.treefmt-nix.lib.evalModule pkgs {
          projectRootFile = "flake.nix";
          programs = {
            clang-format.enable = true;
            nixfmt.enable = true;
          };
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "core-embedded";
          packages = with pkgs; [
            platformio
          ];

          shellHook = ''
            for d in $(find . -mindepth 2 -maxdepth 2 -name "platformio.ini" -printf '%h\n'); do
              if [[ ! -f "$d"/compile_commands.json ]]; then
                export COMPILATIONDB_INCLUDE_TOOLCHAIN=True
                pio run -d "$d" -t compiledb
                unset COMPILATIONDB_INCLUDE_TOOLCHAIN
              fi
            done

            echo "core-embedded dev shell"
            echo "  pio run -d core_main -e core_main_prod -t upload"
            echo "  pio run -d core_main -e core_main_dev -t upload"
          '';
        };

        checks.formatting = treefmtEval.config.build.check self;
        formatter = treefmtEval.config.build.wrapper;
      }
    );
}
