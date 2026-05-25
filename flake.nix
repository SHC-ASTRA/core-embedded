{
  description = "Biosensor embedded dev shell";

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
          name = "biosensor-embedded";
          packages = with pkgs; [
            platformio
          ];

          shellHook = ''
	    if [[ ! -f core_main/compile_commands.json ]]; then
	      export COMPILATIONDB_INCLUDE_TOOLCHAIN=True
	      pio run -d core_main -t compiledb
	      unset COMPILATIONDB_INCLUDE_TOOLCHAIN
	    fi

            echo "core-embedded dev shell"
            echo "  pio run -d core_main -t main_prod - build production"
            echo "  pio run -d core_main -t main_dev  - build development"
          '';
        };

        checks.formatting = treefmtEval.config.build.check self;
        formatter = treefmtEval.config.build.wrapper;
      }
    );
}
