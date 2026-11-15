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

        default_target = "core_main_prod";

        build = pkgs.writeShellScriptBin "build" ''
          root=$(git rev-parse --show-toplevel)
          pio run -d "$root/core_main" -e "''${1:-${default_target}}"
        '';
        upload = pkgs.writeShellScriptBin "upload" ''
          root=$(git rev-parse --show-toplevel)
          pio run -d "$root/core_main" -e "''${1:-${default_target}}" -t upload
        '';
      in
      {
        devShells.default = pkgs.mkShell {
          name = "core-embedded";
          packages = with pkgs; [
            platformio
            build
            upload
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
            echo "  build [target]  - build (default target: ${default_target})"
            echo "  upload [target] - build & upload (default target: ${default_target})"
          '';
        };

        checks.formatting = treefmtEval.config.build.check self;
        formatter = treefmtEval.config.build.wrapper;
      }
    );
}
