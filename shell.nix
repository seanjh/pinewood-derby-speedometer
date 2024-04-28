let
  nixpkgs = fetchTarball "https://github.com/NixOS/nixpkgs/tarball/nixos-23.11";
  pkgs = import nixpkgs { config = {}; overlays = []; };
in 

pkgs.mkShell {
  packages = with pkgs; [
    arduino-cli
    gnumake
    python311
    python311Packages.pyserial
  ];

  shellHook = ''
  arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
  arduino-cli core update-index
  arduino-cli core install esp32:esp32
  '';
}
