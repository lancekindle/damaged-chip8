a "damaged" CHIP8 emulator for the gameboy :)

Includes several classic Chip8 roms
pre-bundled. If you wish to add your own, please
open a pull request! You'll want to copy your .ch8
file next to all the others and call ROM_COPY
with your game name (without .ch8) at end

to compile the game, run `make` in your terminal.
the assembled ROM will be called chip8.gb

----------------------------
Currently compatible with RGBDS v0.3.10 and lower
This is an old release, will not work with newer
versions of RGBDS! (pull requests to address this
are welcome)
