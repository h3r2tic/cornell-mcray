# 🕹️ Cornell McRay t'Racing

A quick'n'dirty game sample using [`kajiya`](https://github.com/EmbarkStudios/kajiya)

https://user-images.githubusercontent.com/16522064/125960227-7ee05c04-f47a-4c32-b494-cc36dc70ab63.mp4

## Building

Sync to the same parent directory that `kajiya` is in, so that the folder structure becomes:

```
kajiya/
kajiya/assets
(...)
cornell-mcray/
(...)
```

Build the `bake` bin in the `kajiya` folder:

```
cd kajiya
cargo build --release -p bake
```

Bake the meshes for `cornell-mcray`:

```
bake.cmd
```

Make sure `dxcompiler.dll` is in the executable environment. You can grab it from `kajiya` and copy into `cornell-mcray`, or stash it somewhere in `PATH`

Run the game!

```
cargo run --release
```
