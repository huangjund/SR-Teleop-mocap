# MocapApi
MocapApi for Axis Studio of Noitom

## Document
English Documentation : https://mocap-api.noitom.com/mocap_api_en.html


## Build dependencies

The demos use CMake, the Noitom MocapApi SDK, and GLFW/OpenGL for the optional
viewer. Make sure the following are available on your system:

- A C++17 compiler
- CMake 3.15+
- MocapApi headers and libraries (already included in `include/` and
  `lib/win32/x64` in this repo)
- GLFW and OpenGL development packages (e.g. on Ubuntu:
  `sudo apt-get install libglfw3-dev libgl1-mesa-dev`)

On Windows you can also run the provided `install.bat` to set up SDK paths.

## Build

```bash
mkdir -p build
cd build
cmake ..
cmake --build . --config Release
```

This produces two executables under `build/`:

- `mocap_demo` – console program that prints hand wrist poses and finger angles
  at 1 Hz.
- `mocap_hands_viewer` – GLFW/OpenGL viewer that renders both hands.

On Windows, `MocapApi.dll` is copied next to the built binaries automatically.
On Linux, ensure `libMocapApi.so` (if provided for your platform) is discoverable
via `LD_LIBRARY_PATH` or next to the executable.

## Run the demos

1. In Axis Studio, broadcast BVH over UDP to `127.0.0.1:7012`.
2. From the `build` directory run either:
   - Console printer: `./mocap_demo`
   - Viewer: `./mocap_hands_viewer`

Both programs will attempt to connect using the default IP/port. If you need a
different endpoint, adjust the `HandMocapReceiver` constructor arguments in
`src/main.cpp` or `src/main_viewer.cpp` and rebuild.
