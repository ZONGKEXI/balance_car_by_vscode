# RGB PWM DMA host checks

Run from the repository root with a native GCC compiler available:

```powershell
& Test/host/rgb_dma/run.ps1
```

The runner compiles the actual `Core/Src/app_rgb.c` against a small hardware
stub. No production source is copied into the tests, and no hardware is opened
or programmed. Compiler warnings are treated as errors. The executable is
written to `build/host-tests/rgb-dma/`.

The checks cover the initial black frame, failed reinitialization and recovery,
GRB/MSB-first encoding, reset padding,
DMA buffer ownership, queued updates, Clear, callback routing, the 20 ms refresh
interval, synchronized five-color breathing, tick rollover, a large elapsed
interval, failed DMA starts, DMA errors, the 3 ms timeout, and nonblocking calls.

To reproduce the initial failure with an older driver without running every
case:

```powershell
& Test/host/rgb_dma/run.ps1 -SourcePath path/to/old/app_rgb.c -InitOnly
```

The weak no-op callbacks and `App_RGB_Proc` in the test harness allow that older
source to link. The new driver's real functions override them, and the full
behavior checks fail if those functions are absent.

These tests verify data and driver state transitions on the host. They do not
measure PA0 pulse timing, electrical levels, or the physical LED response.
