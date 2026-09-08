# MSPM0 command-line closed-loop workflow

All scripts are non-GUI and pin the target to `MSPM0G3507`, SWD, 1 MHz and J-Link
serial `69514110` by default. Override those parameters when using another board.

```powershell
# Build with Keil and require a zero-warning result
.\scripts\Build-KeilFirmware.ps1

# Flash and verify. The explicit switch records that motor output is safe.
.\scripts\Flash-JLink.ps1 -ConfirmMotorSafe

# Capture RTT only
.\scripts\Capture-JLinkRtt.ps1 -OutputPath .\logs\closed_loop\manual-rtt.log -DurationSeconds 30

# Full raised-wheel qualification: build, flash, five repeated runs, strict analysis
.\scripts\Test-MotorQualification.ps1 -Lifted -Build -Flash -Runs 5
```

Evidence is saved under `logs/closed_loop/`. A PASS proves only the unloaded,
raised-wheel speed-control bench criteria. It permits a supervised low-speed
straight ground test; it does not prove loaded line-following performance.

