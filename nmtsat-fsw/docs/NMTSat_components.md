# NMTSat Component list

## Power

| Component | Model | Purpose | Direction | Data Rate | Notes |
|-----------|-------|---------|-----------|-----------|-------|
| EPS | TBD | Power regulation, distribution, load switching | Read/Write | LOW : ~1KB/s | Controls battery charge/discharge, solar MPPT, circuit protections |

## Payload

| Component | Model | Purpose | Direction | Data Rate | Notes |
|-----------|-------|---------|-----------|-----------|-------|
| VLF receiver | TBD | E-field component of VLF | Read | MID : ~100KB/s | Mission instrument |
| Search coil magnetometer | TBD | B-field component of VLF  | Read | MID : ~100KB/s | Mission instrument; on same boom as VHF beacon antenna |
| Beacon transmitter | TBD | 3-signal for ionospheric tomography | Write | VERY LOW : < 1KB/s | 150MHz, 400MHz, 1067MHz |
| Optics (TBD) | TBD | TBD | TBD | TBD | Considered secondary and still being evaluated; no longer in systems diagrams |

## Radio

| Component | Model | Purpose | Direction | Data Rate | Notes |
|-----------|-------|---------|-----------|-----------|-------|
| X/UHF radio | TBD | Uplink/downlink communications | Read/Write | VERY HIGH : > 10MB/s | Telemetry and C&C. Will ask on freq |

## ADCS (May interface with an ADCS subsystem instead of ADCS components, TBD)

| Component | Model | Purpose | Direction | Data Rate | Notes |
|-----------|-------|---------|-----------|-----------|-------|
| Navigation magnetometer | TBD | B-field vector measurement for attitude determination | Read | LOW : ~1KB/s | ADCS use only |
| Star tracker | TBD | Attitude determination via star pattern matching | Read | LOW : ~1KB/s | Likely to become its own subsystem; rest of ADCS likely uses main compute |
| Sun sensor | TBD | Coarse sun vector measurement | Read | LOW : ~1KB/s | |
| Gyroscope | TBD | Angular rate measurement | Read | LOW : ~1KB/s | |
| Magnetorquer | TBD | Attitude control via magnetic torque | Write | LOW : ~1KB/s | |
| Reaction wheel | TBD | Precision attitude control | Read/Write | LOW : ~1KB/s | |

## Navigation

| Component | Model | Purpose | Direction | Data Rate | Notes |
|-----------|-------|---------|-----------|-----------|-------|
| GPS receiver | TBD | Orbit determination, position and velocity fix | Read | LOW : ~1KB/s | |

## General sensors

| Component | Model | Purpose | Direction | Data Rate | Notes |
|-----------|-------|---------|-----------|-----------|-------|
| Various (TBD) | TBD | TBD | TBD | TBD | To be distributed in various locations across the satellite |

## Controls

| Component | Model | Purpose | Direction | Data Rate | Notes |
|-----------|-------|---------|-----------|-----------|-------|
| VLF antenna deploy | TBD | Deploy VLF receive antenna | Write | VERY LOW : < 1KB/s | Burnwire deployment |
| VHF beacon antenna / search coil boom deploy (TBD) | TBD | Deploy VHF beacon antenna and search coil magnetometer boom | Write | VERY LOW : < 1KB/s | Burnwire deployment; search coil and beacon antenna share this boom |
| UHF antenna deploy (TBD) | TBD | Deploy UHF antenna | Write | VERY LOW : < 1KB/s | Burnwire deployment |
| Solar panel deploy | TBD | Deploy solar panels | Write | VERY LOW : < 1KB/s | Burnwire deployment |
