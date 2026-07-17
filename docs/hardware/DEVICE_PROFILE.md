# IPE IRGML-14-I Device Profile

This profile records values observed on the physical joint used by this project.
It is not a substitute for an official EtherCAT device description or a signed
manufacturer specification.

## Identity and bus layout

| Field | Observed value |
| --- | --- |
| EtherCAT name | `CoE Drives` |
| Vendor ID | `0x00041101` |
| Product ID | `0x00009253` |
| Configured address | `1001` |
| Distributed clocks | Supported; SYNC0 used |
| Expected WKC | `3` for one configured slave |

## Process data

- Output PDO size: 160 bits
- Input PDO size: 256 bits
- SM2 output: 20 bytes at `0x1100`
- SM3 input: 32 bytes at `0x1400`
- Mailbox protocols observed: CoE, FoE, and EoE

The project validates vendor and product identity before enabling motion.

## Joint calibration used by this checkout

- Absolute encoder resolution: 18 bits, or 262,144 counts per encoder revolution
- Measured reduction ratio: approximately 101:1
- Rated motor speed from the supplied data sheet: 3,000 rpm
- Derived rated flange speed: approximately 29.7 rpm
- ROS zero count: machine-specific; currently configured by bringup
- ROS direction: `-1` for this installation

`velocity_raw_to_rad_s` is based on the measured 101:1 ratio. Recalibrate it for
a joint with a different reduction ratio. The raw torque-to-Nm scale has not been
confirmed, so torque commands remain explicitly named `torque_raw`.

## Supporting local evidence

Raw `slaveinfo`, SDO, PDO-map, and terminal captures are stored under the ignored
`artifacts/` directory. They may contain device-specific information and are not
published to Git.
