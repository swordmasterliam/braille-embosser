# CAD Files

## Active Assembly (`CanNewAssymbly/`)

Contains the final hybrid Tetrix Prime + custom 3D-printed SolidWorks assembly (`Assem1.SLDASM`) along with all custom printed parts and directly referenced structural beam models so SolidWorks opens the assembly without missing part links:

- **Carriage & Gantry Mounts**: `Part1.SLDPRT` through `Part7.SLDPRT` (two-piece ball-bearing carriage, X-motor housing, lowered idler mount, slotted Z-motor housing, sensor brackets)
- **Z-Axis Punch**: `gearRack1.SLDPRT` (vertical rack & tapered stylus), `60 Tooth Gear (228-2500-215).SLDPRT`
- **Y-Axis Single Roller Mounts**: `RollerMotorHolder.SLDPRT`, `RollerAxelHolder.SLDPRT` / `.STEP`, `RollerMotorLid.SLDPRT` / `.STEP`
- **Indented Die Base Plate**: `BrailleBasePlate.SLDPRT` / `.STEP`

> **Print Note:** Verify the hole pitch in `BrailleBasePlate.SLDPRT` (`2.5 mm` left edge offset, `3.5 mm` intra-cell spacing, `5.5 mm` inter-cell gap / `9.0 mm` character pitch, `2.5 mm` cavity diameter) matches `firmware/braille_embosser.cpp` before slicing, and print a labeled spare plate.

## Superseded Iterations (`archive/`)

Kept for design-history reference only (do not print for the final build):
- `BASE.SLDPRT` and `1010-aluminum-extrusion-1.snapshot.1/` - Dropped Iteration 2 1010 aluminum extrusion frame
- `MainAssymbly.SLDASM` and `TetrixFrameV2.SLDASM` - Early Iteration 3 Tetrix base assemblies
