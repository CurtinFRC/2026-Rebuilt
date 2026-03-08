# Team 4788 Formatting Standards

- package names are all lowercase, for example `frc.robot.drive`
- class names are always upper camel case, for example `MotorController`
- variable and function names are lower camel case, for example `hoodPosition`
- imports are all at the top of the file
- all visibility modifiers should be explicitly declared
- prefer final variables where possible
- constants should be final, static and named in SCREAMING_CASE
- constants should be defined at the top of a subsystem
- the formatter will enforce things like tab spacing
- variables should be in SI units, except angles which use rotations. common units used include
    - volts (Voltage)
    - amps (Amperage)
    - rotations (Angle)
    - meters (Distance)
    - seconds (Time)
- when naming a variable with a unit it should be named in plural form for example, `outputVolts`
