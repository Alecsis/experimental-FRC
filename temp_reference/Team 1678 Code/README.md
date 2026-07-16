# C2026-Public
Code for Team 1678's 2026 robot, Limestone.

![Robot Image](/images/Limestone.png)

See also:
- Our CAD Release and Scouting Whitepaper
- Code from [2024](https://github.com/frc1678/C2024-Public/) and [2025](https://github.com/frc1678/C2025-Public/).

## PowerSlice: A custom Power Analyzer
 New for 2026, we developed PowerSlice: a custom Power Analyzer that takes in .wpilog files and outputs metrics on power consumption. If you are intrested in using our app please download it [here](https://drive.google.com/drive/folders/1V3aWsZ3ZWTn6AuyU0q038G_n137dYQmz?usp=sharing)

## Highlights
- Targeting
  
  Limestone features our fully custom targeting system [located here](/src/main/java/frc/robot/shooting/targeting/Targeting.java) that dynamically selects where on the field our robot would target if it was shooting at any given instant 

- LEDs
  
    Depending on the state that the robot is in, the LEDs are constantly updating to avoid having to look away from the robot to know what state it's in. Examples of these states include different colors for when robot is ready to shoot verus when the robot is not ready. For more information, you can look [at our LED subsystem](/src/main/java/frc/robot/subsystems/leds/LEDs.java) for reference.

- Controls

    LImestone only requires a single controller to utilize all of its functionality--the second controller serves as a debug controller incase something goes wrong.
  
    Our controlboard layout, [located here](/src/main/java/frc/robot/controlboard/ControlBoard.java), shows how we map all the buttons on our controllers to various different features

- Vision

    Limestone has one Limelight 4 positioned below the shooter drum, which is always used for AprilTag detection and pose estimation.

## Notable Package Functions
- [`frc.robot.autos`](/src/main/java/frc/robot/autos/)

  Contains the base files along with specific auto routines grouped by type. Also contains special commads which handle our bump crossing logic along with how we time the shooting time with the contraction of our expandable hoppper

- [`frc.robot.subsystems`](/src/main/java/frc/robot/subsystems/)

  Contains all subsystems

- [`frc.robot.subsystems.superstructure`](/src/main/java/frc/robot/subsystems/superstructure/)

  Holds all inter-subsystem sequencing

- [`frc.lib.io`](/src/main/java/frc/lib/io/)

  Holds all IO files used for our subsystems.

  
