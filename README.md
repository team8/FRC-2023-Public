# Team 8 FRC 2023

Team 8's 2023 FRC code for [Eir](https://www.palyrobotics.com/robots/). Our code is written in Java.

## Robot
### Subsystems

* [Drivetrain](src/main/java/frc2023/subsystems/Drive.java)
  
  Four SDS MK4i swerve modules powered by Falcon 500s with a theoretical top speed of 16.3 ft/sec; 26.5" x 26.5" frame perimeter

* [Arm](src/main/java/frc2023/subsystems/Arm.java)
  
  Double-jointed arm with pneumatic cylinder actuated bottom pivot and motor driven top pivot powered by 2x NEOs at a 182.25:1 overall reduction

* [Pivot](src/main/java/frc2023/subsystems/Pivot.java)

  Chain and sprocket pivot driven by a NEO

* [Intake](src/main/java/frc2023/subsystems/Intake.java)

  Two-roller intake driven by a 775pro for grabbing cones and cubes from ground and substation in any orientation

## Setup Instructions

### General
1. Clone this repo with ``git clone git@github.com:team8/FRC-2023-Rewrite.git``
2. ``./gradlew build`` - builds the code
3. ``./gradlew simulateJava`` - simulates the robot code locally
4. ``./gradlew deploy`` - deploys the code on to the robot (make sure you are connected to the robot's wifi)
5. Have fun!

### IDE
We recommend using IntelliJ, however, Visual Studio Code and Eclipse both work.

## Code

### Packages
  
* [frc2023.robot](src/main/java/frc2023/robot)
    
    Contains all the central classes and functions used in the robot. We use [RobotState](src/main/java/frc2023/robot/RobotState.java) 
  to keep data on the state of the robot (velocity, vision targets, pneumatics state), and we use
  [Commands](src/main/java/frc2023/robot/Commands.java) in order to state we want to be done 
  (set the intake to go a certain speed, translate the drivetrain, etc.). [OperatorInterface](src/main/java/frc2023/robot/OperatorInterface.java) 
  interfaces with the drivers' controllers and updates [Commands](src/main/java/frc2023/robot/Commands.java) based on controls.
  
* [frc2023.subsystems](src/main/java/frc2023/subsystems)
  
    Contains all the subsystems. Each subsystem takes an instance of [RobotState](src/main/java/frc2023/robot/RobotState.java)
  and [Commands](src/main/java/frc2023/robot/Commands.java) in order to take what is wanted
  and turn and output that is applied to the hardware.
  
* [frc2023.behavior](src/main/java/frc2023/behavior)
  
    Handles all the [Routines](src/main/java/frc2023/behavior/RoutineBase.java).
  A Routine is a class that runs for a set period of time and updates commands in order to do something.
  Examples include intaking a cone, balancing on the charge station, raising the arm, etc.

* [frc2023.auto](src/main/java/frc2023/auto)
  
    Handles all the autos. Each auto is a list of Routines.

* [frc2023.util](src/main/java/frc2023/util)
  
    Contains a lot of utility classes and functions. Also contains things that don't belong anywhere else.

* [frc2023.config](src/main/java/frc2023/config)
  
    Contains both constants and configs that can be reloaded without recompiling.

## License

Our code is released under the MIT License. A copy of this license is included in the LICENSE file.
