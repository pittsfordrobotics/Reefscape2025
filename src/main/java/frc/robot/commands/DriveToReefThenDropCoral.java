// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToReefThenDropCoral extends SequentialCommandGroup {
  /** Creates a new DriveToReefThenDropCoral. */
  public DriveToReefThenDropCoral(Swerve swerve, Coral coral, Elevator elevator, ObjectiveTracker tracker) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        swerve.driveToReef(tracker::isRightSide),
        elevator.dynamicElevatorLevel(tracker::getElevatorLevel).until(
          () -> elevator.isAtLevel(tracker::getElevatorLevel))
      ),
      coral.placeCoral()
    );
  }
}
