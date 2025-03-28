// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoral extends SequentialCommandGroup {
  /** Intake coral, sets elevator to intake level and stops when staged */
  public IntakeCoral(Coral coral, Elevator elevator) {
    addCommands(
        Commands.runOnce(() -> coral.setIntaken(false)),
        elevator.setElevatorLevel(ElevatorLevels.INTAKE).andThen(Commands.waitUntil(() -> elevator.isAtLevel(() -> ElevatorLevels.INTAKE))),
        coral.intakeCoral(),
        Commands.waitUntil(coral::isCoralDetected),
        Commands.waitUntil(() -> !coral.isCoralDetected()),
        coral.retractCoral(),
        Commands.waitUntil(coral::isCoralDetected),
        coral.stopCoral(),
        Commands.runOnce(() -> coral.setIntaken(true)));
  }
}
