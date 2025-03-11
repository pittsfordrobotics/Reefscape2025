// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator.ElevatorLevels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoral extends SequentialCommandGroup {
  /** Intake coral, sets elevator to intake level and stops when staged */
  public IntakeCoral(Intake intake, Coral coral, Elevator elevator) {
    addCommands(
        elevator.setElevatorLevel(ElevatorLevels.INTAKE).until(() -> elevator.isAtLevel(() -> ElevatorLevels.INTAKE)),
        intake.intakeCoral().alongWith(coral.intakeCoral()),
        Commands.waitUntil(coral::isCoralDetected),
        intake.slowIntakeCoral(),
        Commands.waitUntil(() -> !coral.isCoralDetected()),
        intake.stopIntake().alongWith(coral.stopCoral()));
  }
}
