// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorLevels;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

/** Add your docs here. */
public class AlgaeCommands {
    public static Command algaeIntake(Algae algae, Elevator elevator, ObjectiveTracker objectiveTracker) {
        return elevator.dynamicElevatorLevel(objectiveTracker::getElevatorLevel).alongWith(algae.intakeAlgae());
    }

    public static Command algaeOuttake(Algae algae, Elevator elevator, ObjectiveTracker objectiveTracker) {
        return elevator
                .elevatorLevelUntilReached(
                        () -> objectiveTracker.getElevatorLevel() == ElevatorLevels.L4 ? ElevatorLevels.BARGE
                                : ElevatorLevels.PROCESSOR)
                .andThen(algae.outtakeAlgae());
    }
}
