// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectiveTracker;

import org.ejml.dense.row.decomposition.eig.SwitchingEigenDecomposition_FDRM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIO.MoveDirection;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIO.ObjectiveSelectorInputs;

public class ObjectiveTracker extends SubsystemBase {
  private final ObjectiveSelectorIO objectiveSelector;
  private final ObjectiveSelectorInputs selectorInputs = new ObjectiveSelectorInputs();

  /** Creates a new ObjectiveTracker. */
  public ObjectiveTracker(ObjectiveSelectorIO objectiveSelector) {
    this.objectiveSelector = objectiveSelector;
  }

  @Override
  public void periodic() {
    objectiveSelector.updateInputs(selectorInputs);
  }

  public String getObjectiveString() {
    String s = (selectorInputs.selectedIndex % 2 == 0) ? "L" : "R";
    int level = 4 - (selectorInputs.selectedIndex / 2);
    
    return s + level;
  }

  public int getObjectiveIndex() {
    return selectorInputs.selectedIndex;
  }

  public boolean isRightSide() {
    return selectorInputs.selectedIndex % 2 == 1;
  }

  public Command moveIndex(MoveDirection direction) {
    return this.runOnce(() -> moveIndexInternal(direction)).ignoringDisable(true);
  }

  private void moveIndexInternal(MoveDirection direction)
  {
    // Indexes are a 2x4 rectangle with 0,0 at the upper left.
    int newIndex = selectorInputs.selectedIndex;
    
    switch (direction) {
      case UP:
        newIndex -= 2;
        if (newIndex < 0)  {
          newIndex += 8;
        }
        break;
      case DOWN:
        newIndex += 2;
        if (newIndex > 7) {
          newIndex -= 8;
        }
        break;
      case LEFT:
        newIndex -= 1;
        if ((newIndex + 1) % 2 == 0) {
          newIndex += 2;
        }
        break;
      case RIGHT:
        newIndex += 1;
        if (newIndex % 2 == 0) {
          newIndex -= 2;
        }
        break;
      default:
        // Unknown direction
        break;
    }

    objectiveSelector.setIndex(newIndex);
  }
}
