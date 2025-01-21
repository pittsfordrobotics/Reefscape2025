// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectiveTracker;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBoard extends SubsystemBase {
  
  public interface ButtonSupplier {
    boolean getButtonState(int buttonNumber);
  }

  private ButtonSupplier buttonSupplier;
  private int level = 1;
  private boolean rightSide = false;
  private int reefSide = 0;

  /** Creates a new ButtonBoard. */
  public ButtonBoard(ButtonSupplier buttonSupplier) {
    this.buttonSupplier = buttonSupplier;
  }

  @Override
  public void periodic() {
    if(buttonSupplier.getButtonState(5)) {
      level = 4;
    }

    else if(buttonSupplier.getButtonState(6)) {
      level = 3;
    }

    else if(buttonSupplier.getButtonState(7)) {
      level = 2;
    }

    else if(buttonSupplier.getButtonState(8)) {
      level = 1;
    }

    if(buttonSupplier.getButtonState(1)) {
      rightSide = true;
    }

    else if(buttonSupplier.getButtonState(2)) {
      rightSide = false;
    }
    
    if(buttonSupplier.getButtonState(3)) {
      if(reefSide == 5) {
        reefSide = 0;
      } else {
        reefSide++;
      }
    }

    else if(buttonSupplier.getButtonState(4)) {
      if(reefSide == 0) {
        reefSide = 5;
      } else {
        reefSide--;
      }
    }
    // This method will be called once per scheduler run
  }

  public int getLevel() {
    return level;
  }
  
  public boolean getRightSide() {
    return rightSide;
  }

  public int getReefSide() {
    return reefSide;
  }

  /** @return The current selection index to be shown in the ObjectiveTracker app */
  public int getSelectedIndex() {
    return rightSide ? 1 : 0 + (4 - level) * 2;
  }
}
