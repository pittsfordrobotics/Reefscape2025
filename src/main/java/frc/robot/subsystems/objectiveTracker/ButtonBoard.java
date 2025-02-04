// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectiveTracker;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonBoard extends SubsystemBase {
  private BooleanSupplier[] buttonSupplier;
  private boolean[] buttons;
  private int level = 1;
  private boolean rightSide = false;
  private int reefSide = 0;
  /** This class is used to control the Objective Selector with the button board. */
  public ButtonBoard(BooleanSupplier[] buttonSupplier) {
    this.buttonSupplier = buttonSupplier;
  }

  @Override
  public void periodic() {
    // Take button inputs and update the level, rightSide, and reefSide variables
    for(int i = 0; i < buttonSupplier.length; i++) {
      buttons[i] = buttonSupplier[i].getAsBoolean();
    }

    // Update level
    for(int i=5; i<=8; i++) {
      if(buttons[i]) {
        level = 9 - i;
      }
    }

    // Check if the left/right buttons are pressed and decide which side the robot is on
    if(buttons[1] || buttons[2]) rightSide = buttons[1];
    
    if(buttons[3]) {
      reefSide++;
    } else if(buttons[4]) {
      reefSide+=5; // have to do this so that reefSide never goes negative, basically subtract 1
    }
    reefSide %= 6; //maintain 0-6 range
  }

  public int getLevel() {
    return level;
  }
  
  public boolean getRightSide() {
    return rightSide;
  }
  /** @return The side of the reef the robot is on, to be shown in the ObjectiveTracker app */
  public int getReefSide() {
    return reefSide;
  }

  /** @return The current selection index to be shown in the ObjectiveTracker app */
  public int getSelectedIndex() {
    return (rightSide ? 1 : 0) + (4 - level) * 2;
  }
}
