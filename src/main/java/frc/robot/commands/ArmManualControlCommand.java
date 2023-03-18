// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualControlCommand extends CommandBase {

  private final XboxController xboxController;
  private final ArmSubsystem armSubsystem;

  public ArmManualControlCommand(XboxController xboxController, ArmSubsystem armSubsystem) {
    this.xboxController = xboxController;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }


  @Override
  public void execute() {
    double speed = 0;

    // Check the POV button on the controller to determine the speed and direction of the arm
    switch (xboxController.getPOV()) {
      case 0: // Up
        speed = 0.5;
        break;
      case 180: // Down
        speed = -0.5;
        break;
      default: // No button pressed
        break;
    }

    ArmSubsystem.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop(); // Stop the arm when the command ends
  }

}
