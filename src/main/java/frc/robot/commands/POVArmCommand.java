// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class POVArmCommand extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final XboxController controller;

  public POVArmCommand(ArmSubsystem armSubsystem, XboxController controller) {
    this.armSubsystem = armSubsystem;
    this.controller = controller;
    addRequirements(armSubsystem);
  }

  @Override
  public void execute() {
    int povValue = controller.getPOV();
    double position = armSubsystem.getPosition();
    if (povValue == 0) {
      position += 0.1; // move arm up
    } else if (povValue == 180) {
      position -= 0.1; // move arm down
    }
    armSubsystem.setArmPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}

