// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
  import java.util.function.*;

public class MoveElevatorCommand extends CommandBase {
 
    
    private final ElevatorSubsystem elevator;
    private final Supplier<Double> positionSupplier;
    
    public MoveElevatorCommand(ElevatorSubsystem elevator, Supplier<Double> positionSupplier) {
      this.elevator = elevator;
      this.positionSupplier = positionSupplier;
      addRequirements(elevator);
    }
    
    @Override
    public void initialize() {
      double position = positionSupplier.get();
      elevator.setPosition(position);
      SmartDashboard.putNumber("Elevator Setpoint", position);
    }
    
    @Override
    public void execute() {
      elevator.move();
      SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
    }
    
    @Override
    public boolean isFinished() {
      return elevator.atTargetPosition();
    }
    
    @Override
    public void end(boolean interrupted) {
      elevator.stop();
    }
    
  }
  
