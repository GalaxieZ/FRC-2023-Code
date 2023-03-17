// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorToPositionCommand extends CommandBase {
 

    private final ElevatorSubsystem elevator;
    private final ElevatorPosition position;

    public MoveElevatorToPositionCommand(ElevatorSubsystem elevator, ElevatorPosition position) {
        this.elevator = elevator;
        this.position = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSetpoint(position.getPosition());
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    public enum ElevatorPosition {
        BOTTOM(0),
        MIDDLE(1000),
        TOP(2000);

        private final int position;

        ElevatorPosition(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }
    }
}
