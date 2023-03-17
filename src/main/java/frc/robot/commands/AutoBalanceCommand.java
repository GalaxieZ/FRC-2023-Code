// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoBalanceCommand extends CommandBase {

    private final Drivetrain driveSubsystem;
    private final AHRS navx;

    private final double Kp = 0.03; // Proportional constant for balancing
    private final double Kd = 0.06; // Derivative constant for balancing
    private double prevError = 0; // Previous error for derivative control

    public AutoBalanceCommand(Drivetrain driveSubsystem, AHRS navx) {
        this.driveSubsystem = driveSubsystem;
        this.navx = navx;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double angle = navx.getPitch(); // Get pitch angle from navx gyro
        double error = -angle; // Calculate error (negative since we want to balance)

        // Calculate motor output using PID control
        double output = Kp * error + Kd * (error - prevError);
        prevError = error;

        // Set motor output for both sides of the drivetrain
        driveSubsystem.tankDrive(output, -output);
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the robot is balanced within a certain threshold
        return Math.abs(navx.getPitch()) < 2.0;
    }
}

//Have not tested it yet