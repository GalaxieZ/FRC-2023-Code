// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase {

  // Spark MAX motor controller for the elevator motor
  private final CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless);

  // PID controller for the motor
  private final ProfiledPIDController pidController = new ProfiledPIDController(
      0.1, 0.0, 0.0,
      new TrapezoidProfile.Constraints(10.0, 5.0));

  // Current setpoint for the motor
  private double setpoint = 0.0;

  public ElevatorSubsystem() {
    // Set the maximum output voltage for the Spark MAX controller
    motor.getPIDController().setOutputRange(-12.0, 12.0);

    // Reset the encoder on the motor
    motor.getEncoder().setPosition(0.0);
  }

  // Set the desired position of the elevator
  public void setPosition(double position) {
    // Set the setpoint for the PID controller
    setpoint = position;
  }

  // Move the elevator to the desired position
  public void move() {
    // Calculate the output of the PID controller
    double output = pidController.calculate(motor.getEncoder().getPosition(), setpoint);

    // Set the output of the Spark MAX controller
    motor.getPIDController().setReference(output, CANSparkMax.ControlType.kDutyCycle);
  }

  // Stop the elevator motor
  public void stop() {
    // Set the output of the Spark MAX controller to zero
    motor.getPIDController().setReference(0.0, CANSparkMax.ControlType.kDutyCycle);
  }

  // Check if the elevator has reached its target position
  public boolean atTargetPosition() {
    // Return true if the position error is within a tolerance
    return Math.abs(pidController.getPositionError()) < 0.1;
  }

  // Override the default command to stop the elevator
  @Override
  public void periodic() {
    stop();
  }

  public void setSetpoint(int position) {
  }

public boolean atSetpoint() {
    return false;
}

public double getPosition() {
    return 0;
}

}
