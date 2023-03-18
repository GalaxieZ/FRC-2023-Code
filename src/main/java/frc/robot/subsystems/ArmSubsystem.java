// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  
  private final double kP = 0.1;
  private final double kI = 0.0;
  private final double kD = 0.0;
  
  private final CANSparkMax motor1;
 // private final CANSparkMax motor2;
  
  private final PIDController controller1;
  //private final PIDController controller2;
  
  public ArmSubsystem() {
    motor1 = new CANSparkMax(10, MotorType.kBrushless);
   // motor2 = new CANSparkMax(2, MotorType.kBrushless);
    
    controller1 = new PIDController(kP, kI, kD);
//controller2 = new PIDController(kP, kI, kD);
    
    motor1.getEncoder().setPositionConversionFactor(1.0 / 64.0); // set encoder conversion factor based on motor gearing
   // motor2.getEncoder().setPositionConversionFactor(1.0 / 64.0); // set encoder conversion factor based on motor gearing
  }
  
  public void setArmPosition(double position) {
    double output1 = controller1.calculate(motor1.getEncoder().getPosition(), position);
    //double output2 = controller2.calculate(motor2.getEncoder().getPosition(), position);
    
    motor1.set(output1);
   // motor2.set(output2);
  }

  @Override
  public void periodic() {
    // Any code placed here will be run on a periodic basis as part of the subsystem
  }

  public Object stop() {
    return null;
  }

  public double getPosition() {
    return 0;
  }

public static void setSpeed(double speed) {
}
  
}
