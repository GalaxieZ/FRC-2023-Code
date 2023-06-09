// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

// import frc.robot.Constants;
// import frc.robot.Constants.PneumaticConstants;
public class Claw extends SubsystemBase {
  //meathods for controling subsystems
  /** Creates a new Claw. */
  DoubleSolenoid DoubleSolenoidR = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  DoubleSolenoid DoubleSolenoidL = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
  
    // define solenoid states
    private final DoubleSolenoid.Value forward = DoubleSolenoid.Value.kForward;
    private final DoubleSolenoid.Value reverse = DoubleSolenoid.Value.kReverse;
    private final DoubleSolenoid.Value off = DoubleSolenoid.Value.kOff;

  public Claw() {
    
  }
  public void extendSolenoid() {
    DoubleSolenoidR.set(forward);
    DoubleSolenoidL.set(forward);
}

// method to retract solenoid
public void retractSolenoid() {
  DoubleSolenoidR.set(reverse);
  DoubleSolenoidL.set(reverse);
}

// method to turn off solenoid
public void stopSolenoid() {
  DoubleSolenoidR.set(off);
  DoubleSolenoidL.set(off);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

