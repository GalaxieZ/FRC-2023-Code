// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
public static final class DriveConstants {
    //ports
        public static final int dtFrontLeftPort = 1;
        public static final int dtBackLeftPort = 2;
        public static final int dtFrontRightPort = 3;
        public static final int dtBackRightPort = 4;

        public static final boolean kCurrentLimitingEnabled = true; // True/False to enable/disable limit feature.
        public static final int kPeakCurrentAmps = 15; // The "holding" current (amperes) to limit to when feature is activated.
        public static final int kPeakTimeMs = 0; // How long current must exceed threshold (seconds) before limiting occurs.
        public static final int kContCurrentAmps = 10; // Current must exceed this threshold (amperes) before limiting occurs.
        public static final int kTimeoutMs = 30; // Amount of time (in milliseconds) to wait for a specified element to be found before an error is thrown

        public static final double kDeadband = 0.05;

         // Speed constants
         public static final double kDriveSpeed = .95;
         public static final double kTurnSpeed = .90;

         public static final double cDriveSpeed = .95;
         public static final double cTurnSpeed = .95;
         public static final double kTurnToleranceDeg = 0;
         public static final double kTurnRateToleranceDegPerS = 0;

         public static final double ksVolts = 0.663;
         public static final double kvVoltSecondsPerMeter = 1.802;
         public static final double kaVoltSecondsSquaredPerMeter = 0.28;
         public static final double kPDriveVel = 5.0;

           // Physical measurements of DT
        public static final double kWheelRadius = .1524; // meters
        public static final double kTrackWidth = .61; // meters

        public static boolean kGyroReversed = true;

         public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
}

    public static class ElevatorConstants {
       public static final int dLeftElevatorMotor = 5;
       public static final int dRightElevatorMotor = 6;
        
        public static final int TOP_NODE_BOX = 0; //change to the encoder position on the elevator 0-1
        public static final int BOTTOM_NODE_BOX = 0; //change to the encoder postion 0-1
        public static final int GROUNDPICKUP = 0; //theoretically this should be 0
        public static final int LOWEr_BOX_NODE = 0; ///
    }

    public static class ArmConstants {
        public static final int dLeftArmMotor = 7; //chane these 
        public static final int dRightArmMotor = 8; //change these 
    
    

    }

    public static class ClawConstants {
        public static final int ClawSolenoid1 =  9; //change these 
        public static final int ClawSolenoid2 = 10; //change these 

        
    }
    public static class UnitConversionConstants {
        public static final double angleConversionFactor = (Math.PI / 180); // angles to radians
        public static final double distanceConversionFactor = 39.37; // inches to meters
    }
    public static final class PhysicsConstants {
        public static final double gAcceleration = 386.09; // in inches per second squared
    }
   
    public static final class IntakeConstants {
        public static final double kIntakeSpeed = 0.20;
        public static final int intakePort = 11;

        public static final int intakeSolenoidRight = 1;
        public static final int intakeSolenoidLeft = 2;
    }

    public static final class XboxConstants {
        public static final int intakeValue = 0;
    }

    public static final class HopperConstants {
        public static final double kHopperSpeed = 0.20;

        // SparkMAX CAN ports
        public static final int hopperPort = 4;

        // Solenoid ports
        public static final int hopperSolenoid = 3;
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}

