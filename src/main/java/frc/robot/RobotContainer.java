// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArmManualControlCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.MoveElevatorToPositionCommand;
import frc.robot.commands.POVArmCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.SwitchDriveMode;

//marker
import frc.robot.commands.Solenoids.ExtendSolenoidCommand;
import frc.robot.commands.Solenoids.RetractSolenoidCommand;
import frc.robot.commands.Solenoids.StopSolenoidCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.commands.TurnToTarget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and comm ands are defined here...
  public static final Drivetrain m_drivetrain = new Drivetrain();

  public static final XboxController m_driverController = new XboxController(0);
  public static final XboxController m_driverController2 = new XboxController(1);

  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();


  

  //public static final AHRS m_gyro;


  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    SlewRateLimiter filter = new SlewRateLimiter(2.6, 0, 0);

   // ArmManualControlCommand armManualControlCommand = new ArmManualControlCommand(m_driverController, m_arm);
    
   /* m_arm.setDefaultCommand(new RunCommand(() -> m_arm.stop(), m_arm));
    configureButtonBindings(); */

    m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.stop(), m_elevator));
    configureButtonBindings();
     
    //armManualControlCommand.schedule();
    
    if (m_driverController.getLeftBumper()) {
      //SLEWRATELIMITERTOGGLED
      m_drivetrain.setDefaultCommand(new RunCommand(() ->
          m_drivetrain.arcadeDrive(
              filter.calculate(-m_driverController.getLeftY()),
              (-m_driverController.getRightX())), m_drivetrain));
  } else {
      m_drivetrain.setDefaultCommand(new RunCommand(() ->
          m_drivetrain.arcadeDrive(
              m_driverController.getLeftY(),
              -m_driverController.getRightX()), m_drivetrain));
    m_drivetrain.setDefaultCommand(new RunCommand(() ->
    m_drivetrain.arcadeDrive(
      -m_driverController.getLeftY(), -m_driverController.getRightX()), m_drivetrain));
  }

    Drivetrain.setDefaultCommand(m_driverController);

    chooser.addOption("curvy path", loadPathplannerTrajectoryToRamseteCommand(
        "pathplanner/generatedJSON/curvy.wpilib.json",
        true));
        chooser.addOption("Jerk Path", loadPathplannerTrajectoryToRamseteCommand(
        "pathplanner/generatedJSON/JerkPath.wpilib.json",
        true));
    chooser.addOption("FowardOnly", loadPathplannerTrajectoryToRamseteCommand(
          "pathplanner/generatedJSON/FowardOnly.wpilib.json",
          true));
    chooser.addOption("straight", loadPathplannerTrajectoryToRamseteCommand(
        "pathplanner/generatedJSON/straight.wpilib.json",
        true));
    chooser.addOption("testpath", loadPathplannerTrajectoryToRamseteCommand(
        "pathplanner/generatedJSON/testpath.wpilib.json",
        true));
    chooser.addOption("test2", loadPathplannerTrajectoryToRamseteCommand(
        "pathplanner/generatedJSON/test2.wpilib.json",
        true));
    chooser.addOption("test32", loadPathplannerTrajectoryToRamseteCommand(
          "pathplanner/generatedJSON/test32.wpilib.json",
          true));
    chooser.addOption("New Path", loadPathplannerTrajectoryToRamseteCommand(
            "pathplanner/generatedJSON/New Path.wpilib.json",
            true));
     chooser.addOption("TestLeft", loadPathplannerTrajectoryToRamseteCommand(
            "pathplanner/generatedJSON/TestLeft.wpilib.json",
            true));
     chooser.addOption("testwithlib", loadPathplannerTrajectoryToRamseteCommand(
              "pathplanner/generatedJSON/testwithlib.wpilib.json",
              true));

    Shuffleboard.getTab("Autonomous").add(chooser);

  
  }

  public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdomtry) {
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();

      
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drivetrain::tankDriveVolts,
        m_drivetrain);

    if (resetOdomtry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } else {
      return ramseteCommand;
    }

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kRightBumper.value)
  .onTrue(new SwitchDriveMode(m_drivetrain));

  
//This is for manual control. Also for finding encoder positions
 /*   new POVButton(m_driverController, 0)
  .whileTrue(new MoveElevatorCommand(m_elevator, () -> 1.0));

  new POVButton(m_driverController, 180)
  .whileTrue(new MoveElevatorCommand(m_elevator, () -> -1.0)); */


  // Need To update positions in MoveELevator
  new JoystickButton(m_driverController, Button.kY.value)
  .onTrue(new MoveElevatorToPositionCommand(m_elevator, MoveElevatorToPositionCommand.ElevatorPosition.TOP));

  new JoystickButton(m_driverController, Button.kB.value)
  .onTrue(new MoveElevatorToPositionCommand(m_elevator, MoveElevatorToPositionCommand.ElevatorPosition.MIDDLE));

  new JoystickButton(m_driverController, Button.kA.value)
  .onTrue(new MoveElevatorToPositionCommand(m_elevator, MoveElevatorToPositionCommand.ElevatorPosition.BOTTOM));

    
 // new JoystickButton(m_driverController, Button.kLeftBumper.value)
 // .onTrue(new AutoBalanceCommand(m_drivetrain, m_gyro));

 /*new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new RunCommand(() -> m_intake.setIntake(-.2), m_elevator))
        .onFalse(new RunCommand(() -> m_intake.setIntake(0), m_elevator));*/

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }

  public Drivetrain getDriveTrainSubsystem() {
    return m_drivetrain;
  }

}
