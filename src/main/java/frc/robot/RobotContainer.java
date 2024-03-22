// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import org.photonvision.PhotonCamera;

public class RobotContainer {
  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  Arm arm = new Arm();

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(driverJoystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverJoystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);


/*+++++++++++++++++++++++++++++++DRIVER CONTROLER END+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/




    operatorJoystick.leftBumper().whileTrue(intake.runIntakeUntilBeamBreakCommand());
    
    operatorJoystick.rightBumper().whileTrue(intake.setIntakePower(-0.9));
    operatorJoystick.leftTrigger().whileTrue(intake.setIntakePower(0.9));

    operatorJoystick.rightTrigger().whileFalse((new InstantCommand(()-> shooter.setShooterPower(0), shooter)));
    operatorJoystick.rightTrigger().whileTrue((new InstantCommand(()-> shooter.setShooterPower(operatorJoystick.getRightTriggerAxis()), shooter)));
    // operatorJoystick.rightTrigger().whileTrue(shooter.setShooterPower(operatorJoystick.getRightTriggerAxis()));
    
    // operatorJoystick.x().whileTrue(shooter.setShooterPower(0.7));
    
    operatorJoystick.povDown().whileTrue(arm.setArmPower(-0.1));
    operatorJoystick.povUp().whileTrue(arm.setArmPower(0.1));

    
    // operatorJoystick.b().whileTrue(new InstantCommand(()-> arm.setArmPosition(40), arm)).onFalse(new InstantCommand(()-> arm.setArmPosition(20), arm));
    operatorJoystick.b().whileTrue(new InstantCommand(()-> arm.setFrontSubwooferPosition(), arm));
    operatorJoystick.a().whileTrue(new InstantCommand(()-> arm.setPickupPosition(), arm));
    operatorJoystick.y().whileTrue(new InstantCommand(()-> arm.setTopPosition(), arm));

  }

  public Command oneNoteScoreAtSubwoofer() {
    return new SequentialCommandGroup(
    
    new InstantCommand(()-> arm.autonTopToSubwooferPosition(), arm),
    new InstantCommand(()-> shooter.setShooterPower(0.6), shooter),
    new WaitCommand(5),
    new InstantCommand(()-> intake.autonomousIntakePower(0.7), intake));

}

  private final SendableChooser<Command> autoChooser;  

  public RobotContainer() {
    drivetrain.getModule(0).getDriveMotor().getConfigurator().refresh(TunerConstants.driverRamp);
    drivetrain.getModule(0).getSteerMotor().getConfigurator().refresh(TunerConstants.steerRamp);
    drivetrain.getModule(1).getDriveMotor().getConfigurator().refresh(TunerConstants.driverRamp);
    drivetrain.getModule(1).getSteerMotor().getConfigurator().refresh(TunerConstants.steerRamp);
    drivetrain.getModule(2).getDriveMotor().getConfigurator().refresh(TunerConstants.driverRamp);
    drivetrain.getModule(2).getSteerMotor().getConfigurator().refresh(TunerConstants.steerRamp);
    drivetrain.getModule(3).getDriveMotor().getConfigurator().refresh(TunerConstants.driverRamp);
    drivetrain.getModule(3).getSteerMotor().getConfigurator().refresh(TunerConstants.steerRamp);


    NamedCommands.registerCommand("printSomething", drivetrain.printSomething("++++++++++++++"));
    NamedCommands.registerCommand("oneNoteSubwooferRoutine", oneNoteScoreAtSubwoofer());

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }




  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
