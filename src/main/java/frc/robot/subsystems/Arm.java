// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import frc.robot.subsystems.Intake;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

public class Arm extends SubsystemBase {

  public double throughBoreEncoderOffset = 0.49;
  public double tolerance = 1;
  public double throughBoreEncoderUsableValue;
  // 1 - 0.50866;
  // 1 - 0.509721113920211792;

  // private DutyCycleEncoder armAbsoluteEncoder = new DutyCycleEncoder();
  
  CANSparkMax leftArmMotor = new CANSparkMax(22, MotorType.kBrushless); // - = up FOR SURE
  CANSparkMax rightArmMotor = new CANSparkMax(24, MotorType.kBrushless); // + = up PROBABLY
  
  
  // RelativeEncoder leftEncoder = leftArmMotor.getEncoder();

  PIDController armPidController = new PIDController(0.00000001, 0.00, 0.0);

  DigitalInput armLimitSwitch = new DigitalInput(0);

  String setPoint;

  private int pos = 1;


  /*Shuffleboard Initializations */
  private ShuffleboardTab tab = Shuffleboard.getTab("Arm Setpoints Degrees");


  private GenericEntry inputAmpSetpoint = tab.add("Amp Setpoint", 20).
      withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 100))
      .withSize(4, 1).getEntry();

  private GenericEntry encoderDegreesReadout =

      tab.add("Encoder Degrees Readout", 0)
         .getEntry();


  /** Creates a new Arm. */
  public Arm() {
    rightArmMotor.setSmartCurrentLimit(30);
    leftArmMotor.setSmartCurrentLimit(30);

    rightArmMotor.follow(leftArmMotor, true);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    armPidController.enableContinuousInput(0, 360);
    //armPidController.setSetpoint(0);

    // Intake.throughBoreEncoder.setZeroOffset(throughBoreEncoderOffset);
    Intake.throughBoreEncoder.setInverted(true);
    // leftEncoder.setPosition(0);
  }

  public void moveRightArmUp() {
    rightArmMotor.set(0.3);
  }


  public void setArmPosition(double setpoint) {

    armPidController.setSetpoint(-setpoint);  
  }

  public void testPID() {
    
    
  }

  public void setPickupPosition() {
    armPidController.setPID(0.0075, 0, 0.0);
    // if (getPositionDegrees() > 0){
    //   armPidController.setPID(0.01, 0.001, 0.0);
    // } else {
    //   armPidController.setPID(0.3, 0.0, 0.0);
    // }
    // if (setPoint.equals("Transition")) {
      armPidController.setSetpoint(2);
      pos = 1;
    // }
  }

  public void setTransitionalPosition() {
    // armPidController.setPID(0.1, 0, 0);
    // if (getPositionDegrees() > 10){
    //   armPidController.setPID(0.01, 0.001, 0.0);
    // } else {
    //   armPidController.setPID(0.3, 0.0, 0.0);
    // }
    armPidController.setSetpoint(-10);
    setPoint = "Transition";
  }
  
  public void setTopPosition() {
    // if (getPositionDegrees() > 40){
    //   armPidController.setPID(0.01, 0.001, 0.0);
    // } else {
    //   armPidController.setPID(0.3, 0.0, 0.0);
    // }

    armPidController.setPID(0.007, 0, 0);   
    var result = aprilCam.getLatestResults();
    if(!result.hasTargets()){
      System.out.println("Could not find targets; defaulting to 90 degrees");
      // default to 90 degrees
      armPidController.setSetpoint(90);
    }
    else{
      // trust photonvision model to know the best target
      PhotonTrackedTarget target = result.getBestTarget();
      // getting a full pose is unnecessary, because we can already get what we need without calculations
      double pitch = target.getPitch();
      // in future may want to add more sanity checks on the returned value
      // we can assume it will be greater than zero, because if our robot is _above_ an AprilTag
      // we have much bigger problems
      if(pitch < 0){
        pitch = 0;
      };

      // put upper boundary at 90 degrees
      if(pitch > 90){
        pitch = 90;
      };
      armPidController.setSetpoint(pitch);
    }

    pos = 3;
    
  }

  public void setDefensePosition() {
    // if (getPositionDegrees() > 40){
    //   armPidController.setPID(0.01, 0.001, 0.0);
    // } else {
    //   armPidController.setPID(0.3, 0.0, 0.0);
    // }
      armPidController.setPID(0.007, 0, 0);
            // System.out.println("going up to top");
    
    armPidController.setSetpoint(60);  
    pos = 3;
  }

  public void setFrontSubwooferPosition() {
    if (pos == 3) {
      armPidController.setPID(0.006, 0, 0);
      // System.out.println("going down to subwoofer");
    } else {
      armPidController.setPID(0.04, 0.001, 0);
      // System.out.println("going up to subwoofer");
    }
    armPidController.setSetpoint(15);
    pos = 2;
    
  }

  public void autonTopToSubwooferPosition() {
      armPidController.setPID(0.006, 0, 0);
      armPidController.setSetpoint(15);
      // System.out.println("going down to subwoofer");
  
    }
  
    public void manualArmPositionUp() {
      // if (getPositionDegrees() > 40){
      //   armPidController.setPID(0.01, 0.001, 0.0);
      // } else {
      //   armPidController.setPID(0.3, 0.0, 0.0);
      // }
        armPidController.setPID(0.007, 0, 0);
              // System.out.println("going up to top");
        
      armPidController.setSetpoint(armPidController.getSetpoint()+1);
      
    }

    public void manualArmPositionDown() {
      // if (getPositionDegrees() > 40){
      //   armPidController.setPID(0.01, 0.001, 0.0);
      // } else {
      //   armPidController.setPID(0.3, 0.0, 0.0);
      // }
        armPidController.setPID(0.007, 0, 0);
              // System.out.println("going up to top");
        
      armPidController.setSetpoint(armPidController.getSetpoint()-1);
      
    }
  
  // public void autonArmTopToBottomInitialization() {
  //   while (armLimitSwitch.get() == true) {
  //    leftArmMotor.set(-0.2);
  //  }
  //   leftArmMotor.set(0);
  //   // Intake.throughBoreEncoder.setPositionConversionFactor(1/256);
    
  //   System.out.println("--------------------------------------------------------------------------------------------------------------------------------------");
  // }


  

  public Command setArmPower(double power){
    return runEnd(
      () -> leftArmMotor.set(power),
      () -> leftArmMotor.set(0)
      
      ); 
    }
    
    /*Converts Neo internal motor encoder to degrees */
    public double getPositionDegrees() {
      return (Units.rotationsToDegrees(Intake.throughBoreEncoder.getPosition()));
    }
    

    @Override
    public void periodic() {
      
      /*This method will be called once per scheduler run  */
      SmartDashboard.putNumber("Through Bore Encoder Readout", getPositionDegrees());
      encoderDegreesReadout.setDouble(getPositionDegrees());

      SmartDashboard.putNumber("Arm Current Draw", leftArmMotor.getOutputCurrent());

      // System.out.println(Intake.throughBoreEncoder.getPosition()*360);
      // System.out.println(Intake.throughBoreEncoder.getPosition());


      /* Limits Arm Maximum Angle, if maximum arm angle is reached will set arm speed to 0 */
      // if(getPositionDegrees() >= 80){
      //   leftArmMotor.set(0);
      // } else {
      throughBoreEncoderUsableValue = Intake.throughBoreEncoder.getPosition();
      
      leftArmMotor.set(armPidController.calculate(Intake.throughBoreEncoder.getPosition()));
      // }

      /*Resets neo encoder position to 0 when arm hits limit switch */

     // if (armLimitSwitch.get() == true) {
       // System.out.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");        
     // }

      // if (leftArmMotor.getOutputCurrent() >= 40) {
      //   throughBoreEncoderOffset = Intake.throughBoreEncoder.getPosition() * 360;
      //   Intake.throughBoreEncoder.setZeroOffset(throughBoreEncoderOffset);
      // }

      //if (Math.abs(Intake.throughBoreEncoder.getPosition()*360 -360) <= tolerance) {
        //throughBoreEncoderOffset = Intake.throughBoreEncoder.getPosition();
       // Intake.throughBoreEncoder.setZeroOffset(throughBoreEncoderOffset);
      //}

      //System.out.println(((Intake.throughBoreEncoder.getPosition())));

      //if (armLimitSwitch.get() == true) {
        //throughBoreEncoderOffset = Intake.throughBoreEncoder.getPosition();
        //Intake.throughBoreEncoder.setZeroOffset(throughBoreEncoderOffset);

     // }
    }
  }
