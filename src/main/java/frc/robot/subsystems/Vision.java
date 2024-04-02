// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*
 * 
 * 
 * 
 * LOOK AT THIS REPO
 * https://github.com/Pantherbotics/FRC-2024-Crescendo/blob/main/src/main/java/frc/robot/subsystems/Vision.java
 * 
 * 
 * 
 */
public class Vision extends SubsystemBase {
    
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    PhotonCamera noteCam = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
    PhotonCamera aprilCam = new PhotonCamera("USB_Camera");
  
    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  /** Creates a new Vision. */
  public Vision() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    noteCam.setDriverMode(true);
    noteCam.setPipelineIndex(1); // Note pipeline
    aprilCam.setDriverMode(false);
    aprilCam.setPipelineIndex(2); // AprilTag pipeline

  }

  // public getSpeakerTagAngle() {

  // }

  // PhotonPoseEstimator photonPoseEstimate = new PhotonPoseEstimator(null, null, null)
  boolean hasTargets;
  List<PhotonTrackedTarget> targets;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     var result = camera.getLatestResult();
     hasTargets = result.hasTargets();

    // Get a list of currently tracked targets.
    targets = result.getTargets();


  }
  // Hypothetically Added Code for auto align
  // 90909000909090909090900909090

  // HYPOTHETICAL UNTESTED CODE
  // WORKING ON MORE COOOODE

  // Commit 2
}
