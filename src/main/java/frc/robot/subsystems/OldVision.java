// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.net.PortForwarder;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.util.Units;

// /*
//  * 
//  * 
//  * 
//  * LOOK AT THIS REPO
//  * https://github.com/Pantherbotics/FRC-2024-Crescendo/blob/main/src/main/java/frc/robot/subsystems/Vision.java
//  * 
//  * 
//  * 
//  */
// public class OldVision extends SubsystemBase {
    
//     final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
//     final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
//     // Angle between horizontal and the camera.
//     final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

//     // How far from the target we want to be
//     final double GOAL_RANGE_METERS = Units.feetToMeters(3);

//     // Change this to match the name of your camera
//     public PhotonCamera camera;
  
//     // The field from AprilTagFields will be different depending on the game.
//     public AprilTagFieldLayout aprilTagFieldLayout;
	
// 	// PhotonVision pose estimator
// 	public PhotonPoseEstimator poseEstimator;

//   /** Creates a new Vision. */
//   public OldVision() {
//     camera = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
// 	aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
// 	poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.kRobotToMainCam);
//     PortForwarder.add(5800, "photonvision.local", 5800);
// //    noteCam.setDriverMode(true);
// //    noteCam.setPipelineIndex(1); // Note pipeline
//     camera.setDriverMode(false);
//     camera.setPipelineIndex(2); // AprilTag pipeline

//   }

//   // public getSpeakerTagAngle() {

//   // }

//   // PhotonPoseEstimator photonPoseEstimate = new PhotonPoseEstimator(null, null, null)
//   private boolean hasTargets;
//   private List<PhotonTrackedTarget> targets;
//   private Optional<EstimatedRobotPose> estimated;
//   private final Field2d m_field = new Field2d();
  
//   public void updatePose(){
// 	estimated = poseEstimator.update(camera.getLatestResult());
	
// 	if(estimated.isPresent()){
// 		m_field.setRobotPose(estimated.get().estimatedPose.toPose2d());
// 	}
	
//   }
  
//   @Override
//   public void periodic() {
//     updatePose();
//   }
// }