// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new NewVision. */
  public final PhotonCamera mainCam;
  public final PhotonCamera backCam;
  public AprilTagFieldLayout tagLayout;
  public PhotonPoseEstimator mainPoseEstimator;
  public PhotonPoseEstimator backPoseEstimator;
  private Optional<EstimatedRobotPose> mainEstimated;
  private Optional<EstimatedRobotPose> backEstimated;
  private CommandSwerveDrivetrain swerve;
  public PhotonPipelineResult notePipelineResult;

  public PhotonPipelineResult mainCamPipelineResult = new PhotonPipelineResult();
  public PhotonPipelineResult backCamPipelineResult = new PhotonPipelineResult();


  private final Field2d m_field = new Field2d();

/*
 * 
 *                                          TODO
 * 
 *  1) Get kMainCameraName and kBackCameraName from photonVision UI and input them into VisionConstants.java
 *  3) Test code using advantagescope to check pose output
 *  2) Not yet but eventually: kRobotToMainCam and kRobotToBackCam Variables
 * 
 * 
 * 
 * 
 */
  

  public Vision(CommandSwerveDrivetrain swerve) {
    mainCam = new PhotonCamera(VisionConstants.kMainCameraName);
    backCam = new PhotonCamera(VisionConstants.kBackCameraName);
    this.swerve = swerve;
    tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // mainPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mainCam, VisionConstants.kRobotToMainCam);
    // backPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam, VisionConstants.kRobotToBackCam);
    mainPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToMainCam);
    backPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToBackCam);
    

    SmartDashboard.putData("cam poses", m_field);
    
    
  }

  public void updateAiprilTagPipelineResults() {
    if (mainCam.getAllUnreadResults().size() > 0) {
      mainCamPipelineResult = mainCam.getAllUnreadResults().get(mainCam.getAllUnreadResults().size()-1);
    }

    if (backCam.getAllUnreadResults().size() > 0) {
      backCamPipelineResult = backCam.getAllUnreadResults().get(backCam.getAllUnreadResults().size()-1);
    }
    
  }

  public void updatePose(){
    // mainEstimated = mainPoseEstimator.update(mainCamPipelineResult);
    // backEstimated = backPoseEstimator.update(backCamPipelineResult);

    mainEstimated = mainPoseEstimator.update(mainCam.getLatestResult());
    backEstimated = backPoseEstimator.update(backCam.getLatestResult());
    
    if (mainEstimated.isPresent()){
      swerve.addVisionMeasurement(mainEstimated.get().estimatedPose.toPose2d(), mainEstimated.get().timestampSeconds);
      m_field.setRobotPose(mainEstimated.get().estimatedPose.toPose2d());
    }
    if (backEstimated.isPresent()){
      swerve.addVisionMeasurement(backEstimated.get().estimatedPose.toPose2d(), backEstimated.get().timestampSeconds);
      m_field.getObject("backCam").setPose(backEstimated.get().estimatedPose.toPose2d());
    }

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateAiprilTagPipelineResults();
    updatePose();
  }
}

