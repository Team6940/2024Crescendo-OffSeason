// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis.DriveSubsystem;

public class objectTracker extends SubsystemBase {
  DriveSubsystem m_DriveSubsystem;
  VisionIO m_visionIO;

  public List<Translation2d> othersPoses= new ArrayList<>();
  public List<Translation2d> gamePiecePoses= new ArrayList<>();
  public List<TrackingPose> trackedOthersPoses= new ArrayList<>();
  public List<TrackingPose> trackedGamePiecePoses= new ArrayList<>();
  public Translation2d bestGamePiecePose=new Translation2d(100,100);
  public Translation2d mostDangerousRobot=new Translation2d(100,100);

  private Field2d gamePiecePose = new Field2d();
  private Field2d enemyPose = new Field2d();

  private static final double EMA_ALPHA = 0.9;
  private static final double TIMEOUT = 0.2; // Keep pose for 1 second if not detected
  private static final double MERGE_DISTANCE_THRESHOLD = 0.3;

  /** Creates a new Vision. */
  public objectTracker(VisionIO visionIO,DriveSubsystem driveSubsystem) {
    this.m_visionIO=visionIO;
    this.m_DriveSubsystem=driveSubsystem;

    SmartDashboard.putData("gamePiecePose",gamePiecePose); 
    SmartDashboard.putData("enemyPose",enemyPose); 
  }

  public boolean inField(Translation2d pose){
    return (pose.getY()>0&&pose.getY()<8.21)&&((pose.getX()>0&&pose.getX()<16.54));
  }
  // public List<Translation2d> getFieldToObjectPoses(List<TimestampedObjectTranslation> robotToObjectPoses){
  //   List<Translation2d> fieldToObjectPoses=new ArrayList<>();
  //   for(TimestampedObjectTranslation robotToObjectPose: robotToObjectPoses){
  //     if(robotToObjectPose.confidence>Constants.BlackholeVisionConstants.confiTolerance){
  //       Pose2d fieldToRobotPose=m_DriveSubsystem.getPoseWithTime(robotToObjectPose.timestamp);
  //       Transform2d robotToObjectTranform2d=new Transform2d(robotToObjectPose.translation,new Rotation2d(0));
  //       Translation2d fieldToObjectPose=fieldToRobotPose.transformBy(robotToObjectTranform2d).getTranslation();
  //       if(inField(fieldToObjectPose)){
  //         fieldToObjectPoses.add(fieldToObjectPose);
  //       } 
  //     }
  //   }
  //   return fieldToObjectPoses;
  // }
  public List<Translation2d> rankPosesByDistance(List<Translation2d> poses, Translation2d robotPose) {
    poses.sort(Comparator.comparingDouble(pose -> pose.getDistance(robotPose)));
    return poses;
  }
   // Remove poses that are too close to another robot
   public List<Translation2d> filterClosePoses(List<Translation2d> poses, List<Translation2d> otherRobots,Translation2d currentPose, double minDistance) {
    List<Translation2d> filteredPoses = new ArrayList<>();
    for (Translation2d pose : poses) {
        boolean isTooClose = false;
        for (Translation2d robot : otherRobots) {
            if (pose.getDistance(robot) < Math.min(currentPose.getDistance(pose),minDistance)) {
                isTooClose = true;
                break;
            }
        }
        if (!isTooClose) {
            filteredPoses.add(pose);
        }
    }
    return filteredPoses;
  }

  public List<TrackingPose> getUpdatedPoses(List<Translation2d> newPoses,List<TrackingPose> trackedPoses, double currentTime) {
    // Update tracked poses with new measurements
    for (Translation2d newPose : newPoses) {
        boolean matched = false;
        for (TrackingPose trackedPose : trackedPoses) {
            if (trackedPose.translation.getDistance(newPose) < MERGE_DISTANCE_THRESHOLD) {
                // Apply Exponential Moving Average (EMA) for smoothing
                Translation2d smoothedTranslation = new Translation2d(
                        EMA_ALPHA * newPose.getX() + (1 - EMA_ALPHA) * trackedPose.translation.getX(),
                        EMA_ALPHA * newPose.getY() + (1 - EMA_ALPHA) * trackedPose.translation.getY()
                );
                trackedPose=new TrackingPose(smoothedTranslation, currentTime);
                matched = true;
                break;
            }
        }
        if (!matched) {
            trackedPoses.add(new TrackingPose(newPose, currentTime));
        }
      }
      // Remove poses that haven't been seen for a while
      Iterator<TrackingPose> iterator = trackedPoses.iterator();
      while (iterator.hasNext()) {
          TrackingPose trackedPose = iterator.next();
          if (currentTime - trackedPose.lastSeen > TIMEOUT) {
              iterator.remove();
          }
      }
      return trackedPoses;
    }

    public List<Translation2d> getStabilizedPoses(List<TrackingPose> trackedPoses) {
      List<Translation2d> stabilizedPoses = new ArrayList<>();
      for (TrackingPose trackedPose : trackedPoses) {
          stabilizedPoses.add(trackedPose.translation);
      }
      return stabilizedPoses;
    }
    

  public Translation2d getBestGamePiecePose() {
      return bestGamePiecePose;
  }
  public Translation2d getMostDangerousRobot() {
      return mostDangerousRobot;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    List<TimestampedObjectTranslation> othersRawPoses=m_visionIO.othersPoses;
    List<TimestampedObjectTranslation> gamePieceRawPoses=m_visionIO.notePoses;

    // othersPoses=getFieldToObjectPoses(othersRawPoses);
    // gamePiecePoses=getFieldToObjectPoses(gamePieceRawPoses);

    // trackedOthersPoses=getUpdatedPoses(othersPoses, trackedOthersPoses, Timer.getFPGATimestamp());
    // trackedGamePiecePoses=getUpdatedPoses(gamePiecePoses, trackedGamePiecePoses, Timer.getFPGATimestamp());

    // othersPoses=getStabilizedPoses(trackedOthersPoses);
    // gamePiecePoses=getStabilizedPoses(trackedGamePiecePoses);

    othersPoses=rankPosesByDistance(othersPoses, m_DriveSubsystem.getPose().getTranslation());

    gamePiecePoses=filterClosePoses(gamePiecePoses, othersPoses,m_DriveSubsystem.getPose().getTranslation(), 1.5);
    gamePiecePoses=rankPosesByDistance(gamePiecePoses, m_DriveSubsystem.getPose().getTranslation());
    
    
    if(gamePiecePoses.size()>=1){
      bestGamePiecePose=gamePiecePoses.get(0);
      gamePiecePose.setRobotPose(new Pose2d(bestGamePiecePose ,new Rotation2d(0)));
    }else{
      bestGamePiecePose=null;
    }
    if(othersPoses.size()>=1){
      mostDangerousRobot=othersPoses.get(0);
      enemyPose.setRobotPose(new Pose2d(mostDangerousRobot,new Rotation2d(0)));
    }else{
      mostDangerousRobot=null;
    }
    
    


  }
  public static record TimestampedObjectTranslation(
      double timestamp, Translation2d translation, double confidence) {}
  public static record TrackingPose(
      Translation2d translation, double lastSeen) {}
}
