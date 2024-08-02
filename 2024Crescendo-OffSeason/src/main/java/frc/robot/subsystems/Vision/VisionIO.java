// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;


import java.nio.file.FileSystem;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.Vision.objectTracker.TimestampedObjectTranslation; 

class VisionMeasurement{
  Pose3d pose;
  double error;
  double latency;
  VisionMeasurement(Pose3d pose, double error, double latency){
    this.pose=pose;
    this.error=error;
    this.latency=latency;
  }
}
public class VisionIO extends SubsystemBase {
  /** Creates a new VisionIO. */
  public List<TimestampedVisionUpdate> robotPoses = new ArrayList<>();
  public List<TimestampedObjectTranslation> notePoses = new ArrayList<>();
  public List<TimestampedObjectTranslation> othersPoses = new ArrayList<>();

  public List<NetworkTable> cameraTables = new ArrayList<>();
  public NetworkTable notePoseTable;
  public NetworkTable othersPoseTable;

  public List<NetworkTableEntry> multiTagRobotPoseEntries = new ArrayList<>();
  public List<List<NetworkTableEntry>> robotPosesEntries = new ArrayList<>();

  public NetworkTableEntry notePoseEntries;
  public NetworkTableEntry othersPoseEntries;


  private Field2d visionPose = new Field2d();

  public VisionIO() {
    SmartDashboard.putString("config",Constants.BlackholeVisionConstants.config);
    
    NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    networkTableInstance.startServer();
    for(String tableName : Constants.BlackholeVisionConstants.tableNames){
      NetworkTable table=networkTableInstance.getTable(tableName);
      cameraTables.add(table);
      multiTagRobotPoseEntries.add(table.getEntry("multiTagRobotPose"));
    }
    notePoseTable = networkTableInstance.getTable("note");
    othersPoseTable = networkTableInstance.getTable("robot");

    notePoseEntries = notePoseTable.getEntry("poses");
    othersPoseEntries = othersPoseTable.getEntry("poses");

    SmartDashboard.putData("VisionField",visionPose); 
  }
  
  public void clearMeasurement(){
    robotPoses.clear();
    notePoses.clear();
    othersPoses.clear();
  }

  public void addMultiTagPoses(){
    Pose3d multiTagRobotPose;
    double error;
    double latency;
    double[] poseArray;
    for(NetworkTableEntry entry : multiTagRobotPoseEntries){
      poseArray=entry.getDoubleArray(new double[]{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
      if(poseArray.length==8){
        multiTagRobotPose= new Pose3d(poseArray[0],poseArray[1],poseArray[2],new Rotation3d(poseArray[3],poseArray[4],poseArray[5]));
        error=poseArray[6];
        latency=poseArray[7];
        robotPoses.add(new TimestampedVisionUpdate(Timer.getFPGATimestamp()-latency, multiTagRobotPose, VecBuilder.fill(error/500, error/500, error/500)));//to do
      }
    } 
  }

  public double[][] separateObjectPoses(double[] oneDPoseArray){
    int length=oneDPoseArray.length;
    int height=(int)(Math.ceil((double)oneDPoseArray.length/6));
    double[][] separatePosesArray= new double[height][6];
    for(int i=0;i<length;i++){
      separatePosesArray[i/6][i%6]=oneDPoseArray[i];
    }
    
    return separatePosesArray;

  }
  public void addNotePoses(){
    double[] notePosesArray=notePoseEntries.getDoubleArray(new double[]{0,0,0,0,0,0});
    double[][] separatedNotePosesArray=separateObjectPoses(notePosesArray);
    for(double[] notePoseArray : separatedNotePosesArray){
      Translation2d pose=new Translation2d(notePoseArray[0],notePoseArray[1]);
      pose=pose.times(1+(0.18/pose.getNorm()));
      notePoses.add(new TimestampedObjectTranslation(
        Timer.getFPGATimestamp()-notePoseArray[5], pose,notePoseArray[4]) );
    }
  }
  public void addOthersPoses(){
    double[] othersPosesArray=othersPoseEntries.getDoubleArray(new double[]{0,0,0,0,0,0});
    double[][] separatedOthersPosesArray=separateObjectPoses(othersPosesArray);
    for(double[] othersPoseArray : separatedOthersPosesArray){
      Translation2d pose=new Translation2d(othersPoseArray[0],othersPoseArray[1]);
      pose=pose.times(1+(0.5/pose.getNorm()));
      othersPoses.add(new TimestampedObjectTranslation(
        Timer.getFPGATimestamp()-othersPoseArray[5], pose,othersPoseArray[4]) );
    }
  }
  public List<TimestampedVisionUpdate> getRobotPose3ds(){
    return robotPoses;
  }
  public void sendTrainInputs(double currentTime,Pose2d robotPose,ChassisSpeeds robotSpeeds,Translation2d enemyPoseRelativeToRobot,boolean hasNote){
    if(DriverStation.isTeleop()&&DriverStation.isEnabled()){
      SmartDashboard.putNumberArray("measurements", new double[]{
        currentTime,
        robotPose.getX(),
        robotPose.getY(),
        robotPose.getRotation().getRadians(),
        robotSpeeds.vxMetersPerSecond,
        robotSpeeds.vyMetersPerSecond,
        robotSpeeds.omegaRadiansPerSecond,
        enemyPoseRelativeToRobot.getX(),
        enemyPoseRelativeToRobot.getY(),
        (hasNote?1:0)
      });
    }else{
      SmartDashboard.putNumberArray("measurements", new double[]{});
    }
  }
  public void sendActions(double currentTime,Translation2d desireVelocity,double rotation){
    if(DriverStation.isTeleop()&&DriverStation.isEnabled()){
      SmartDashboard.putNumberArray("actions", new double[]{currentTime,desireVelocity.getX(),desireVelocity.getY(),rotation});
    }else{
      SmartDashboard.putNumberArray("actions", new double[]{});
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    clearMeasurement();
    addMultiTagPoses();
    addNotePoses();
    addOthersPoses();
    if(robotPoses.size()>0){
      visionPose.setRobotPose(robotPoses.get(0).pose().toPose2d());
    }
    
  }
  
}
