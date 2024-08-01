// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class PoseEstimator {
    public SwerveDrivePoseEstimator sEstimator;
    public TimeInterpolatableBuffer<Double> turretYawBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    public TimeInterpolatableBuffer<Double> gyroYawBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    public NavigableMap<Double, Pose2d> robotPoseBuffer = new TreeMap<>();//TODO: limit size to save memory
    public Pose2d visionPose = new Pose2d();
    public PoseEstimator(){
        sEstimator = new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.swerveKinematics, 
            new Rotation2d(), 
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d(), 
            Constants.PoseEstimatorConstants.stateStdDevs, 
            Constants.PoseEstimatorConstants.visionStdDevs
        );
    }
    /** Check if this returns true before using {@link #updateVision()} 
     * @return If time buffers are !null */
    public boolean readyToUpdateVision(){
        return gyroYawBuffer.getSample(0).isPresent();
    }

    /** Update estimator with Swerve States and Gyro Yaw data.
     * Needs to be updated every loop. */
    public void updateSwerve(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions){
        sEstimator.update(gyroAngle, modulePositions);
        gyroYawBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle.getRadians());
        robotPoseBuffer.put(Timer.getFPGATimestamp(), getEstimatedPosition());
        SmartDashboard.putString("robotPoseBuffer", robotPoseBuffer.toString());
        while (robotPoseBuffer.firstKey()<Timer.getFPGATimestamp() -5) {
            robotPoseBuffer.pollFirstEntry();
        }
    }

    /** Update estimator with vision data. 
     *  Should only be updated when target is visible.
     * @param latency seconds */
    public void updateVision(Pose2d pose, double latency){
        double timeStamp = Timer.getFPGATimestamp() - latency;
        Rotation2d gyro = new Rotation2d(gyroYawBuffer.getSample(timeStamp).get());
        sEstimator.addVisionMeasurement(
            new Pose2d(pose.getX(), pose.getY(), gyro),
            timeStamp
        );
    }
    
    /** Update estimator with vision data. 
     *  Should only be updated when target is visible.
     * @param latency seconds */
    public void updateVision(Pose2d pose, double latency,double FOM){
        double timeStamp = Timer.getFPGATimestamp() - latency;
        Rotation2d gyro = new Rotation2d(gyroYawBuffer.getSample(timeStamp).get());
        sEstimator.addVisionMeasurement(
            new Pose2d(pose.getX(), pose.getY(), gyro),
            timeStamp,
            VecBuilder.fill(FOM, FOM, 0.1)
        );
        
    }

    public void updateVisionWithTime(Pose2d pose, double timeStamp){
        Rotation2d gyro = new Rotation2d(gyroYawBuffer.getSample(timeStamp).get());
        try{
            sEstimator.addVisionMeasurement(
                new Pose2d(pose.getX(), pose.getY(), gyro),
                timeStamp
            );
        }catch(ConcurrentModificationException e){

        }
    }

    public boolean poseIsValid(Pose3d visionPose,Rotation2d gyro, double gyroTolerance){
        if(Math.abs(visionPose.toPose2d().getRotation().getRadians()-gyro.getRadians())>gyroTolerance){
            return false;
        }
        return true;
    }
    // public void visionInput(List<TimestampedVisionUpdate> visionPoses,double gyroYawTolerance){
    //     for(TimestampedVisionUpdate visionPose : visionPoses){
    //         Pose2d estimatedPose=getRobotPoseWithTime(visionPose.timestamp);
    //         if (estimatedPose!=null){
    //             if(poseIsValid(visionPose.pose, estimatedPose.getRotation(), gyroYawTolerance)){
    //                 updateVisionWithTime(visionPose.pose.toPose2d(), visionPose.timestamp);
    //             }
    //         }
    //     }
    // }
    public Pose2d getEstimatedPosition(){
        return sEstimator.getEstimatedPosition();
    }
    // public Pose2d getRobotPoseWithTime(double timestamp){
    //     try{
    //         return robotPoseBuffer.floorEntry(timestamp).getValue();
    //     }catch(ConcurrentModificationException e){
    //         return null;
    //     }
    // }
    public void setPose(Rotation2d gyroYaw,SwerveModulePosition[] modulePositions,Pose2d pose) {
      sEstimator.resetPosition(gyroYaw, modulePositions, pose);
    }
    public static record TimestampedVisionUpdate(
      double timestamp, Pose3d pose, Matrix<N3, N1> stdDevs) {}

}
