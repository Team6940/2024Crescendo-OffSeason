// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Library.team1706.LinearInterpolationTable;
import frc.robot.Library.team8814.util.COTSTalonFXSwerveConstants;
import frc.robot.Library.team8814.util.SwerveModuleConstants;
import frc.robot.Library.team95.BetterSwerveKinematics;
import frc.robot.subsystems.ImprovedXboxController.Button;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.PIDConstants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants {
    public static final int ArmLeft_ID = 14;
    public static final int ArmRight_ID = 15;

    public static final double ArmGearRatio = 200.;

    public static final double kP = 17.;
    public static final double kI = 0.;
    public static final double kD = 0.;
    public static final double kS = 0.;
    public static final double kG = 0.;

    public static final double ArmVelocity = 1.;
    public static final double ArmAcceleration = 4;

    public static final double ArmTolerence = 1;

    public static final double ArmDefaultDegree = -72.29;
    public static final double ArmAMPDegree = 46.;
    public static final double ArmUpSPKDegree = 50;
    public static final double ArmDownSPKDegree = 0.;
  }

  public static class IntakerConstants {
    public static final int Intaker_ID = 20;

    public static final double kP = 0.;
    public static final double kI = 0.;
    public static final double kD = 0.;

    public static final double NoteInOutput = 1.;
    public static final double NoteOutOutput = -1.;

    public static final int Sensor_ID = 0;
  }

  public static class ShooterConstants {
    public static final int ShooterLeft_ID=0;
    public static final int ShooterRight_ID=23;

    public static final double kP = 0.1;
    public static final double kI = 0.;
    public static final double kD = 0.012;
    public static final double kS = 0.25;
    public static final double kV = 0.11458;

    public static final double ShooterDifferenceTolerence = 0.5;
    public static final double ShooterSpeedTolerence = 1.;

    public static final double ShooterAMPRPS = 0.;
    public static final double ShooterManualSPKRPS = 0.;
  }

  public static class BlockerConstants {
    public static final int Blocker_ID = 30;
    public static final int Sensor_ID = 2;

    public static final double NoteOutOutput = -1.;//传递球到shooter前
    public static final double NoteInOutput = 1.;//传递球到shooter前
    public static final double GiveNoteOutput = 1.;//shooter加速完毕后给球
    public static final double AMPOutput = -1.;
  }

  public static class AutoShootConstants{
    public static final double kP=0.2;
    public static final double kI=0.;
    public static final double kD=0.0;
    public static final double DegreeTolerance=2.;
    public static final double VelocityTolerance=4.;
    public static final double RPSInAdvance=30;
    public static final double HightDifference=1.;//单位暂定meter

    public static final Point2D[] ArmPoints={
      new Point2D.Double(18.04,-33),
      new Point2D.Double(13.31,-39),
      new Point2D.Double(7.15,-50),
      new Point2D.Double(2.86,-56),
      new Point2D.Double(-2.31,-62),
      new Point2D.Double(-6.91,-70)
    };
    public static final Point2D[] RPSPoints={
      new Point2D.Double(18.04,40),
      
      new Point2D.Double(-2.34,50),
      
      new Point2D.Double(-6.89,60)
    };

    public static final Point2D[] DisToArmPoints = {
      new Point2D.Double(1.66,-45),
      new Point2D.Double(2.08,-52),
      new Point2D.Double(2.54,-56),
      new Point2D.Double(3.08,-63),
      
      new Point2D.Double(3.5,-65),
      new Point2D.Double(4.07,-70),
      new Point2D.Double(5.16,-72)
    };

    public static final Point2D[] DisToRPSPoints = {
      new Point2D.Double(1.66,40),
      
      new Point2D.Double(3.07,50),
      
      new Point2D.Double(5.16,60)
    };

    public static final InterpolatingDoubleTreeMap ArmTable=new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap RPSTable=new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DisToArmTable = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DisToRPSTable = new InterpolatingDoubleTreeMap();
    public static final double MaxRange = 5.;
    public static final double CoastVelocity = 2.;
  }

  public static class GlobalConstants {
    public final static float INF = (float) Math.pow(10, 6); // this was defined for the 1690 lib
  }

  public static final class FieldConstants{
      public static final class SPKTranslation{
        public static final Translation2d Blue = new Translation2d(0, 0);
        public static final Translation2d Red = new Translation2d(16.43, 5.50);
    }
  }

  public static class DriveConstants {
    public final static double kInnerDeadband = 0.004;
    public final static double kOuterDeadband = 0.98; // these were defined for the 1706 lib;
    public static final int IntakeButton = 0;
  }
  public static class AutoAMPConstants{
    public final static double AutoAMPRotationkP=0.15;
    public final static double AutoAMPRotationkI=0.;
    public final static double AutoAMPRotationkD=0.;
    public final static double AutoAMPDegreeTolerance=3;
    public final static double AutoAMPDegreeOmegaTolerance=0.1;
    public final static double AutoAMPTranslationkP=3;
    public final static double AutoAMPTranslationkI=0.;
    public final static double AutoAMPTranslationkD=0.;
    public final static double AutoAMPTranslationTolerance=0.2;
    public final static double AutoAMPArmThreshold=10;
      public final static double AMPDistancetoGo=0.4;
    public final static class AMPTagID{   //TODO
      public final static int Blue = 0;
      public final static int Red = 0;
    }
    public final static class AutoAMPPose{    //TODO
      public final static Pose2d Blue = new Pose2d(0, 0, new Rotation2d(0));
      public final static Pose2d Red = new Pose2d(14.7,7.0, new Rotation2d(-Math.PI/2));
    }
  }

  public static final class SwerveConstants { // From 8814
    public static final int pigeonID = 13;

    public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.WCP.SwerveXFlipped
        .KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X3_10);

    /* Drivetrain Constants */
    public static final double trackWidth = 0.6195;//TODO
    public static final double wheelBase = 0.6195;
    public static final double wheelCircumference = 0.1016 * Math.PI;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 50;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKV = 3;

    /* Drive Motor PID Values */
    public static final double driveKP = 2.0; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0;// 0.0;
    public static final double driveKF = 0.0;// 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.13; // TODO: This must be tuned to specific robot
    public static final double driveKV = 3
    ;
    public static final double driveKA = 0;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.; // TODO: This must be tuned to specific robot
    public static final double maxModuleSpeed = 7 ; // TODO: This must be tuned to specific robot

    public static final double maxAcceleration = 5;
    public static final double maxDeceleration = 30;

    public static final double loopDuration = 0.02; // in second

    /** Radians per Second */
    public static final double stationaryAngularVelocity = 1.5 * Math.PI * 2; // TODO: This must be tuned to specific
                                                                              // robot
    public static final double maxRadius = 0.4;
    public static final double maxAngularAcceleration = 20;
    public static final double maxAngularDeceleration = 30;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 9;
      // the bigger the offset, the smaller the setPosition, thus the relative CW
      // point of the start angle
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-87.60);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 12;
      // the bigger the offset, the smaller the setPosition, thus the relative CW
      // point of the start angle
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(63.45703125);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 6;
      // the bigger the offset, the smaller the setPosition, thus the relative CW
      // point of the start angle
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(143.3828125);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 3;
      // the bigger the offset, the smaller the setPosition, thus the relative CW
      // point of the start angle
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(108.28125);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
    public static final double OdometryPeriod = 0.01;
  }

  public static final class BlackholeVisionConstants {
    public static final String[] tableNames = { "testCamera1", "testCamera" };
    public static final String config = "{\"time\":10,\"cameras\": [" +
        "{\"name\": \"testCamera\",\"ID\": \"/dev/camera_bottom_left\",\"pose\":{\"x\":0,\"y\":0,\"z\":1,\"roll\":0,\"pitch\":0,\"yaw\":0},\"cameraMatrix\": [[906.11130401,0,691.15552533],[0,906.35171032,371.55847183],[0,0,1]],\"distortionCoeffs\": [0.02281145,-0.01416523,0.00050521,0.0011957,-0.02318725]},"
        +
        "{\"name\": \"testCamera1\",\"ID\": \"/dev/camera_bottom_right\",\"pose\":{\"x\":0,\"y\":0,\"z\":1,\"roll\":0,\"pitch\":0,\"yaw\":0},\"cameraMatrix\": [[906.11130401,0,691.15552533],[0,906.35171032,371.55847183],[0,0,1]],\"distortionCoeffs\": [0.02281145,-0.01416523,0.00050521,0.0011957,-0.02318725]}"
        +
        "],\"capture\": {\"resolution\": {\"width\": 1280,\"height\":800},\"FPS\": 100,\"autoExposure\": 0,\"exposure\": 15,\"gain\": 30,\"brightness\": 30,\"contrast\": 100}"
        +
        ",\"apriltagDetector\": {\"resolution\": {\"width\": 1600,\"height\":1200},\"aruco\": \"DICT_APRILTAG_36H11\",\"rejectingIds\":[]}"
        +
        ",\"tagPoseEstimator\": {\"tagSize\":0.165}" +
        ",\"Calibration\":{\"charuco_dict\": \"DICT_6X6_250\",\"charuco_board\": {\"squares_x\": 5,\"squares_y\": 7,\"square_length\": 0.037,\"marker_length\": 0.022},\"image_path\": \"/home/jetson/Desktop/BlackHoleVision/BlackHoleVision1.0(5,19,2024)/calibration/calibration_videos/video_*.jpg\"}"
        +
        ",\"objectDetection\":{\"modelName\":\"last.pt\" },\"output\":{\"networkTable\":{\"teamNumber\":1001}}}\r\n";

    public static final double confiTolerance = 0.3;
  }

  public static final class PoseEstimatorConstants {
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.7, 0.7, 0.1);
    public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final InterpolatingDoubleTreeMap tAtoDev=new InterpolatingDoubleTreeMap();
    
  }

  public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
                                            // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPTranslationController = 2;
    public static final double kPRotationController = 2;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  public static class AutoAlignConstants{
    public static final double kAutoAlignRotationP=1;
    public static final double kAutoAlignRotationI=0;
    public static final double kAutoAlignRotationD=0;
    
    public static final double kAutoAlignMovementP=0.07;
    public static final double kAutoAlignMovementI=0;
    public static final double kAutoAlignMovementD=0;
    public static final double kAutoAlignTolerance=2;
    public static final double kAutoAlignPeak=0.5;
  }
  public static class PathPlannerConstants {
    public final static PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0, 0); // TODO
    public final static PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0, 0); // TODO
  }

  public static class LimelightConstants {
    public final static String SPKR_LLname = "limelight-spkr";
    public final static String AUTP_LLname = "limelight-pick";

    public final static double LLAngle = 0.;//ll视线相对水平面的仰角
  }

  public static class ManualShootConstants{
    public final static double ChassisSpeedTolerance = 0.5;
  }
}
