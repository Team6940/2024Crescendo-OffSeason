// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Library.team1706.LinearInterpolationTable;
import frc.robot.Library.team95.BetterSwerveKinematics;
import frc.robot.Subsystems.ImprovedXboxController.Button;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.Map;

import com.pathplanner.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants{
    public static final int ArmLeft_ID = 0;
    public static final int ArmRight_ID = 0;

    public static final double kP=0.;
    public static final double kI=0.;
    public static final double kD=0.;
    public static final double kS=0.;

    public static final double ArmVelocity=0.;
    public static final double ArmAcceleration=0.;

    public static final double ArmTolerence=0.;

    public static final double ArmDefaultDegree=0.;
    public static final double ArmAMPDegree=0.;
    public static final double ArmUpSPKDegree=0.;
    public static final double ArmDownSPKDegree=0.;
  }

  public static class IntakerConstants {
    public static final int Intaker_ID=0;
    public static final int Sensor_ID=0;

    public static final double kP=0.;
    public static final double kI=0.;
    public static final double kD=0.;

    public static final double NoteInOutput=0.;
    public static final double NoteOutOutput=0.;
    public static final double PassNoteOutput=0.;
  }

  public static class ShooterConstants {
    public static final int ShooterLeft_ID=0;
    public static final int ShooterRight_ID=0;

    public static final double kP=0.;
    public static final double kI=0.;
    public static final double kD=0.;
    public static final double kF=0.;
    public static final double kV=0.;

    public static final double ShooterDifferenceTolerence=0.;
    public static final double ShooterSpeedTolerence=0.;

    public static final double ShooterAMPRPS=0.;
    public static final double ShooterManualSPKRPS=0.;
  }

  public static class BlockerConstants {
    public static final int Blocker_ID=0;
    public static final int Sensor_ID=0;

    public static final double PassNoteOutput=0.;
    public static final double GiveNoteOutput=0.;
    public static final double AMPOutput=0.;
  }

  public static class GlobalConstants{
    public final static float INF = (float)Math.pow(10, 6);   //this was defined for the 1690 lib
  }

  public static class DriveConstants{
    public final static double kInnerDeadband = 0.10;
    public final static double kOuterDeadband = 0.98;           //these were defined for the 1706 lib;
  }

  public static class SwerveConstants{
    public final static double MaxSpeed = 2.;     //TODO
  }

  public static class PathPlannerConstants{
    public final static PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0, 0);    //TODO
    public final static PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0, 0);       //TODO
  }

  public static class LimelightConstants{
    public final static String SPKR_LLname = "SPKRLimelight";
  }
}
