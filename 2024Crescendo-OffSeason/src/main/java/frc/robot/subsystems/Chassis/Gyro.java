// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.Chassis;

// import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Gyro extends SubsystemBase {
//   /** Creates a new Gyro. */
//   public  Pigeon2 gyro;
//   public Gyro() {
//         gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
//         gyro.getConfigurator().apply(new Pigeon2Configuration());
//   }
//   public Rotation2d getGyroYaw() {
//         return Rotation2d.fromDegrees(gyro.getYaw().getValue());
//   }
//   public void zeroHeading(){
//     gyro.setYaw(0);
// }
//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
