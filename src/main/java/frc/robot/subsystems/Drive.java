// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  private static Drive m_Drive = null;
  

  private final MAXSwerveModule m_frontleft = new MAXSwerveModule(
    RobotMap.D_FRONT_LEFT_DRIVE_MOTOR_CAN_ID, RobotMap.D_FRONT_LEFT_TURN_MOTOR_CAN_ID, Constants.DriveModuleConstants.kFrontLeftChassisAngularOffset);
  private final MAXSwerveModule m_frontright = new MAXSwerveModule(
    RobotMap.D_FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, RobotMap.D_FRONT_RIGHT_TURN_MOTOR_CAN_ID, Constants.DriveModuleConstants.kFrontRightChassisAngularOffset);
  private final MAXSwerveModule m_backleft = new MAXSwerveModule(
    RobotMap.D_BACK_LEFT_DRIVE_MOTOR_CAN_ID, RobotMap.D_BACK_LEFT_TURN_MOTOR_CAN_ID, Constants.DriveModuleConstants.kBackLeftChassisAngularOffset);
  private final MAXSwerveModule m_backright = new MAXSwerveModule(
    RobotMap.D_BACK_RIGHT_DRIVE_MOTOR_CAN_ID, RobotMap.D_BACK_RIGHT_TURN_MOTOR_CAN_ID, Constants.DriveModuleConstants.kBackRightChassisAngularOffset);


  private final ADIS16470_IMU m_Gyro = new ADIS16470_IMU();  

  private Drive() {
    super("Drive");
  }

  public static Drive getInstance() {
    if (m_Drive == null) {
      m_Drive = new Drive();
    }
    return m_Drive;
  }

  public double getHeading() {
    return m_Gyro.getAngle(IMUAxis.kZ);
  }

  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontleft.setDesiredState(desiredStates[0]);
    m_frontright.setDesiredState(desiredStates[1]);
    m_backleft.setDesiredState(desiredStates[2]);
    m_backright.setDesiredState(desiredStates[3]);
  }

  public void drive(ChassisSpeeds speeds) {
    //ChassisSpeeds limitSpeeds = limitRates(speeds);
    var swerveModuleStates = Constants.DriveConstants.kKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedCommanded = xSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedCommanded = ySpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double rotCommanded = rot * Constants.DriveConstants.kMaxAngularSpeed;


    ChassisSpeeds chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedCommanded, ySpeedCommanded, rotCommanded, Rotation2d.fromDegrees(getHeading())) :
      new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, rotCommanded);
    drive(chassisSpeeds);
  }

  

  @Override
  public void periodic() {
    System.out.println("Hi my name is not calcium :(\nCheese");
    // This method will be called once per scheduler run
    // Or will it? :(
  }
}
