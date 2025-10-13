// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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


  private Drive() {
    super("Drive");
  }

  public static Drive getInstance() {
    if (m_Drive == null) {
      m_Drive = new Drive();
    }
    return m_Drive;
  }



  @Override
  public void periodic() {
    System.out.println("Hi my name is not calvin :(");
    System.out.println("Cheese");
    // This method will be called once per scheduler run
    // Or will it? :(
  }
}
