// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;

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
  private final Field2d m_Field;

  //these arent used but they seem good to have
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private ChassisSpeeds m_lastSpeeds = new ChassisSpeeds();

  //SwerveCrivePoseEstimator class for tracking robot pose
  SwerveDrivePoseEstimator m_DrivePoseEstimator;

  //testing dashboard instances
  TDNumber TDxSpeedCommanded;
  TDNumber TDySpeedCommanded;
  TDNumber TDrotSpeedCommanded;

  TDNumber TDFrontLeftDriveSpeed;
  TDNumber TDFrontLeftAngle;
  TDNumber TDFrontRightDriveSpeed;
  TDNumber TDFrontRightAngle;
  TDNumber TDBackLeftDriveSpeed;
  TDNumber TDBackLeftAngle;
  TDNumber TDBackRightDriveSpeed;
  TDNumber TDBackRightAngle;

  TDNumber TDxSpeedMeasured;
  TDNumber TDySpeedMeasured;
  TDNumber TDrotSpeedMeasured;

  TDNumber TDPoseX;
  TDNumber TDPoseY;
  TDNumber TDPoseAngle;
  TDNumber TDGyroAngle;

  ChassisSpeeds m_requestSpeeds = new ChassisSpeeds();
  ChassisSpeeds m_limitSpeeds = new ChassisSpeeds();
  double m_driveTime = 0;

  TDNumber m_DLeftFrontCurrentOutput;
  TDNumber m_DRightFrontCurrentOutput;
  TDNumber m_DLeftBackCurrentOutput;
  TDNumber m_DRightBackCurrentOutput;

  TDNumber m_TLeftFrontCurrentOutput;
  TDNumber m_TRightFrontCurrentOutput;
  TDNumber m_TLeftBackCurrentOutput;
  TDNumber m_TRightBackCurrentOutput;


  private Drive() {
    super("Drive");

    m_DrivePoseEstimator = new SwerveDrivePoseEstimator(
      Constants.DriveConstants.kKinematics,
      Rotation2d.fromDegrees(m_Gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
        m_frontleft.getPosition(),
        m_frontright.getPosition(),
        m_backleft.getPosition(),
        m_backright.getPosition()
      },
      new Pose2d());
      m_Field = new Field2d();
      new TDSendable(this,"Field", "Position", m_Field);
  }

  public static Drive getInstance() {
    if (m_Drive == null) {
      m_Drive = new Drive();
    }
    return m_Drive;
  }

  public double getHeading() {
    return getPose().getRotation().getDegrees();
  }

  public Pose2d getPose() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_DrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(m_Gyro.getAngle(IMUAxis.kZ)), 
      new SwerveModulePosition[] {
        m_frontleft.getPosition(),
        m_frontright.getPosition(),
        m_backleft.getPosition(),
        m_backright.getPosition()
      },
      pose);
  }

public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
  m_DrivePoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
}

  public ChassisSpeeds getMeasuredSpeeds() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    moduleStates[0] = m_frontleft.getState();
    moduleStates[1] = m_frontright.getState();
    moduleStates[2] = m_backleft.getState();
    moduleStates[3] = m_backright.getState();

    return Constants.DriveConstants.kKinematics.toChassisSpeeds(moduleStates);
  }

  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontleft.setDesiredState(desiredStates[0]);
    m_frontright.setDesiredState(desiredStates[1]);
    m_backleft.setDesiredState(desiredStates[2]);
    m_backright.setDesiredState(desiredStates[3]);
  }

  public void drive(ChassisSpeeds speeds) {
    ChassisSpeeds limitSpeeds = limitRates(speeds);
    var swerveModuleStates = Constants.DriveConstants.kKinematics.toSwerveModuleStates(limitSpeeds);
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

  public ChassisSpeeds limitRates(ChassisSpeeds commandedSpeeds) {
    //Apply Speed Limits
    double linearSpeed = Math.hypot(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
    Rotation2d direction = Rotation2d.kZero;
    if(Math.abs(commandedSpeeds.vxMetersPerSecond) > 1e-6 ||
       Math.abs(commandedSpeeds.vyMetersPerSecond) > 1e-6){
      direction = new Rotation2d(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
    }
    double limitedSpeed = Math.min(linearSpeed, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    double limitedTheta = MathUtil.clamp(commandedSpeeds.omegaRadiansPerSecond, 
                                            -Constants.DriveConstants.kMaxSpeedMetersPerSecond, 
                                            Constants.DriveConstants.kMaxAngularSpeed);
    
    Translation2d linearVelocity = (new Pose2d(new Translation2d(), direction)
                                      .transformBy(new Transform2d(new Translation2d(limitedSpeed, 0.0), new Rotation2d()))).getTranslation();
                        
    //Apply Acceleration Limits
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double accelerationDif = Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared * (currentTime-m_prevTime);
    double xSpeed = MathUtil.clamp(linearVelocity.getX(),
                                  m_lastSpeeds.vxMetersPerSecond - accelerationDif,
                                  m_lastSpeeds.vxMetersPerSecond + accelerationDif);

    double ySpeed = MathUtil.clamp(linearVelocity.getY(),
                                 m_lastSpeeds.vyMetersPerSecond - accelerationDif,
                                 m_lastSpeeds.vyMetersPerSecond + accelerationDif);

    double thetaAccelDif = Constants.DriveConstants.kMaxAngularAcceleration * (currentTime - m_prevTime);
    double thetaSpeed = MathUtil.clamp(limitedTheta,
                                 m_lastSpeeds.omegaRadiansPerSecond - thetaAccelDif,
                                 m_lastSpeeds.omegaRadiansPerSecond + thetaAccelDif);

   ChassisSpeeds limitedSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
   m_lastSpeeds = limitedSpeeds;
   m_prevTime = currentTime;
   return limitedSpeeds;
  }

  private void updateTD() {
    m_Field.setRobotPose(getPose());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Or will it?
    m_DrivePoseEstimator.update(Rotation2d.fromDegrees(m_Gyro.getAngle(IMUAxis.kZ)), new SwerveModulePosition[] {
      m_frontleft.getPosition(),
      m_frontright.getPosition(),
      m_backleft.getPosition(),
      m_backright.getPosition()
    });
    updateTD();
    super.periodic();
  }
}
