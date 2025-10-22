// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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


  ChassisSpeeds m_requestSpeeds = new ChassisSpeeds();
  ChassisSpeeds m_limitSpeeds = new ChassisSpeeds();
  double m_driveTime = 0;


  private TDNumber td_xInput;
  private TDNumber td_yInput;
  private TDNumber td_rotInput;
  private TDNumber td_xMeasured;
  private TDNumber td_yMeasured;
  private TDNumber td_rotMeasured;
  private TDNumber td_frontLeftSpeed;
  private TDNumber td_frontLeftAngle;
  private TDNumber td_frontRightSpeed;
  private TDNumber td_frontRightAngle;
  private TDNumber td_backLeftSpeed;
  private TDNumber td_backLeftAngle;
  private TDNumber td_backRightSpeed;
  private TDNumber td_backRightAngle;
  private TDNumber td_poseX;
  private TDNumber td_poseY;
  private TDNumber td_poseRot;
  private TDNumber td_frontLeftDriveCurrent;
  private TDNumber td_frontLeftTurnCurrent;
  private TDNumber td_frontRightDriveCurrent;
  private TDNumber td_frontRightTurnCurrent;
  private TDNumber td_backLeftDriveCurrent;
  private TDNumber td_backLeftTurnCurrent;
  private TDNumber td_backRightDriveCurrent;
  private TDNumber td_backRightTurnCurrent;
 
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
      
      td_xInput = new TDNumber(this, "Drive Input", "X Input Speed");
      td_yInput = new TDNumber(this, "Drive Input", "Y Input Speed");
      td_rotInput = new TDNumber(this, "Drive Input", "Rotation Input");
      td_xMeasured = new TDNumber(this, "Measured Speeds", "X Measured Speed");
      td_yMeasured = new TDNumber(this, "Measured Speeds", "Y Measured Speed");
      td_rotMeasured = new TDNumber(this, "Measured Speeds", "Rotation Measured");
      td_frontLeftSpeed = new TDNumber(this, "Module Speeds", "Front Left Speed");
      td_frontLeftAngle = new TDNumber(this, "Module Speeds", "Front Left Angle");
      td_frontRightSpeed = new TDNumber(this, "Module Speeds", "Front Right Speed");
      td_frontRightAngle = new TDNumber(this, "Module Speeds", "Front Right Angle");
      td_backLeftSpeed = new TDNumber(this, "Module Speeds", "Back Left Speed");
      td_backLeftAngle = new TDNumber(this, "Module Speeds", "Back Left Angle");
      td_backRightSpeed = new TDNumber(this, "Module Speeds", "Back Right Speed");
      td_backRightAngle = new TDNumber(this, "Module Speeds", "Back Right Angle");

      td_poseX = new TDNumber(this, "Robot Pose", "X");
      td_poseY = new TDNumber(this, "Robot Pose", "Y");
      td_poseRot = new TDNumber(this, "Robot Pose", "Rotation");

      td_frontLeftDriveCurrent = new TDNumber(this, "Motor Electrical Currents","Front Left Drive Current");
      td_frontLeftTurnCurrent = new TDNumber(this, "Motor Electrical Currents","Front Left Turning Current");
      td_frontRightDriveCurrent = new TDNumber(this, "Motor Electrical Currents","Front Right Drive Current");
      td_frontRightTurnCurrent = new TDNumber(this, "Motor Electrical Currents","Front Right Turning Current");
      td_backLeftDriveCurrent = new TDNumber(this, "Motor Electrical Currents","Back Left Drive Current");
      td_backLeftTurnCurrent = new TDNumber(this, "Motor Electrical Currents","Back Left Turning Current");
      td_backRightDriveCurrent = new TDNumber(this, "Motor Electrical Currents","Back Right Drive Current");
      td_backRightTurnCurrent = new TDNumber(this, "Motor Electrical Currents","Back Right Turning Current");

      DCMotor neovortex = DCMotor.getNEO(1).withReduction(Constants.DriveModuleConstants.kDrivingMotorReduction);

      ModuleConfig kSwerveModuleConfig = new ModuleConfig(Constants.DriveModuleConstants.kWheelDiameterMeters/2, Constants.DriveConstants.kMaxSpeedMetersPerSecond, 
      Constants.AutoConstants.kPathFollowerWheelCoeficientFriction, neovortex, Constants.DriveModuleConstants.kDrivingMotorCurrentLimit, 4);
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getMeasuredSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController(Constants.AutoConstants.kPathFollowerTranslationPID, Constants.AutoConstants.kPathFollowerRotationPID),
        new RobotConfig(Constants.AutoConstants.kPathFollowerMass, Constants.AutoConstants.kPathFollowerMomentOfInertia, kSwerveModuleConfig, Constants.DriveModuleConstants.m_ModulePositions),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
      PathPlannerLogging.setLogCurrentPoseCallback((pose)->{
        m_Field.getObject("AutoCurrent").setPose(pose);
      });
      PathPlannerLogging.setLogTargetPoseCallback((pose)->{
        m_Field.getObject("AutoTarget").setPose(pose);
      });
      PathPlannerLogging.setLogActivePathCallback((poses)->{
        m_Field.getObject("Auto Active Path").setPoses(poses);
      });
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

    td_xInput.set(xSpeedCommanded);
    td_yInput.set(ySpeedCommanded);
    td_rotInput.set(rotCommanded);

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

    ChassisSpeeds measuredSpeeds = getMeasuredSpeeds();
    td_xMeasured.set(measuredSpeeds.vxMetersPerSecond);
    td_yMeasured.set(measuredSpeeds.vyMetersPerSecond);
    td_rotMeasured.set(measuredSpeeds.omegaRadiansPerSecond);

    SwerveModuleState frontLeft = m_frontleft.getState();
    SwerveModuleState frontRight = m_frontright.getState();
    SwerveModuleState backLeft = m_backleft.getState();
    SwerveModuleState backRight = m_backright.getState();
    
    td_frontLeftSpeed.set(frontLeft.speedMetersPerSecond);
    td_frontLeftAngle.set(frontLeft.angle.getDegrees());
    td_frontRightSpeed.set(frontRight.speedMetersPerSecond);
    td_frontRightAngle.set(frontRight.angle.getDegrees());
    td_backLeftSpeed.set(backLeft.speedMetersPerSecond);
    td_backLeftAngle.set(backLeft.angle.getDegrees());
    td_backRightSpeed.set(backRight.speedMetersPerSecond);
    td_backRightAngle.set(backRight.angle.getDegrees());

    td_poseX.set(getPose().getX());
    td_poseY.set(getPose().getY());
    td_poseRot.set(getHeading());

    td_frontLeftDriveCurrent.set(m_frontleft.getDriveOutputCurrent());
    td_frontLeftTurnCurrent.set(m_frontleft.getTurningOutputCurrent());
    td_frontRightDriveCurrent.set(m_frontright.getDriveOutputCurrent());
    td_frontRightTurnCurrent.set(m_frontright.getTurningOutputCurrent());
    td_backLeftDriveCurrent.set(m_backleft.getDriveOutputCurrent());
    td_backLeftTurnCurrent.set(m_backleft.getTurningOutputCurrent());
    td_backRightDriveCurrent.set(m_backright.getDriveOutputCurrent());
    td_backRightTurnCurrent.set(m_backright.getTurningOutputCurrent());
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
