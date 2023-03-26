// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private WPI_VictorSPX r1Motor = new WPI_VictorSPX(DriveConstants.kR1);
  private WPI_VictorSPX r2Motor = new WPI_VictorSPX(DriveConstants.kR2);
  public MotorControllerGroup rGroup  = new MotorControllerGroup(r1Motor, r2Motor);

  private WPI_VictorSPX l1Motor = new WPI_VictorSPX(DriveConstants.kL1);
  private WPI_VictorSPX l2Motor = new WPI_VictorSPX(DriveConstants.kL2);
  private MotorControllerGroup lGroup = new MotorControllerGroup(l1Motor, l2Motor);

  private Encoder ldriveEncoder = new Encoder(DriveConstants.lEncoder.port0, DriveConstants.lEncoder.port1);
  private Encoder rdriveEncoder = new Encoder(DriveConstants.rEncoder.port0, DriveConstants.rEncoder.port1);
  private EncoderSim ldriveEncoderSim = new EncoderSim(ldriveEncoder);
  private EncoderSim rdriveEncoderSim = new EncoderSim(rdriveEncoder);

  private DifferentialDrive haylazDrive;
  private DifferentialDriveOdometry odometry ;
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidthMeters);
  
  private DifferentialDrivetrainSim m_driveSim;

  private AHRS gyro = new AHRS(I2C.Port.kMXP);
  int dev = SimDeviceDataJNI.getSimDeviceHandle("gyro");
  SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,"Yaw"));

  private Field2d field ;
  private PIDController m_leftPIDController =  new PIDController(DriveConstants.pidLeft.kp, 0, DriveConstants.pidLeft.kd );
  private PIDController m_rightPIDController =  new PIDController(DriveConstants.pidRight.kp, 0, DriveConstants.pidRight.kd);


  private DifferentialDrivePoseEstimator haylazEstimator =  new DifferentialDrivePoseEstimator(
    m_kinematics,
    gyro.getRotation2d(),
    ldriveEncoder.getDistance(),
    rdriveEncoder.getDistance(),
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  public DriveSubsystem(){
    haylazDrive = new DifferentialDrive(rGroup, lGroup);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0,getStartLocation());
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    ldriveEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.kWheelRadius / DriveConstants.kEncoderResolution);
    rdriveEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.kWheelRadius / DriveConstants.kEncoderResolution);
    
    m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
    KitbotGearing.k10p71,        // 10.71:1
    KitbotWheelSize.kSixInch,    // 6" diameter wheels.
    null                         // No measurement noise.
  );
  }

  public void resetOdometry(){
    rdriveEncoder.reset();
    ldriveEncoder.reset();
    gyro.reset();    
}
 public void ramsete(){
 
 }
 private Pose2d getStartLocation()
 {
   int naber= DriverStation.getAlliance() == DriverStation.Alliance.Red ? DriverStation.getLocation() : DriverStation.getLocation() + 3;
   switch(naber)
   {
     case 1:
     return Constants.kRed._1;
     case 2:
     return Constants.kRed._2;
     case 3:
     return Constants.kRed._3;
     case 4:
     return Constants.kBlue._1;
     case 5:
     return Constants.kBlue._2;
     case 6:
      return Constants.kBlue._3;
     default:
       return new Pose2d();
   }
 }
 public void ArcadeDrive( double x, double z) {
  double leftOutput = (x-z)*m_leftPIDController.calculate(ldriveEncoder.getRate(), 40);
  double rightOutput = (x+z)*m_rightPIDController.calculate(rdriveEncoder.getRate(), 40);
  lGroup.setVoltage(leftOutput);
  rGroup.setVoltage(rightOutput);
}

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), ldriveEncoder.getDistance(), rdriveEncoder.getDistance() );
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("Robot x", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Robot y", odometry.getPoseMeters().getTranslation().getY());
    haylazEstimator.update(gyro.getRotation2d(), ldriveEncoder.getDistance(), rdriveEncoder.getDistance());
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_driveSim.update(0.02);
    m_driveSim.setInputs(
        lGroup.get() * RobotController.getBatteryVoltage(),
        rGroup.get() * RobotController.getBatteryVoltage());
    
    
    setSimDoubleFromDeviceData("navX-Sensor[0]", "Yaw", m_driveSim.getHeading().getDegrees());
		ldriveEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
		ldriveEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
		
		rdriveEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
		rdriveEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
  }

  public void setSimDoubleFromDeviceData(String deviceName, String keyName, double value) {
    SimDouble simDouble = new SimDouble(
      SimDeviceDataJNI.getSimValueHandle(
          SimDeviceDataJNI.getSimDeviceHandle(deviceName),
          keyName));
    simDouble.set(value);
  }
}
