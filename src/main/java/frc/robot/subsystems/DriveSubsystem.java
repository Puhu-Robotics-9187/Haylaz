// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
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
  private WPI_VictorSPX r1Motor = new WPI_VictorSPX(Constants.kR1);
  private WPI_VictorSPX r2Motor = new WPI_VictorSPX(Constants.kR2);
  public MotorControllerGroup rGroup  = new MotorControllerGroup(r1Motor, r2Motor);

  private WPI_VictorSPX l1Motor = new WPI_VictorSPX(Constants.kL1);
  private WPI_VictorSPX l2Motor = new WPI_VictorSPX(Constants.kL2);
  private MotorControllerGroup lGroup = new MotorControllerGroup(l1Motor, l2Motor);

  private Encoder ldriveEncoder = new Encoder(Constants.lEncoder.port0, Constants.lEncoder.port1);
  private Encoder rdriveEncoder = new Encoder(Constants.rEncoder.port0, Constants.rEncoder.port1);
  private EncoderSim ldriveEncoderSim = new EncoderSim(ldriveEncoder);
  private EncoderSim rdriveEncoderSim = new EncoderSim(rdriveEncoder);

  private DifferentialDrive haylazDrive;
  private DifferentialDriveOdometry odometry ;
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.482);
  
  private DifferentialDrivetrainSim m_driveSim;

  private AHRS gyro = new AHRS(I2C.Port.kMXP);
  int dev = SimDeviceDataJNI.getSimDeviceHandle("gyro");
  SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,"Yaw"));

  private Field2d field ;
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
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    ldriveEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);
    rdriveEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);
    
    m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
    KitbotGearing.k10p71,        // 10.71:1
    KitbotWheelSize.kSixInch,    // 6" diameter wheels.
    null                         // No measurement noise.
  );
  }

  public void ArcadeDrive(double x , double z){
    haylazDrive.curvatureDrive(x,z, true);
  }
  
  public void resetOdometry(Pose2d pose){
    rdriveEncoder.reset();
    ldriveEncoder.reset();
    gyro.reset();    
}
 public void ramsete(){
 
 }
 public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
  final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
  final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

  final double leftOutput =
          m_leftPIDController.calculate(ldriveEncoder.getRate(), speeds.leftMetersPerSecond);
  final double rightOutput =
          m_rightPIDController.calculate(rdriveEncoder.getRate(), speeds.rightMetersPerSecond);
  lGroup.setVoltage(leftOutput + leftFeedforward);
  rGroup.setVoltage(rightOutput + rightFeedforward);
}

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), ldriveEncoder.getDistance(), rdriveEncoder.getDistance() );
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("Robot x", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Robot y", odometry.getPoseMeters().getTranslation().getY());
    
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_driveSim.setInputs(
        lGroup.get() * RobotController.getBatteryVoltage(),
        rGroup.get() * RobotController.getBatteryVoltage());
    m_driveSim.update(0.02);

    
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
