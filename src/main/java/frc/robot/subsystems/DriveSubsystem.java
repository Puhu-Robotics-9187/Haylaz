// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.lEncoder;
import frc.robot.Constants.rEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private CANSparkMax r1Motor = new CANSparkMax(Constants.kR1, MotorType.kBrushed);
  private CANSparkMax r2Motor = new CANSparkMax(Constants.kR2, MotorType.kBrushed);
  private MotorControllerGroup rGroup = new MotorControllerGroup(r1Motor, r2Motor);

  private CANSparkMax l1Motor = new CANSparkMax(Constants.kL1, MotorType.kBrushed);
  private CANSparkMax l2Motor = new CANSparkMax(Constants.kL2, MotorType.kBrushed);
  private MotorControllerGroup lGroup= new MotorControllerGroup(l1Motor, l2Motor);

  private Encoder ldriveEncoder = new Encoder(Constants.lEncoder.port0, Constants.lEncoder.port1);
  private Encoder rdriveEncoder = new Encoder(Constants.rEncoder.port0, Constants.rEncoder.port1);
  private EncoderSim ldriveEncoderSim = new EncoderSim(ldriveEncoder);
  private EncoderSim rdriveEncoderSim = new EncoderSim(rdriveEncoder);

  private DifferentialDrive haylazDrive;
  private DifferentialDriveOdometry odometry ;
  private DifferentialDrivetrainSim m_driveSim;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = -4096;


  private AnalogGyro gyros = new AnalogGyro(0);
  private AnalogGyroSim gyroism = new AnalogGyroSim(gyros);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  int dev = SimDeviceDataJNI.getSimDeviceHandle("gyro");
  SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev,"Yaw"));

  private Field2d field ;

  public DriveSubsystem(){
    haylazDrive = new DifferentialDrive(rGroup, lGroup);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    lEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    rEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    
    m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
    KitbotGearing.k10p71,        // 10.71:1
    KitbotWheelSize.kSixInch,    // 6" diameter wheels.
    null                         // No measurement noise.
  );
  }

  public void ArcadeDrive(double x , double z){
    haylazDrive.arcadeDrive(x,z, true);
  }
  public void resetOdometry(){
    rdriveEncoder.reset();
    ldriveEncoder.reset();
    gyro.reset();    
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
        lGroup.get(),
        rGroup.get());
    m_driveSim.update(0.02);

    
    ldriveEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    ldriveEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    rdriveEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    rdriveEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    gyroism.setAngle(-m_driveSim.getHeading().getDegrees());
  }
    /** 
   * Allows for shorter more direct references of SimDoubles. Takes a SimDevice device name,
   * a key that points to a SimDouble, and a double to set the SimDouble to.
   * 
   * @param deviceName The name of the SimDevice
   * @param keyName The key that the double is associated with'
   * @param value Double that will be set to the SimDouble at the given key
   */
  public void setSimDoubleFromDeviceData(String deviceName, String keyName, double value) {
    SimDouble simDouble = new SimDouble(
      SimDeviceDataJNI.getSimValueHandle(
          SimDeviceDataJNI.getSimDeviceHandle(deviceName),
          keyName));
    simDouble.set(value);
  }
}
