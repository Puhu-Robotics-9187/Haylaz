// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public CANSparkMax r1Motor = new CANSparkMax(Constants.kR1, MotorType.kBrushed);
  public CANSparkMax r2Motor = new CANSparkMax(Constants.kR2, MotorType.kBrushed);
  public MotorControllerGroup rGroup = new MotorControllerGroup(r1Motor, r2Motor);

  public CANSparkMax l1Motor = new CANSparkMax(Constants.kL1, MotorType.kBrushed);
  public CANSparkMax l2Motor = new CANSparkMax(Constants.kL2, MotorType.kBrushed);
  public MotorControllerGroup lGroup= new MotorControllerGroup(l1Motor, l2Motor);


  public DifferentialDrive haylazDrive = new DifferentialDrive(rGroup, lGroup);
  public DriveSubsystem() {

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void ArcadeDrive(double x , double z){
    haylazDrive.arcadeDrive(x,z, true);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
