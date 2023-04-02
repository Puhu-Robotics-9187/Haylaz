// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor = new CANSparkMax(31, MotorType.kBrushless);

  private Encoder armEncoder = new Encoder(Constants.ArmConstants.armEncA, Constants.ArmConstants.armEncB);

  private PIDController armPID = new PIDController(0, 0, 0);
  /** Creates a new Arm. */
  public Arm() {
    armEncoder.setDistancePerPulse(Constants.ArmConstants.armEncDPR);
    armEncoder.setReverseDirection(false);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.enableVoltageCompensation(11);
  }

  public double getAngle(){
    return armEncoder.getDistance();
  }

  @Override
  public void periodic() {

  }
}
