package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
 private CANSparkMax armMotor = new CANSparkMax(Constants.kArm, MotorType.kBrushless);
 private CANSparkMax intakeMotor = new CANSparkMax(Constants.kIntake, MotorType.kBrushless);
 private Encoder armEncoder = new Encoder(Constants.ArmEncoder.port0, Constants.ArmEncoder.port1);

 

 public ArmSubsystem() {
    armMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}