// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_VictorSPX motor = new WPI_VictorSPX(Constants.ArmConstants.motorPort);
  private final Encoder encoder = new Encoder(Constants.EncoderConstants.channel1, Constants.EncoderConstants.channel2);

  /** Creates a new Arm. */
  public ArmSubsystem() {
    motor.setNeutralMode(NeutralMode.Brake);
    encoder.reset();
    //encoder.setDistancePerPulse(Constants.EncoderConstants.distancePerPulse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setSpeed(double speed) {
    motor.set(MathUtil.clamp(speed, -1, 1) * 0.1);
  }

  public double getEncoderDistance() {
    return encoder.getDistance();
  }

  public double getEncoderAngle(){
    return getEncoderDistance()*360/Constants.EncoderConstants.pulsesPerRev;
  }
}