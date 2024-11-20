// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

/** Motor with PID plus feedforward speed control. */
public class MotorSubsystem extends SubsystemBase implements AutoCloseable {

  /** Hardware components for the motor subsystem. */
  public static class Hardware {
    CANSparkMax motor;
    RelativeEncoder encoder;

    public Hardware(CANSparkMax motor, RelativeEncoder encoder) {
      this.motor = motor;
      this.encoder = encoder;
    }
  }

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private double motorVoltageCommand = 0.0;

  /** Create a new motorSubsystem for open loop voltage control. */
  public MotorSubsystem(Hardware motorHardware) {
    this.motor = motorHardware.motor;
    this.encoder = motorHardware.encoder;

    initializeMotor();

    ShuffleboardTab sbMotorTab = Shuffleboard.getTab("Motor");
    sbMotorTab.addNumber("Motor Voltage", this::getVoltageCommand);
  }

  private void initializeMotor() {

    motor.restoreFactoryDefaults();
    motor.clearFaults();

    // Setup the encoder scale factors and reset encoder to 0. Since this is a relation encoder,
    // motor position will only be correct if the motor is in the starting rest position when
    // the subsystem is constructed.
    encoder.setPositionConversionFactor(MotorConstants.MOTOR_ROTATIONS_PER_ENCODER_ROTATION);
    encoder.setVelocityConversionFactor(MotorConstants.MOTOR_ROTATIONS_PER_ENCODER_ROTATION);

    // Configure the motor to use EMF braking when idle and set voltage to 0.
    motor.setIdleMode(IdleMode.kBrake);
    motor.setVoltage(0.0);
  }

  /**
   * Create hardware devices for the motor subsystem.
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    CANSparkMax motorMotor = new CANSparkMax(MotorConstants.MOTOR_PORT, MotorType.kBrushless);
    RelativeEncoder motorEncoder = motorMotor.getEncoder();

    return new Hardware(motorMotor, motorEncoder);
  }

  @Override
  public void periodic() {
    // Put periodic functions like dashboards and data logging here.
    SmartDashboard.putNumber("Motor Voltage", getVoltageCommand());
    SmartDashboard.putNumber("Motor Velocity", getSpeed());
  }

  /** Set the output voltage to the motor. */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    motorVoltageCommand = voltage; // Save the output for simulation
  }

  /**
   * Set the motor idle mode to brake or coast.
   *
   * @param enableBrake Enable motor braking when idle
   */
  public void setBrakeMode(boolean enableBrake) {
    if (enableBrake) {
      motor.setIdleMode(IdleMode.kBrake);
    } else {
      motor.setIdleMode(IdleMode.kCoast);
    }
  }

  /** Returns the motor speed for PID control and logging (Units are RPM). */
  public double getSpeed() {
    return encoder.getVelocity();
  }

  /** Returns the motor motor commanded voltage. */
  public double getVoltageCommand() {
    return motorVoltageCommand;
  }

  /** Returns the motor current. */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity", this::getSpeed, null);
  }

  /** Close any objects that support it. */
  @Override
  public void close() {
    motor.close();
  }
}
