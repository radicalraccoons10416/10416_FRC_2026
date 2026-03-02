package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX shooterMotorLeft = new TalonFX(51, "rio");
    public final TalonFX shooterMotorRight = new TalonFX(52, "rio");
    public final TalonFX intakeMotor = new TalonFX(53, "rio");
    
    double shooterWheelSpeed = 0;
    double intakeWheelSpeed = 0;

    public ShooterSubsystem(){
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        //slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4; // A position error of 3 rotations results in 1.2 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps(Max Speed)
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)(How fast we want to get to max speed)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)(Idk)

        shooterMotorRight.getConfigurator().apply(slot0Configs);
        shooterMotorLeft.getConfigurator().apply(slot0Configs);
       shooterMotorRight.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);
    }

    public void runShooter(int speed) {
        shooterWheelSpeed = speed;
        intakeWheelSpeed = 5;
    }
    
    public void stopShooter() {
        shooterWheelSpeed = 0;
        intakeWheelSpeed = 0;
    }

    @Override
    public void periodic() {
        shooterMotorLeft.setControl(new MotionMagicVelocityVoltage(shooterWheelSpeed));
        shooterMotorRight.setControl(new Follower(shooterMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        intakeMotor.setControl(new VoltageOut(intakeWheelSpeed));
    }
}