package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HopperSubsystem extends SubsystemBase {
    // TODO: assign ID
    public final TalonFX intakeMotor = new TalonFX(100, "rio");
    
    public enum WheelDirections {
        None,
        Normal,
        Backwards
    }
    
    private WheelDirections state = WheelDirections.None;
    private double wheelSpeed = 0.0;

    private double getMultiplier(WheelDirections direction) {
        switch (direction) {
            case Normal:
                return 1.0;
            case Backwards:
                return -1.0;
            case None:
            default:
                return 0.0;
        }
    }

    public void setWheelDirection(WheelDirections direction) {
        state = direction;
    }

    public void runIntake(double speed) {
        wheelSpeed = speed;
    }

    @Override
    public void periodic() {
        double outputVoltage = wheelSpeed * getMultiplier(state);
        intakeMotor.setControl(new VoltageOut(outputVoltage));
        wheelSpeed = 0;
    }
}