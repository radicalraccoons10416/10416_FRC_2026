// We ended up not finishing/using our climber


package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.States;

public class ClimberSubsystem extends SubsystemBase {
    CANBus rio = new CANBus("rio");
    public final TalonFX climberMotor = new TalonFX(56, rio);

    private States state = States.NONE;
    private double wheelSpeed = 0;

    // public ClimberSubsystem(){
    //     var slot0Configs = new Slot0Configs();

    //     slot0Configs.kP = 0;
    //     slot0Configs.kD = 0;

    //     climberMotor.getConfigurator().apply(slot0Configs);
    //     climberMotor.setNeutralMode(NeutralModeValue.Brake);
    // }

    // PositionTorqueCurrentFOC m_request = new PositionTorqueCurrentFOC(0).withSlot(0);
    // double position = 0;

    // public Command goToPos(double pos){
    //     return run(() ->
    //         position = pos
    //     );
    // }

    public Command runClimber(States direction) {
        return run(() -> {
            wheelSpeed = 3;
            state = direction;
        });
    }

    @Override
    public void periodic() {
        double outputVoltage = wheelSpeed * state.getMultiplier();
        climberMotor.setControl(new VoltageOut(outputVoltage));
        // climberMotor.setControl(m_request.withPosition(position));
        wheelSpeed = 0;
    }
}