package org.curtinfrc.frc2026.subsystems.intake;

import static org.curtinfrc.frc2026.util.PhoenixUtil.tryUntilOk;

import org.curtinfrc.frc2026.util.PhoenixUtil;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOComp implements IntakeIO {

    //Creating variable called motorID for ports
    private static final int motorID = 99;

    //Creating variable intakeMotor and inserting motorID to control the motor
    private final TalonFX intakeMotor = new TalonFX(motorID);

    //Getting values and placing them into variables. 
    private final StatusSignal<Voltage> voltage = intakeMotor.getMotorVoltage ();
    private final StatusSignal<AngularVelocity> angularVelocity = intakeMotor.getVelocity();
    private final StatusSignal<Angle> angle = intakeMotor.getPosition();
    private final StatusSignal<Current> current = intakeMotor.getStatorCurrent();
    private static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs ().withSupplyCurrentLimit(30).withStatorCurrentLimit(60);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    
    //constructor
    public IntakeIOComp() {
        //literally trying until it is ok. The max attempts is 5.  
        tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs ().
        withInverted(InvertedValue.Clockwise_Positive)).withCurrentLimits(currentLimits)));

        //Setting update frequency
        BaseStatusSignal.setUpdateFrequencyForAll(20.0, voltage, current, angle, angularVelocity);

        //Setting update frequency (which is slowed down) for variables which do not have an update frequency
        intakeMotor.optimizeBusUtilization ();

        PhoenixUtil.registerSignals(false, voltage, current, angularVelocity, angle);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.positionRotations = angle.getValueAsDouble();
        inputs.angularVelocityRotationsPerMinute = angularVelocity.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        voltageRequest.withOutput(volts); 
    }
}






