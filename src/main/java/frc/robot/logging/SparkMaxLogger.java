package frc.robot.logging;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {
    public SparkMaxLogger() {
        super(SparkMax.class);
    }
    
    @Override
    public void update(EpilogueBackend backend, SparkMax sparkMax) {
        backend.log("SparkMax ID", sparkMax.getDeviceId());
        backend.log("Motor Temperature (Degrees Celsius)", sparkMax.getMotorTemperature());
        backend.log("Output Current", sparkMax.getOutputCurrent());
        backend.log("Encoder Position", sparkMax.getEncoder().getPosition());
        backend.log("Encoder Velocity", sparkMax.getEncoder().getVelocity());
        
    }
}



/* 
public class SparkMaxLogger {
    private final SparkMax sparkMax;

    public SparkMaxLogger(SparkMax sparkMax) {
        this.sparkMax = sparkMax;
        
    }

    @Logged(name="Applied output (V)")
    public double getAppliedOutput() {
        return sparkMax.getAppliedOutput();
    }

    @Logged(name="Output current (A)")
    public double getOutputCurrent() {
        return sparkMax.getOutputCurrent();
    }

    @Logged(name="Encoder position")
    public double getEncoderPosition() {
        return sparkMax.getEncoder().getPosition();
    }

    @Logged(name="Encoder velocity")
    public double getEncoderVelocity() {
        return sparkMax.getEncoder().getVelocity();
    }

    @Logged(name="Temperature (C)")
    public double getTemperature() {
        return sparkMax.getMotorTemperature();
    }

    @Logged(name="Is at reverse limit")
    public boolean isAtReverseLimit() {
        return sparkMax.getReverseLimitSwitch().isPressed();
    }

    @Logged(name="Is at forward limit")
    public boolean isAtForwarLimit() {
        return sparkMax.getForwardLimitSwitch().isPressed();
    }
} */
