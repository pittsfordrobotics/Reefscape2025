package frc.robot.Constants;

public class IntakeConstants {

    public static final int CAN_INTAKE_MOTOR = 1;
    

    // FF constants
    // See diagram its very usefull
    // deg -> rad = deg * .0175
    // inches -> meters = inches * .0254
    // lbs -> kg = lbs * .453592
    public static final double Alpha_Offset = -23.8*.0175;
    public static final double L3_WpivtoCm2 = 7.8*.0254;
    public static final double L2_WpivPerptoWpiv = 4.18*.0254;
    public static final double M2_Total_Mass_of_Intake = 14*.453592;
    public static final double INTAKE_Pivot_FF_Multiplier = 0.036; //.35 for basic ff

}