package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardStuff extends SubsystemBase {

    public DashboardStuff() {
        createSmartDashboardNumber("test", 1.434);
    }

    /**
     * Initialize value on SmartDashboard for user input, but leave old value if already present.
     *
     * @param key The SmartDashboard key to associate with the value.
     * @param defValue The default value to assign if not already on dashboard.
     *
     * @return The current value that appears on the dashboard.
     */
    public static double createSmartDashboardNumber(String key, double defValue) {

        // See if already on dashboard, and if so, fetch current value
        double value = SmartDashboard.getNumber(key, defValue);

        // Make sure value is on dashboard, puts back current value if already set
        // otherwise puts back default value
        SmartDashboard.putNumber(key, value);

        return value;
    }

    @Override
    public void periodic() {
        System.out.println(SmartDashboard.getNumber("test", 0));
    }

    
}
