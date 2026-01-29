package org.firstinspires.ftc.teamcode.Extensions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

public class LimelightTurret {

    private final DcMotorEx turret;

    // PID gains (tune on robot)
    private double kP = 0.025;
    private double kI = 0.0;
    private double kD = 0.002;

    private double integral = 0.0;
    private double prevError = 0.0;
    private long lastTime = 0;

    private static final double MAX_POWER = 0.35;

    public LimelightTurret(HardwareMap hw, String turretMotorName) {
        turret = hw.get(DcMotorEx.class, turretMotorName);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /** Call this every loop() */
    public void update() {
        LimelightData data = getLimelightData();
        if (!data.valid) {
            turret.setPower(0);
            integral = 0;
            return;
        }

        double errorDeg = data.tx;  // target horizontal offset (degrees)

        long now = System.nanoTime();
        if (lastTime == 0) lastTime = now;
        double dt = (now - lastTime) / 1e9;

        integral += errorDeg * dt;
        double derivative = (errorDeg - prevError) / dt;

        double output =
                kP * errorDeg +
                        kI * integral +
                        kD * derivative;

        output = Range.clip(output, -MAX_POWER, MAX_POWER);
        turret.setPower(output);

        prevError = errorDeg;
        lastTime = now;
    }

    /** Stop turret manually */
    public void stop() {
        turret.setPower(0);
    }

    /** Optional live PID tuning */
    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    // ================= LIMELIGHT HTTP =================

    private LimelightData getLimelightData() {
        try {
            URL url = new URL("http://limelight.local:5807/results");
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(30);
            conn.setReadTimeout(30);

            BufferedReader br = new BufferedReader(
                    new InputStreamReader(conn.getInputStream()));
            StringBuilder json = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                json.append(line);
            }
            br.close();

            JSONObject root = new JSONObject(json.toString());

            LimelightData data = new LimelightData();
            data.valid = root.getBoolean("v");
            data.tx = root.getDouble("tx");
            data.ty = root.getDouble("ty");
            data.tid = root.optInt("tid", -1);

            return data;

        } catch (Exception e) {
            return new LimelightData(); // invalid
        }
    }

    // Simple container
    private static class LimelightData {
        boolean valid = false;
        double tx = 0;
        double ty = 0;
        int tid = -1;
    }
}
