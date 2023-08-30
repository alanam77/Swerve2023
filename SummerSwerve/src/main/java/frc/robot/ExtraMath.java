// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ExtraMath {
    public static final double leftAngleError(double startingAngle, double travelledAngle) {
        double error = travelledAngle - startingAngle;
        if(error < 0) {
            error += 360;
        }
        return error;
    }

    public static final double angleError(double startingAngle, double travelledAngle) {
        double error = startingAngle - travelledAngle;
        if(Math.abs(error) > 180) {
            return error - Math.copySign(360, error);
        }
        return error;
    }

    public static final double simpleAngleError(double startingAngle, double travelledAngle) {
        double x = startingAngle - travelledAngle;
        return mod((x-Math.PI), 2*Math.PI) - Math.PI;
    }

    public static final double mod(double x, double y) {
        double r = x % y;
        
        return x < 0 ? r+=y : r;
    }

    public static final double atanNew(double x, double y) {
        if(x < 0) {
            return Math.atan(y/x) + Math.PI;
        } else if (y < 0) {
            return Math.atan(y/x) + (Math.PI * 2);
        }
        return Math.atan(y/x);
    }

    public static final double exponential(double x, double a, double b) {
        return Math.copySign(x*x*a, x) + x*b;
    }

    public static final double clip(double x, double high) {
        if(Math.abs(x) > high) {
            return Math.copySign(high, x);
        } 
        return x;
    }

    public static final double clipLowBound(double x, double a) {
        if(Math.abs(x) < a) {
            return 0;
        }
        return x;
    }
}
