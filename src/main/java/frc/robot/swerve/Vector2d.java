package frc.robot.swerve;

public class Vector2d {
    public final double x;
    public final double y;
    public final double magnitude;
    public final double angle;

    public Vector2d(Point point) {
        if (point instanceof CartesianPoint) {
            this.x = point.var1;
            this.y = point.var2;
            this.magnitude = calculateMagnitude(x, y);
            this.angle = calculateAngle(x, y);
        } else {
            this.x = point.var1 * Math.cos(point.var2);
            this.y = point.var1 * Math.sin(point.var2);
            this.magnitude = point.var1;
            this.angle = point.var2 % (2 * Math.PI);
        }
    }

    public Vector2d() {
        this(new Vector2d.CartesianPoint(0, 0));
    }

    private double calculateMagnitude(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    private double calculateAngle(double x, double y) {
        if (x >= 0.0) {
            return Math.atan(y / x);
        } else {
            return Math.atan(y / x) + Math.PI;
        }
    }

    public Vector2d rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2d(new CartesianPoint(x * cos - y * sin, x * sin + y * cos));
    }

    private abstract static class Point {
        public final double var1;
        public final double var2;

        public Point(double var1, double var2) {
            this.var1 = var1;
            this.var2 = var2;
        }
    }

    public static class CartesianPoint extends Point {
        public CartesianPoint(double x, double y) {
            super(x, y);
        }
    }

    public static class PolarPoint extends Point {
        public PolarPoint(double r, double theta) {
            super(r, theta);
        }
    }
}
