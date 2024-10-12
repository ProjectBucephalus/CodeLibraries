package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

public class GeoFenceObject
    {
        public double xLimit;
        public double yLimit;
        public double xSize;
        public double ySize;
        public boolean avoidInterior;
        public double buffer;

        public GeoFenceObject()
        {
            xLimit = -100;
            yLimit = -100;
            xSize = 0;
            ySize = 0;
            avoidInterior = true;
            buffer = 0.5;
        }

        /**
         * Define a single point to avoid
         * @param x x-coordinate, metres
         * @param y y-coordinate, metres
         */
        public GeoFenceObject(double x, double y)
        {
            xLimit = x;
            yLimit = y;
            xSize = 0;
            ySize = 0;
            avoidInterior = true;
            buffer = 0.5;
        }

        /**
         * Define rectangular geofence
         * @param xLimit Start x-coordinate of region, metres
         * @param yLimit Start y-coordinate of region, metres
         * @param xSize Size of region in x-axis, metres
         * @param ySize Size of region in y-axis, metres
         * @param avoidInterior Keep robot outside the marked area, false to keep robot within marked area 
         */
        public GeoFenceObject(double xLimit, double yLimit, double xSize, double ySize, boolean avoidInterior, double buffer)
        {
            this.xLimit = Math.min(xLimit, xLimit + xSize);
            this.yLimit = Math.min(yLimit, yLimit + ySize);
            this.xSize = Math.abs(xSize);
            this.ySize = Math.abs(ySize);
            this.avoidInterior = avoidInterior;
            this.buffer = buffer;
        }

        /**
         * Determines if the robot is close enough to the Geofence object to potentially need motion damping
         * @param robotXY Coordinates of the robot, metres
         * @param robotR Effective radius of the robot, metres
         * @return BOOLEAN is the robot close enough to the object to check more thoroughly
         */
        public boolean checkPosition(Translation2d robotXY, double robotR)
        {
            // det. robotXY within (Geofence + robotR + buffer) box
            if (avoidInterior)
            {
                if      (robotXY.getX() <= xLimit + xSize + (robotR + buffer)  // Within +X of barrier + buffer
                     &&  robotXY.getX() >= xLimit - (robotR + buffer))         // Within -X of barrier + buffer
                            {return true;} 
                else if (robotXY.getY() <= yLimit + ySize + (robotR + buffer)  // Within +Y of barrier + buffer
                     &&  robotXY.getY() >= yLimit - (robotR + buffer))         // Within -Y of barrier + buffer
                            {return true;}
            }
            else
            {
                if      (robotXY.getX() >= xLimit + xSize - (robotR + buffer)) {return true;}  // Close to inside of +X barrier
                else if (robotXY.getX() <= xLimit + (robotR + buffer))         {return true;}  // Close to inside of -X barrier
                else if (robotXY.getY() >= yLimit + ySize - (robotR + buffer)) {return true;}  // Close to inside of +Y barrier
                else if (robotXY.getY() <= yLimit + (robotR + buffer))         {return true;}  // Close to inside of -Y barrier
            }
            return false;
        }

        /**
         * Damps the motion of the robot in the direction of a Geofence object to prevent collision
         * @param robotXY Coordinates of the robot, metres
         * @param motionXY Control input to be damped, must be aligned to field coordinates
         * @param robotR Effective radius of the robot, metres
         * @return Modified input to send to drive command
         */
        public Translation2d dampMotion(Translation2d robotXY, Translation2d motionXY, double robotR)
        {
            // TODO: lock input to field coordinates
            
            if (!checkPosition(robotXY, robotR)) {return motionXY;}
            // orth. v. diagonal v. internal
            double motionX = motionXY.getX();
            double motionY = motionXY.getY();
            if (avoidInterior) 
            {
                // TODO: Orthogonal only check
                // TODO: Orthogonal compensation
                // TODO: Diagonal compensation
            }
            else
            {
                if (motionX > 0)
                {
                    motionX *= (clamp((xLimit + xSize) - (robotXY.getX() + robotR), 0, buffer))/buffer;
                }
                else if (motionX < 0)
                {
                    motionX *= (clamp((robotXY.getX() - robotR) - (xLimit), 0, buffer))/buffer;
                }

                if (motionY > 0)
                {
                    motionY *= (clamp((yLimit + ySize) - (robotXY.getY() + robotR), 0, buffer))/buffer;
                }
                else if (motionY < 0)
                {
                    motionY *= (clamp((robotXY.getY() - robotR) - (yLimit), 0, buffer))/buffer;
                }

                return new Translation2d(motionX, motionY);
            }
            // det. dist.
            // discard if dist. beyond threshold
            // det. angle robot to Geofence
            // convert motionXY to para. perp. components
            // discard if para. comp. away from Geofence
            // damp para. comp.
            // convert para. perp. comp. to motionXY
            
            return motionXY;
        }

        private double clamp(double value, double min, double max)
        {
            return Math.min(Math.max(value, Math.min(min,max)), Math.max(min,max));
        }

        public Translation2d[] getObject()
        {
            Translation2d[] corners = new Translation2d[2];
            corners[0] = new Translation2d(xLimit,yLimit);
            corners[1] = new Translation2d(xLimit+xSize,yLimit+ySize);
            return corners;
        }
    }