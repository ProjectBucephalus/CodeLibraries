package frc.robot.Swerve.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Swerve.util.Conversions;

public class GeoFenceObject
    {
        /*
         * NOTE: +X is robot Forward, called North, +Y is robot Left, called West
         */


        public enum ObjectTypes {walls, box, line, point};

        private double Xa;
        private double Ya;
        private double Xb;
        private double Yb;
        private ObjectTypes objectType;
        private double buffer;
        private double radius;
        private double fullRadius;

        private double dXab = 0;
        private double dYab = 0;
        private double dot2ab = 0;

        /**
         * Default empty constructor
         */
        public GeoFenceObject()
        {
            Xa = -100;
            Ya = -100;
            Xb = -100;
            Yb = -100;
            objectType = ObjectTypes.point;
            buffer = 1;
            radius = 0;
        }

        /**
         * Define a minimum point-type object to avoid
         * @param x x-coordinate, metres
         * @param y y-coordinate, metres
         */
        public GeoFenceObject(double x, double y)
        {
            this(x, y, 0.5, 0);
        }

        /**
         * Define a simple point-type object with radius
         * @param x x-coordinate, metres
         * @param y y-coordinate, metres
         * @param r hard-stop radius around object, metres
         */
        public GeoFenceObject(double x, double y, double r)
        {
            this(x, y, 0.5, r);
        }

        /**
         * Define a point-type object to avoid
         * @param x x-coordinate, metres
         * @param y y-coordinate, metres
         * @param buffer range over which the robot slows down, metres
         * @param radius hard-stop radius around object, metres
         */
        public GeoFenceObject(double x, double y, double buffer, double radius)
        {
            this(x, y, x, y, buffer, radius, ObjectTypes.point);
        }

        /**
         * Define a line-type object to avoid 
         * @param Xa x-coordinate in meters of the first point
         * @param Ya y-coordinate in meters of the first point
         * @param Xb x-coordinate in meters of the second point
         * @param Yb y-coordinate in meters of the second point
         * @param buffer range over which the robot slows down, metres  
         * @param radius hard-stop radius around object, metres
         */
        public GeoFenceObject(double Xa, double Ya, double Xb, double Yb, double buffer, double radius) 
        {
            this(Xa, Ya, Xb, Yb, buffer, radius, ObjectTypes.line);
        }

        /**
         * Define rectangular geofence (box or walls)
         * @param Xa Start x-coordinate of region, metres
         * @param Ya Start y-coordinate of region, metres
         * @param Xb Size of region in x-axis, metres
         * @param Yb Size of region in y-axis, metres
         * @param buffer range over which the robot slows down, metres  
         * @param radius hard-stop radius around object, metres
         * @param objectType Type of object to avoid. Walls (stay within area), box (stay outside area), line, or point 
         */
        public GeoFenceObject(double Xa, double Ya, double Xb, double Yb, double buffer, double radius, ObjectTypes objectType)
        {
            if (objectType == ObjectTypes.walls || objectType == ObjectTypes.box)
            {
                this.Xa = Math.min(Xa, Xb);
                this.Ya = Math.min(Ya, Yb);
                this.Xb = Math.max(Xa, Xb);
                this.Yb = Math.max(Ya, Yb);
            }
            else if (objectType == ObjectTypes.point)
            {
                this.Xa = Xa;
                this.Xb = Xa;
                this.Ya = Ya;
                this.Yb = Ya;
            }
            else
            {
                this.Xa = Xa;
                this.Xb = Xb;
                this.Ya = Ya;
                this.Yb = Yb;

                this.dXab = Xb - Xa;
                this.dYab = Yb - Ya;
                this.dot2ab = Math.pow(dXab,2) + Math.pow(dYab,2);
            }
            this.objectType = objectType;
            this.buffer = Math.max(buffer, 0.1);
            this.radius = radius;
        }

        /**
         * Determines if the robot is close enough to the Geofence object to potentially need motion damping
         * @param robotXY Coordinates of the robot, metres
         * @param robotR Effective radius of the robot, metres
         * @return BOOLEAN is the robot close enough to the object to check more thoroughly
         */
        public boolean checkPosition(Translation2d robotXY, double checkRadius)
        {
            // det. robotXY within (Geofence + robotR + buffer) box
            if (objectType == ObjectTypes.walls)
            {
                if      (robotXY.getX() >= Xb - checkRadius) {return true;}  // Close to inside of +X barrier
                else if (robotXY.getX() <= Xa + checkRadius) {return true;}  // Close to inside of -X barrier
                else if (robotXY.getY() >= Yb - checkRadius) {return true;}  // Close to inside of +Y barrier
                else if (robotXY.getY() <= Ya + checkRadius) {return true;}  // Close to inside of -Y barrier
                else                                         {return false;}
            }
            else if (objectType == ObjectTypes.box)
            {
                if (robotXY.getX() <= Xa - checkRadius) {return false;} // Far from -X barrier
                if (robotXY.getX() >= Xb + checkRadius) {return false;} // Far from +X barrier
                if (robotXY.getY() <= Ya - checkRadius) {return false;} // Far from -Y barrier
                if (robotXY.getY() >= Yb + checkRadius) {return false;} // Far from +Y barrier
                return true;                                            // Close to at least one barrier
            }
            else
            {
                if (robotXY.getX() <= Math.min(Xa, Xb) - checkRadius) {return false;} // Far from -X barrier
                if (robotXY.getX() >= Math.max(Xa, Xb) + checkRadius) {return false;} // Far from +X barrier
                if (robotXY.getY() <= Math.min(Ya, Yb) - checkRadius) {return false;} // Far from -Y barrier
                if (robotXY.getY() >= Math.max(Ya, Yb) + checkRadius) {return false;} // Far from +Y barrier
                return true;                                            // Close to at least one barrier
            }
        }

        private Translation2d pointDamping(double pointX, double pointY, Translation2d motionXY, double robotR, Translation2d robotXY)
        {
            // Calculates X and Y distances to the point
            double distanceX = pointX - robotXY.getX();
            double distanceY = pointY - robotXY.getY();
            // Calculates the normal distance to the corner through pythagoras; this is the actual distance between the robot and point
            double distanceN = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
            // Calculates the robot's motion normal and tangent to the point; i.e., towards and away from the point, and from side to side relative to the point
            double motionN   = ((distanceX * motionXY.getX()) + (distanceY * motionXY.getY())) / distanceN;
            double motionT   = ((distanceX * motionXY.getY()) - (distanceY * motionXY.getX())) / distanceN;
            
            // Clamps the normal motion, i.e. motion towards the point, in order to clamp robot speed
            // Sets maximum input towards the object as:
            //      (position within the buffer normalised to [0..1]) * (angle normalisation factor [1..sqrt(2)])
            //         (dNormal - object radii)[0..buffer] / buffer   *    (mNormal / max(|X|,|Y|))
            motionN = Math.min(motionN, motionN * Conversions.clamp(distanceN-(robotR + radius), 0, buffer)
                                         / (Math.max(Math.abs(distanceX),Math.abs(distanceY)) * buffer));
            
            // Converts clamped motion from normal back to X and Y
            double motionX   = ((motionN * distanceX) - (motionT * distanceY)) / distanceN;
            double motionY   = ((motionN * distanceY) + (motionT * distanceX)) / distanceN;

            return new Translation2d(motionX, motionY);
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
            double distanceToEdgeX;
            double distanceToEdgeY;
            fullRadius = robotR + radius;
            
            if (!checkPosition(robotXY, fullRadius + buffer)) {return motionXY;}
            // orth. v. diagonal v. internal
            double motionX = motionXY.getX();
            double motionY = motionXY.getY();
            
            //if(objectType != null)
            switch (objectType) 
            {
                case point:
                    return pointDamping(Xa, Ya, motionXY, robotR, robotXY);
                case line:
                    /*
                     * Calculates the nearest point on the line to the robot
                     * Uses the dot product of the lines A-B and A-Robot to project the robot position onto the line
                     * Then clamps the calculated point between the line endpoints
                     *      
                     *            /              (robotX - aX) * (bX - aX) + (robotY - aY) * (bY - aY) \
                     *      aXY + | (bXY - aXY) *   ------------------------------------------------   |
                     *            \                       (robotX - aX)^2 + (robotY - aY)^2            /
                     */
                    distanceToEdgeX = robotXY.getX() - Xa;
                    distanceToEdgeY = robotXY.getY() - Ya;
                    double dot = ((distanceToEdgeX * dXab) + (distanceToEdgeY * dYab)) / dot2ab; // Normalised dot product of the two lines
                    return pointDamping(Conversions.clamp(Xa + dXab * dot, Xa, Xb), Conversions.clamp(Ya + dYab * dot, Ya, Yb), motionXY, robotR, robotXY);
                case box:
                    if (robotXY.getX() < Xa - radius) 
                    {
                        if (robotXY.getY() < Ya - radius) // SE Corner
                        {
                            return pointDamping(Xa, Ya, motionXY, robotR, robotXY);
                        }
                        else if (robotXY.getY() > Yb + radius) // SW Corner
                        {
                            return pointDamping(Xa, Yb, motionXY, robotR, robotXY);
                        }
                        else // S Cardinal
                        {
                            distanceToEdgeX = (Xa - radius) - (robotXY.getX() + robotR); 
                            motionX = Math.min(motionX, (Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
                        }
                    }
                    else if (robotXY.getX() > Xb + radius)
                    {
                        if (robotXY.getY() < Ya - radius) // NE Corner
                        {
                            return pointDamping(Xb, Ya, motionXY, robotR, robotXY);
                        }
                        else if (robotXY.getY() > Yb + radius) // NW Corner
                        {   
                            return pointDamping(Xb, Yb, motionXY, robotR, robotXY);
                        }
                        else // N Cardinal
                        {
                            distanceToEdgeX = (robotXY.getX() - robotR) - (Xb + radius);
                            motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
                        }
                    }
                    else 
                    {
                        if (robotXY.getY() < Ya - radius) // E Cardinal
                        {
                            distanceToEdgeY = (Ya - radius) - (robotXY.getY() + robotR);
                            motionY = Math.min(motionY, (Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
                        } 
                        else if (robotXY.getY() > Yb + radius) // W Cardinal
                        {
                            distanceToEdgeY = (robotXY.getY() - robotR) - (Yb + radius);
                            motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
                        }
                        else // Center (you've met a terrible fate *insert kazoo music here*)
                        {
                            motionX = Conversions.clamp(motionX, -0.5, 0.5);
                            motionY = Conversions.clamp(motionY, -0.5, 0.5);                                  
                        } 
                    }
                    return new Translation2d(motionX, motionY);
                case walls:
                    // Calculates distance to the relevant edge of the field
                    // Calculates edge position, and subtracts robot position + radius from edge position.

                    // Sets the motion in the relevant direction to the minimum of the current motion
                    // And the distance from the edge clamped between 0 and the edge buffer, and normalised to a maximum of 1.
                    // This ensures the motion in that direction does not go above the clamped + normalised distance from the edge, to cap speed.
                    if (motionX > 0)
                    {   
                        distanceToEdgeX = (Xb - radius) - (robotXY.getX() + robotR); 
                        motionX = Math.min(motionX, (Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
                    }
                    else if (motionX < 0)
                    {   
                        distanceToEdgeX = (robotXY.getX() - robotR) - (Xa + radius);
                        motionX = Math.max(motionX, (-Conversions.clamp(distanceToEdgeX, 0, buffer)) / buffer);
                    }

                    if (motionY > 0)
                    {   
                        distanceToEdgeY = (Yb - radius) - (robotXY.getY() + robotR);
                        motionY = Math.min(motionY, (Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
                    }
                    else if (motionY < 0)
                    {   
                        distanceToEdgeY = (robotXY.getY() - robotR) - (Ya + radius);
                        motionY = Math.max(motionY, (-Conversions.clamp(distanceToEdgeY, 0, buffer)) / buffer);
                    }
                    return new Translation2d(motionX, motionY);
                default:
                    return motionXY;
            }            
        }



        public Translation2d[] getObject()
        {
            Translation2d[] corners = new Translation2d[2];
            corners[0] = new Translation2d(Xa,Ya);
            corners[1] = new Translation2d(Xb,Yb);
            return corners;
        }
    }