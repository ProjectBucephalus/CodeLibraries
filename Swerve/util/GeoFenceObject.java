package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

public class GeoFenceObject
    {
        public enum ObjectTypes {walls, box, line, point};

        private double xLimit;
        private double yLimit;
        private double xSize;
        private double ySize;
        private objectTypes objectType;
        private double buffer;
        private double radius;
        private double fullBuffer;

        public GeoFenceObject()
        {
            xLimit = -100;
            yLimit = -100;
            xSize = 0;
            ySize = 0;
            objectType = ObjectTypes.walls;
            buffer = 0.5;
            radius = 0;
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
            objectType = ObjectTypes.walls;
            buffer = 0.5;
            radius = 0;
        }

        /**
         * Define rectangular geofence
         * @param xLimit Start x-coordinate of region, metres
         * @param yLimit Start y-coordinate of region, metres
         * @param xSize Size of region in x-axis, metres
         * @param ySize Size of region in y-axis, metres
         * @param objectType Type of object to avoid. Walls (stay within area), box (stay outside area), line, or point 
         */
        public GeoFenceObject(double xLimit, double yLimit, double xSize, double ySize, ObjectTypes objectType, double buffer)
        {
            this.xLimit = Math.min(xLimit, xLimit + xSize);
            this.yLimit = Math.min(yLimit, yLimit + ySize);
            this.xSize = Math.abs(xSize);
            this.ySize = Math.abs(ySize);
            this.objectType = objectType;
            this.buffer = Math.max(buffer, 0.1);
            radius = 0;
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
            if (objectType = ObjectTypes.box)
            {
                if (robotXY.getX() <= xLimit - checkRadius)         {return false;} // Far from -X barrier
                if (robotXY.getX() >= xLimit + xSize + checkRadius) {return false;} // Far from +X barrier
                if (robotXY.getY() <= yLimit - checkRadius)         {return false;} // Far from -Y barrier
                if (robotXY.getY() >= yLimit + ySize + checkRadius) {return false;} // Far from +Y barrier
                return true;                                                        // Close to at least one barrier
            }
            else
            {
                if      (robotXY.getX() >= xLimit + xSize - checkRadius) {return true;}  // Close to inside of +X barrier
                else if (robotXY.getX() <= xLimit + checkRadius)         {return true;}  // Close to inside of -X barrier
                else if (robotXY.getY() >= yLimit + ySize - checkRadius) {return true;}  // Close to inside of +Y barrier
                else if (robotXY.getY() <= yLimit + checkRadius)         {return true;}  // Close to inside of -Y barrier
                else                                                     {return false;}
            }
            return false;
        }

        private Translation2d pointDamping(double cornerX, double cornerY, Translation2d motionXY, double robotR, Translation2d robotXY, double radius)
        {
            // Calculates X and Y distances to the point
            double distanceX = cornerX - robotXY.getX;
            double distanceY = cornerY - robotXY.getY;
            // Calculates the normal distance to the corner through pythagoras; this is the actual distance between the robot and point
            double distanceN = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
            // Calculates the robot's motion normal and tangent to the point; i.e., towards and away from the point, and from side to side relative to the point
            double motionN   = ((distanceX * motionXY.getX) + (distanceY * motionXY.getY)) / distanceN;
            double motionT   = ((distanceX * motionXY.getY) - (distanceY * motionXY.getX)) / distanceN;
            // Clamps the normal motion, i.e. motion towards the point, in order to clamp robot speed
            motionN = Math.min(motionN, motionN * clamp(distanceN-(robotR + radius), 0, buffer) / (max(distanceX,distanceY) * buffer));
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
            // TODO: lock input to field coordinates

            double distanceToEdgeX;
            double distanceToEdgeY;
            fullBuffer = robotR + buffer + radius;
            
            if (!checkPosition(robotXY, fullBuffer)) {return motionXY;}
            // orth. v. diagonal v. internal
            double motionX = motionXY.getX();
            double motionY = motionXY.getY();
            double motionT;
            double motionN;
            double distanceN;
            if (objectType = ObjectTypes.point)
            {
                motionXY = pointDamping(xLimit, yLimit, motionXY, robotR, robotXY, radius);
                motionX = MotionXY.getX;
                motionY = MotionXY.getY;
            }
            else if (objectType = ObjectTypes.box) 
            {
                if (robotXY.getX() < xLimit - (robotR + radius)) 
                {
                    if (robotXY.getY() < yLimit - (robotR + radius)) // SW Corner
                    {
                        motionXY = pointDamping(xLimit, yLimit, motionXY, robotR, robotXY, radius);
                        motionX = MotionXY.getX;
                        motionY = MotionXY.getY;
                    }
                    else if (robotXY.getY() > yLimit + ySize + (robotR + radius)) // NW Corner
                    {
                        motionXY = pointDamping(xLimit, yLimit, motionXY, robotR, robotXY, radius);
                        motionX = MotionXY.getX;
                        motionY = MotionXY.getY;
                    }
                    else // W Cardinal
                    {
                        distanceToEdgeX = (xLimit - radius) - (robotXY.getX() + robotR); 
                        motionX = Math.min(motionX, (clamp(distanceToEdgeX, 0, buffer)) / buffer);
                    }
                }
                else if (robotXY.getX() > xLimit + xSize + (robotR + radius))
                {
                    if (robotXY.getY() < yLimit - (robotR + radius)) // SE Corner
                    {
                        motionXY = pointDamping(xLimit, yLimit, motionXY, robotR, robotXY, radius);
                        motionX = MotionXY.getX;
                        motionY = MotionXY.getY;
                    }
                    else if (robotXY.getY() > yLimit + ySize + (robotR + radius)) // NE Corner
                    {   
                        motionXY = pointDamping(xLimit, yLimit, motionXY, robotR, robotXY, radius);
                        motionX = MotionXY.getX;
                        motionY = MotionXY.getY;
                    }
                    else // E Cardinal
                    {
                        distanceToEdgeX = (robotXY.getX() - robotR) - (xLimit + xSize + radius);
                        motionX = Math.max(motionX, (-clamp(distanceToEdgeX, 0, buffer)) / buffer);
                    }
                }
                else 
                {
                    if (robotXY.getY() < yLimit - (robotR + radius)) // S Cardinal
                    {
                        distanceToEdgeY = (yLimit - radius) - (robotXY.getY() + robotR);
                        motionY = Math.min(motionY, (clamp(distanceToEdgeY, 0, buffer)) / buffer);
                    } 
                    else if (robotXY.getY() > yLimit + ySize + (robotR + radius)) // N Cardinal
                    {
                        distanceToEdgeY = (robotXY.getY() - robotR) - (yLimit + ySize + radius);
                        motionY = Math.max(motionY, (-clamp(distanceToEdgeY, 0, buffer)) / buffer);
                    }
                    else // Center (you've met a terrible fate *insert kazoo music here*)
                    {
                        motionX = clamp(motionX, -0.5, 0.5);
                        motionY = clamp(motionY, -0.5, 0.5);                                  
                    } 
                }
            }
            else    
            {
                // Calculates distance to the relevant edge of the field
                // Calculates edge position, and subtracts robot position + radius from edge position.

                // Sets the motion in the relevant direction to the minimum of the current motion
                // And the distance from the edge clamped between 0 and the edge buffer, and normalised to a maximum of 1.
                // This ensures the motion in that direction does not go above the clamped + normalised distance from the edge, to cap speed.
                if (motionX > 0)
                {   
                    distanceToEdgeX = (xLimit + xSize - radius) - (robotXY.getX() + robotR); 
                    motionX = Math.min(motionX, (clamp(distanceToEdgeX, 0, buffer)) / buffer);
                }
                else if (motionX < 0)
                {   
                    distanceToEdgeX = (robotXY.getX() - robotR) - (xLimit + radius);
                    motionX = Math.max(motionX, (-clamp(distanceToEdgeX, 0, buffer)) / buffer);
                }

                if (motionY > 0)
                {   
                    distanceToEdgeY = (yLimit + ySize - radius) - (robotXY.getY() + robotR);
                    motionY = Math.min(motionY, (clamp(distanceToEdgeY, 0, buffer)) / buffer);
                }
                else if (motionY < 0)
                {   
                    distanceToEdgeY = (robotXY.getY() - robotR) - (yLimit + radius);
                    motionY = Math.max(motionY, (-clamp(distanceToEdgeY, 0, buffer)) / buffer);
                }
            }
            // det. dist.
            // discard if dist. beyond threshold
            // det. angle robot to Geofence
            // convert motionXY to para. perp. components
            // discard if para. comp. away from Geofence
            // damp para. comp.
            // convert para. perp. comp. to motionXY
            
            return new Translation2d(motionX, motionY);
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