using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Windows.Forms;
using System.Threading;
using Microsoft.Kinect;
using LightBuzz;
using LightBuzz.Vitruvius;
using System.Windows.Media.Media3D;
using System.Windows;
//using System.Numerics;

namespace alice_server
{
    class SerialPortProgram
    {
        class RobotJoint
        {
            public int MaxAngle = 135;
            public int MinAngle = 45;
            public int StartingAngle = 90;
            public bool InvertKinectAngle = false;
            public int KinectAngleOffset = 0;
            private int _angle = 90;
            public int Angle
            {
                get
                {
                    return _angle;
                }
                set{
                    if(value > MaxAngle)
                    {
                        _angle = MaxAngle;
                    } else if (value < MinAngle)
                    {
                        _angle = MinAngle;
                    } else
                    {
                        _angle = value;
                    }
                }
            } 
            public void SetAngleFromKinectAngle(int kinectAngle)
            {
               if (InvertKinectAngle)
                {
                    kinectAngle = 180 - kinectAngle;
                }
                Angle = kinectAngle + KinectAngleOffset;
            }
        }
        const int
            LEFT_GRIPPER = 0
            , LEFT_ELBOW_PITCH = 1
            , LEFT_ELBOW_YAW = 2
            , LEFT_SHOULDER_PITCH = 3
            , LEFT_SHOULDER_YAW = 4
            , LEFT_WRIST = 5
            , HEAD_PITCH = 7
            , HEAD_YAW = 8
            , RIGHT_GRIPPER = 10
            , RIGHT_WRIST = 11
            , RIGHT_ELBOW_PITCH = 12
            , RIGHT_ELBOW_YAW = 13
            , RIGHT_SHOULDER_PITCH = 14
            , RIGHT_SHOULDER_YAW = 15
            , LEFT_WHEEL = 16
            , RIGHT_WHEEL = 17
            , MAX_SPEED = 127
            , MIN_SPEED = 0
            , MAX_TURN_SPEED = 50
            , MAX_TURN_ANGLE = 30
            , MIN_TURN_ANGLE = 10
            ;
        const double
            MIN_LEAN = 0.3
            , MAX_LEAN = 0.6
            , MAX_TURN_DIFFERENTIAL = 1
            , Y_LEAN_OFFSET = 0.3;

        static KinectSensor _sensor;
        static MultiSourceFrameReader _reader;
        static IList<Body> _bodies;
        static int framecount = 0;
        static ulong trackingid = 0;
        static RobotJoint[] robotJoints = new RobotJoint[16];
        //static int left_wheel_speed;
        //static int right_wheel_speed;
        //static double lean_x;
        //static double lean_y;
        static void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    _bodies = new Body[frame.BodyFrameSource.BodyCount];
                    frame.GetAndRefreshBodyData(_bodies);
                    Body trackedBody = null;
                    //find the tracked body
                    foreach (var body in _bodies)
                    {
                        if (body != null)
                        {
                            if (body.TrackingId == trackingid)
                            {
                                trackedBody = body;
                            }
                            else if (body.IsTracked)
                            {
                                trackingid = body.TrackingId;
                                trackedBody = body;
                            }
                        }
                    }
                    var spine_shoulder = trackedBody.Joints[JointType.SpineShoulder];

                    var right_shoulder = trackedBody.Joints[JointType.ShoulderRight];
                    var right_elbow = trackedBody.Joints[JointType.ElbowRight];
                    var right_wrist = trackedBody.Joints[JointType.WristRight];

                    var left_shoulder = trackedBody.Joints[JointType.ShoulderLeft];
                    var left_elbow = trackedBody.Joints[JointType.ElbowLeft];
                    var left_wrist = trackedBody.Joints[JointType.WristLeft];
                    var left_thumb = trackedBody.Joints[JointType.ThumbLeft];
                    var left_hand_tip = trackedBody.Joints[JointType.HandTipLeft];

                    var left_hip = trackedBody.Joints[JointType.HipLeft];
                    var right_hip = trackedBody.Joints[JointType.HipRight];
                    var spine_base = trackedBody.Joints[JointType.SpineBase];

                    var left_hand_closed = (trackedBody.HandLeftState == HandState.Closed);
                    var right_hand_closed = (trackedBody.HandRightState == HandState.Closed);
                    var right_hand_open = (trackedBody.HandRightState == HandState.Open);
                    var left_hand_open = (trackedBody.HandLeftState == HandState.Open);
                    //lean_x = trackedBody.Lean.X;
                    //lean_y = trackedBody.Lean.Y + Y_LEAN_OFFSET;
                    //if (lean_y > 1) lean_y = 1;
                    if (spine_shoulder.Position.Z > 0)
                    {

                        //get shoulder pitch
                        var right_shoulder_pitch = Get3DAngle(new Vector3D(right_shoulder.Position.X, right_shoulder.Position.Y - 1, right_shoulder.Position.Z)
                            , right_shoulder.Position.ToVector3()
                            , right_elbow.Position.ToVector3()
                            );
                        robotJoints[RIGHT_SHOULDER_PITCH].SetAngleFromKinectAngle(
                            (int)right_shoulder_pitch
                        );

                        //get shoulder yaw
                        // if the pitch is less than 30 than the kinect can't determine yaw very well so we default it to 90
                        double right_shoulder_yaw = 90;
                        //if (right_shoulder_pitch > 30)
                        //{                            
                        right_shoulder_yaw = Get3DAngle(
                                new Vector3D(right_shoulder.Position.X - 1, 0, right_shoulder.Position.Z)
                                , new Vector3D(right_shoulder.Position.X, 0, right_shoulder.Position.Z)
                                , new Vector3D(right_elbow.Position.X, 0, right_elbow.Position.Z)
                            );
                        //}
                        robotJoints[RIGHT_SHOULDER_YAW].SetAngleFromKinectAngle(
                            (int)right_shoulder_yaw
                        );

                        //get elbow yaw
                        //unfortunately I don't have 3 easy tracked points that line up to the grid for this
                        //so I'm going to have to do the rotation myself.
                        //going to sleep on this one

                        //okay a couple days later
                        //first move the wrist to a point as if the shoulder was the origin
                        var transformedWrist = new Vector3D(
                            right_wrist.Position.X - right_shoulder.Position.X,
                            right_wrist.Position.Y - right_shoulder.Position.Y,
                            right_wrist.Position.Z - right_shoulder.Position.Z
                            );

                        //now lets position the wrist as if the shoulder pitch and yaw are both 90 degrees
                        var rotatedPoint = RotatePoint(new Vector(transformedWrist.Z * -1, transformedWrist.Y), 90 - right_shoulder_pitch);
                        transformedWrist.Z = rotatedPoint.X * -1;
                        transformedWrist.Y = rotatedPoint.Y;

                        rotatedPoint = RotatePoint(new Vector(transformedWrist.X, transformedWrist.Z), 90 - right_shoulder_yaw);
                        transformedWrist.X = rotatedPoint.X;
                        transformedWrist.Z = rotatedPoint.Y;

                        //now we should be able to measure
                        var right_elbow_yaw = Get3DAngle(
                                new Vector3D(-1, 0, 0)
                                , new Vector3D(0, 0, 0)
                                , new Vector3D(transformedWrist.X, transformedWrist.Y, 0)
                        );

                        robotJoints[RIGHT_ELBOW_YAW].SetAngleFromKinectAngle(
                            (int)right_elbow_yaw
                        );

                        //get elbow pitch
                        robotJoints[RIGHT_ELBOW_PITCH].SetAngleFromKinectAngle(
                            (int)
                            Get3DAngle(right_shoulder.Position.ToVector3()
                            , right_elbow.Position.ToVector3()
                            , right_wrist.Position.ToVector3()
                            )
                        );

                        //get shoulder pitch
                        var left_shoulder_pitch = Get3DAngle(new Vector3D(left_shoulder.Position.X, left_shoulder.Position.Y - 1, left_shoulder.Position.Z)
                            , left_shoulder.Position.ToVector3()
                            , left_elbow.Position.ToVector3()
                            );
                        robotJoints[LEFT_SHOULDER_PITCH].SetAngleFromKinectAngle(
                            (int)left_shoulder_pitch
                        );

                        //get shoulder yaw
                        // if the pitch is less than 30 than the kinect can't determine yaw very well so we default it to 90
                        double left_shoulder_yaw = 90;
                        //if (left_shoulder_pitch > 30)
                        //{
                        left_shoulder_yaw = Get3DAngle(
                                new Vector3D(left_shoulder.Position.X + 1, 0, left_shoulder.Position.Z)
                                    , new Vector3D(left_shoulder.Position.X, 0, left_shoulder.Position.Z)
                                    , new Vector3D(left_elbow.Position.X, 0, left_elbow.Position.Z)
                                );
                        //}
                        robotJoints[LEFT_SHOULDER_YAW].SetAngleFromKinectAngle(
                            (int)left_shoulder_yaw
                        );

                        //get elbow yaw
                        //unfortunately I don't have 3 easy tracked points that line up to the grid for this
                        //so I'm going to have to do the rotation myself.
                        //going to sleep on this one

                        //okay a couple days later
                        //first move the wrist to a point as if the shoulder was the origin
                        transformedWrist = new Vector3D(
                            left_wrist.Position.X - left_shoulder.Position.X,
                            left_wrist.Position.Y - left_shoulder.Position.Y,
                            left_wrist.Position.Z - left_shoulder.Position.Z
                            );

                        //now lets position the wrist as if the shoulder pitch and yaw are both 90 degrees
                        //and take the handtip & thumb along for the ride.
                        
                        rotatedPoint = RotatePoint(new Vector(transformedWrist.Z * -1, transformedWrist.Y), 90 - left_shoulder_pitch);
                        transformedWrist.Z = rotatedPoint.X * -1;
                        transformedWrist.Y = rotatedPoint.Y;                  

                        rotatedPoint = RotatePoint(new Vector(transformedWrist.X, transformedWrist.Z * -1), 90 - left_shoulder_yaw);
                        transformedWrist.X = rotatedPoint.X;
                        transformedWrist.Z = rotatedPoint.Y * -1;
                        
                        //now we should be able to measure
                        var left_elbow_yaw = Get3DAngle(
                                new Vector3D(1, 0, 0)
                                , new Vector3D(0, 0, 0)
                                , new Vector3D(transformedWrist.X, transformedWrist.Y, 0)
                        );

                        robotJoints[LEFT_ELBOW_YAW].SetAngleFromKinectAngle(
                            (int)left_elbow_yaw
                        );

                        var left_elbow_pitch = Get3DAngle(left_shoulder.Position.ToVector3()
                            , left_elbow.Position.ToVector3()
                            , left_wrist.Position.ToVector3()
                            );

                        //get elbow pitch
                        robotJoints[LEFT_ELBOW_PITCH].SetAngleFromKinectAngle(
                            (int)left_elbow_pitch
                            
                        );

                        //claws
                        //if (right_hand_closed)
                        //{
                        //    robotJoints[RIGHT_GRIPPER].Angle = 180;
                        //}
                        //else if (right_hand_open)
                        //{
                        //    robotJoints[RIGHT_GRIPPER].Angle = 0;
                        //}

                        //if (left_hand_closed)
                        //{
                        //    robotJoints[LEFT_GRIPPER].Angle = 180;
                        //}
                        //else if (left_hand_open)
                        //{
                        //    robotJoints[LEFT_GRIPPER].Angle = 0;
                        //}

                        //set wheel speed
                        //initialize wheel speed to 0
                        //left_wheel_speed = 0;
                        //right_wheel_speed = 0;

                        
                        //Comment out all lean for now
                        //first get base speed based on Y lean
                        //int base_speed = 0;
                        ////var lean_range = MAX_LEAN * 2;
                        //if (Math.Abs(lean_y) > MIN_LEAN)
                        //{
                        //    if (Math.Abs(lean_y) > MAX_LEAN)
                        //    {
                        //        if (lean_y > 0) { lean_y = (float)MAX_LEAN; }
                        //        else { lean_y = -1 * (float)MAX_LEAN; }
                        //    }
                        //    if (Math.Abs(lean_x) > MAX_LEAN)
                        //    {
                        //        if (lean_x > 0) { lean_x = (float)MAX_LEAN; }
                        //        else { lean_x = -1 * (float)MAX_LEAN; }
                        //    }
                        //    base_speed = (int)(((lean_y-MIN_LEAN)/(MAX_LEAN-MIN_LEAN)) * MAX_SPEED);
                        //    left_wheel_speed = base_speed;
                        //    right_wheel_speed = base_speed;
                            //now do turn
                            //if (lean_x > 0)//MIN_LEAN)
                            //{
                            //    //leaning right, so slow right wheel for right turn.
                            //    //right_wheel_speed = (int)((1 - ((-1 * lean_x) / MAX_LEAN) * MAX_TURN_DIFFERENTIAL) * right_wheel_speed);
                            //    //leaning right so add speed to left wheel
                            //    var speed_boost = 1 + (2*lean_x / MAX_LEAN);
                            //    left_wheel_speed = (int)(left_wheel_speed * speed_boost);
                            //}
                            //else if (lean_x < 0)//(MIN_LEAN * -1))
                            //{
                            //    //leaning left, so slow left wheel for left turn.
                            //    //left_wheel_speed = (int)((1 - (lean_x / MAX_LEAN) * MAX_TURN_DIFFERENTIAL) * left_wheel_speed);
                            //    //leaning left so add speed to right wheel
                            //    var speed_boost = 1 + ((-2*lean_x) / MAX_LEAN);
                            //    right_wheel_speed = (int)(right_wheel_speed * speed_boost);
                            //}

                        //} else if (lean_x > MIN_LEAN)
                        //    {
                        //        base_speed = (int)((lean_x / MAX_LEAN) * MAX_TURN_SPEED);
                        //        right_wheel_speed = base_speed * - 1;
                        //        left_wheel_speed = base_speed;
                        //    }
                        // else if (lean_x < (MIN_LEAN * -1))
                        //    {
                        //        base_speed = (int)(((-1 * lean_x) / MAX_LEAN) * MAX_TURN_SPEED);
                        //        left_wheel_speed = base_speed * -1;
                        //        right_wheel_speed = base_speed; 
                        // } else{
                        //    left_wheel_speed = 0;
                        //    right_wheel_speed = 0;
                        // }
                        
                        ////make sure neither wheel is greater than max speed
                        //if (Math.Abs(left_wheel_speed) > MAX_SPEED)
                        //{
                        //    if(left_wheel_speed>0) {
                        //        left_wheel_speed = MAX_SPEED;
                        //    } else
                        //    {
                        //        left_wheel_speed = -1 * MAX_SPEED;
                        //    }
                        //}
                        ////make sure neither wheel is greater than max speed
                        //if (Math.Abs(right_wheel_speed) > MAX_SPEED)
                        //{
                        //    if (right_wheel_speed > 0)
                        //    {
                        //        right_wheel_speed = MAX_SPEED;
                        //    }
                        //    else
                        //    {
                        //        right_wheel_speed = -1 * MAX_SPEED;
                        //    }
                        //}


                        
                    }
                }
            }
        }
        static double Get3DAngle(Vector3D a, Vector3D b, Vector3D c)
        {
            //from https://stackoverflow.com/questions/19729831/angle-between-3-points-in-3d-space

            //In pseudo-code, the vector BA(call it v1) is:
            //v1 = { A.x - B.x, A.y - B.y, A.z - B.z}

            //Similarly the vector BC(call it v2) is:
            //v2 = { C.x - B.x, C.y - B.y, C.z - B.z}

            //The dot product of v1 and v2 is a function of the cosine of the angle between them(it's scaled by the product of their magnitudes). So first normalize v1 and v2:

            //v1mag = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z)
            //v1norm = { v1.x / v1mag, v1.y / v1mag, v1.z / v1mag}

            //v2mag = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z)
            //v2norm = { v2.x / v2mag, v2.y / v2mag, v2.z / v2mag}

            //Then calculate the dot product:
            //res = v1norm.x * v2norm.x + v1norm.y * v2norm.y + v1norm.z * v2norm.z

            //And finally, recover the angle:
            //angle = acos(res)
            double angle = 0;

            var v1 = new Vector3D(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
            var v2 = new Vector3D(c.X - b.X, c.Y - b.Y, c.Z - b.Z);
            double v1mag = Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y + v1.Z * v1.Z);
            var v1norm = new Vector3D(v1.X / v1mag, v1.Y / v1mag, v1.Z / v1mag);
            double v2mag = Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y + v2.Z * v2.Z);
            var v2norm = new Vector3D(v2.X / v2mag, v2.Y / v2mag, v2.Z / v2mag);
            var res = v1norm.X * v2norm.X + v1norm.Y * v2norm.Y + v1norm.Z * v2norm.Z;
            angle = Math.Acos(res);            

            return angle.ToDegrees();
        }
        static byte ConvertSignedIntToByte(int signedInt)
        {
            var returnByte = signedInt + 127;
            if (returnByte > 255) { return 255; }
            if (returnByte < 0 ) { return 0; }
            return (byte)returnByte;
        }
        static Vector RotatePoint(Vector point0, double theta)
        {
            ////|cos(theta) -sin(theta)|   |X0|   |X1| 
            ////|sin(theta)  cos(theta)| . |Y0| = |Y1| 

            var point1 = new Vector();
            var x0 = point0.X;
            var y0 = point0.Y;
            float x1 = 0;
            float y1 = 0;

            x1 = (float)(Math.Cos(theta.ToRadians()) * x0 - Math.Sin(theta.ToRadians()) * y0);
            y1 = (float)(Math.Sin(theta.ToRadians()) * x0 + Math.Cos(theta.ToRadians()) * y0);

            point1.X = x1;
            point1.Y = y1;
            return point1;
        }
        // Create the serial port with basic settings
        //com 11 for Feather M4
        static SerialPort port = new SerialPort("COM7",
          9600, Parity.None, 8, StopBits.One);

        static byte[] command_bytes = new byte[20];

        static void Main(string[] args)
        {
            //initialize robot joints
            robotJoints[RIGHT_SHOULDER_PITCH] = new RobotJoint()
            {
                MaxAngle = 180
                , MinAngle = 0
                , InvertKinectAngle = false
                , KinectAngleOffset = 0
                , StartingAngle = 0
                , Angle=0
            };
            robotJoints[RIGHT_SHOULDER_YAW] = new RobotJoint()
            {
                MaxAngle = 180
                ,
                MinAngle = 0
                ,
                InvertKinectAngle = true
                ,
                KinectAngleOffset = 0
                ,
                StartingAngle = 90
                ,
                Angle = 90
            };
            robotJoints[RIGHT_ELBOW_YAW] = new RobotJoint()
            {
                MaxAngle = 180
                ,
                MinAngle = 0
                ,
                InvertKinectAngle = false
                ,
                KinectAngleOffset = 0
                ,
                StartingAngle = 90
                ,
                Angle = 90
            };
            robotJoints[RIGHT_ELBOW_PITCH] = new RobotJoint()
            {
                MaxAngle = 180
                ,
                MinAngle = 0
                ,
                InvertKinectAngle = true
                ,
                KinectAngleOffset = 0
                ,
                StartingAngle = 90
                ,
                Angle = 90
            };

            robotJoints[LEFT_SHOULDER_PITCH] = new RobotJoint()
            {
                MaxAngle = 180
                ,
                MinAngle = 0
                ,
                InvertKinectAngle = true
                ,
                KinectAngleOffset = 0
                ,
                StartingAngle = 180
                ,
                Angle = 180
            };
            robotJoints[LEFT_SHOULDER_YAW] = new RobotJoint()
            {
                MaxAngle = 180
                ,
                MinAngle = 0
                ,
                InvertKinectAngle = false
                ,
                KinectAngleOffset = 0
                ,
                StartingAngle = 90  
                ,
                Angle = 90
            };
            robotJoints[LEFT_ELBOW_YAW] = new RobotJoint()
            {
                MaxAngle = 180
                 ,
                MinAngle = 0
                 ,
                InvertKinectAngle = true
                 ,
                KinectAngleOffset = 0
                 ,
                StartingAngle = 90
                 ,
                Angle = 90
            };
            robotJoints[LEFT_ELBOW_PITCH] = new RobotJoint()
            {
                MaxAngle = 180
                ,
                MinAngle = 0
                ,
                InvertKinectAngle = false
                ,
                KinectAngleOffset = 0
                ,
                StartingAngle = 90
                ,
                Angle = 90
            };
            robotJoints[RIGHT_WRIST] = new RobotJoint()
            {
                MaxAngle = 180
                ,
                MinAngle = 0
                ,
                InvertKinectAngle = false
                ,
                KinectAngleOffset = 0
                ,
                StartingAngle = 0
                ,
                Angle = 0
            };
            //robotJoints[RIGHT_GRIPPER] = new RobotJoint()
            //{
            //    MaxAngle = 180
            //    ,
            //    MinAngle = 0
            //    ,
            //    InvertKinectAngle = false
            //    ,
            //    KinectAngleOffset = 0
            //    ,
            //    StartingAngle = 0
            //    ,
            //    Angle = 0
            //};
            //robotJoints[LEFT_GRIPPER] = new RobotJoint()
            //{
            //    MaxAngle = 180
            //    ,
            //    MinAngle = 0
            //    ,
            //    InvertKinectAngle = false
            //    ,
            //    KinectAngleOffset = 0
            //    ,
            //    StartingAngle = 180
            //    ,
            //    Angle = 180
            //};
            for (var i = 0; i<20; i++)
            {
                command_bytes[i] = 90;
            }


            //command_bytes[LEFT_GRIPPER] = 90;
            command_bytes[LEFT_ELBOW_PITCH] = 135;
            command_bytes[LEFT_ELBOW_YAW] = 90;
            command_bytes[LEFT_SHOULDER_PITCH] = 180;
            command_bytes[LEFT_SHOULDER_YAW] =90;
            command_bytes[HEAD_PITCH] = 90;
            command_bytes[HEAD_YAW] = 90;
            command_bytes[RIGHT_WRIST] = 90;
            command_bytes[LEFT_WRIST] = 90;
            //command_bytes[RIGHT_GRIPPER] = 0;
            command_bytes[RIGHT_ELBOW_PITCH] = 55;
            command_bytes[RIGHT_ELBOW_YAW] = 90;
            command_bytes[RIGHT_SHOULDER_PITCH] = 0;
            command_bytes[RIGHT_SHOULDER_YAW] = 90;
            command_bytes[RIGHT_WHEEL] = ConvertSignedIntToByte(0);
            command_bytes[LEFT_WHEEL] = ConvertSignedIntToByte(0);
            // Kinect sensor initialization
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();
            }

            _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color |
                                              FrameSourceTypes.Depth |
                                              FrameSourceTypes.Infrared |
                                              FrameSourceTypes.Body);
            _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            // Instatiate this class
            new SerialPortProgram();
            while (true) {
                try
                {
                    command_bytes[RIGHT_SHOULDER_PITCH] = (byte)robotJoints[RIGHT_SHOULDER_PITCH].Angle;
                    command_bytes[RIGHT_SHOULDER_YAW] = (byte)robotJoints[RIGHT_SHOULDER_YAW].Angle;
                    command_bytes[RIGHT_ELBOW_YAW] = (byte)robotJoints[RIGHT_ELBOW_YAW].Angle;
                    command_bytes[RIGHT_ELBOW_PITCH] = (byte)robotJoints[RIGHT_ELBOW_PITCH].Angle;

                    command_bytes[LEFT_SHOULDER_PITCH] = (byte)robotJoints[LEFT_SHOULDER_PITCH].Angle;
                    command_bytes[LEFT_SHOULDER_YAW] = (byte)robotJoints[LEFT_SHOULDER_YAW].Angle;
                    command_bytes[LEFT_ELBOW_YAW] = (byte)robotJoints[LEFT_ELBOW_YAW].Angle;
                    command_bytes[LEFT_ELBOW_PITCH] = (byte)robotJoints[LEFT_ELBOW_PITCH].Angle;
                    command_bytes[RIGHT_WRIST] = (byte)(180-robotJoints[RIGHT_ELBOW_YAW].Angle - 90 < 0 ? 0 : 180 - robotJoints[RIGHT_ELBOW_YAW].Angle - 90);//robotJoints[RIGHT_WRIST].Angle;
                    command_bytes[LEFT_WRIST] = (byte)(180-robotJoints[LEFT_ELBOW_YAW].Angle + 90);//robotJoints[RIGHT_WRIST].Angle;
                    //command_bytes[LEFT_GRIPPER] = (byte)robotJoints[LEFT_GRIPPER].Angle;
                    //command_bytes[RIGHT_WHEEL] = ConvertSignedIntToByte(right_wheel_speed);
                    //command_bytes[LEFT_WHEEL] = ConvertSignedIntToByte(left_wheel_speed);
                    //command_bytes[RIGHT_GRIPPER] = (byte)robotJoints[RIGHT_GRIPPER].Angle;
                    //Console.Write(
                        //"\rRWS:" + ((int)command_bytes[RIGHT_WHEEL] - 127)
                        //+ "\tLSP:" + ((int)command_bytes[LEFT_WHEEL] -127)
                        //+ "\tLX:" + String.Format("{0:0.000}%",lean_x)
                        //+ "\tLY:" + String.Format("{0:0.000}%", lean_y)
                    //    + "\tREY:" + (int)command_bytes[RIGHT_ELBOW_YAW]
                    //    + "\tREP:" + (int)command_bytes[RIGHT_ELBOW_PITCH]
                    //    + "\tLSP:" + (int)command_bytes[LEFT_SHOULDER_PITCH]
                    //    + "\tLSY:" + (int)command_bytes[LEFT_SHOULDER_YAW]
                    //    + "\tLEY:" + (int)command_bytes[LEFT_ELBOW_YAW]
                    //    + "\tLEP:" + (int)command_bytes[LEFT_ELBOW_PITCH]
                    //    + "\t");
                    port.WriteLine("\\x" + BitConverter.ToString(command_bytes).Replace("-","\\x"));
                    //Thread.Sleep(250);
                }
                catch (Exception e)
                {
                    //Console.Write("\r" + e.Message + "\t\t\t\t");
                    port.Close();
                    port.Open();
                }
            }
        }

        private SerialPortProgram()
        {
            Console.WriteLine("Incoming Data:");
            port.Handshake = Handshake.XOnXOff;
            port.RtsEnable = true;
            port.DtrEnable = true;
            port.NewLine = "\r";
            port.Encoding = Encoding.UTF8;
            port.WriteTimeout = 500;
            port.ReadTimeout = 500;

            // Attach a method to be called when there
            // is data waiting in the port's buffer
            //port.DataReceived += new
            //  SerialDataReceivedEventHandler(port_DataReceived);

            try {
                // Begin communications
                
                port.Open();
            }
            catch (System.IO.IOException e){
                Console.WriteLine(e.Message);
                Console.WriteLine(e.StackTrace);
                Console.WriteLine("Could not complete serial connection to headset. Please check connection. Press any key to exit.");
                Console.ReadKey();
                Environment.Exit(1);
            }




        }

        private void port_DataReceived(object sender,
          SerialDataReceivedEventArgs e)
        {
            // Show all the incoming data in the port's buffer
            //Console.Write(port.ReadExisting());
        }
    }
}
