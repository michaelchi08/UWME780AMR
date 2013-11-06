// Code controller Planning Lab
// Michael Chi 
//Listen to arrayposemsg;
// Date:6/11/13

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
//#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <turtlebot_example/ips_msg.h>
#include <cmath>
ros::Publisher marker_pub;

// Global variables
double X;
double Y;
double Yaw;// Heading

int currentwaypt;// keep track of the current waypoint list provided by the RRT planner

#define PI 3.14159

//  Waypoints for the simulated path planning

#define NumWaypoints 3
#define WP_Tolerance 0.1 // Tolerance around the waypoint
#define MAXINDEX 1

int Numberofwaypoints;
double dt = 1/20.;
double Ux = 0;
double prevUx = 0;
int Index=1;
// pose callback
#define TAGID 6

void pose_callback(const turtlebot_example::ips_msg& msg)
{
    //This function is called when a new position message is received
    if(msg.tag_id != TAGID) {
            return;
    }
    printf("GOT new info\n");

    X = msg.X; // Robot X position
    Y = msg.Y; // Robot Y position
    Yaw = msg.Yaw; // Robot Yaw
}
    

 double *Waypts_x;
 double *Waypts_y;
 //double *Waypts_yaw;

void getwaypts_callback(const geometry_msgs::PoseArray msg) 
 {
    // This function is called whenever new waypoints are available from the RRT planner message is received 
    int Numberofwaypoints = (sizeof(msg.poses)/sizeof(msg.poses[0]));
    Numberofwaypoints= msg.poses.size();
    double* updatedWaypts_x = new double [Numberofwaypoints];
    double* updatedWaypts_y = new double [Numberofwaypoints];
    
    for(int i=0;i<Numberofwaypoints-1;i++)
    {
        updatedWaypts_x[i] = msg.poses[i].position.x; //Array of pose X waypoints
        updatedWaypts_y[i] = msg.poses[i].position.y;//Array of pose Y waypoints
    }
    Waypts_x = (double*) malloc (Numberofwaypoints*sizeof(double));
    Waypts_y = (double*) malloc (Numberofwaypoints*sizeof(double));

    memcpy (Waypts_x, updatedWaypts_x, Numberofwaypoints*sizeof(double));
    memcpy (Waypts_y, updatedWaypts_y, Numberofwaypoints*sizeof(double));    
 }

double crosstrack_error(double x0,double y0, double x1, double y1)
{
    double alpha = atan2((y1-y0),(x1-x0));
    //return the Y lat error 
    return (sin(-alpha)*(X-x0) + cos(-alpha)*(Y-y0));
}

// This function returns the Yawerror  
double heading_error(double robotheading, double roadHeading, double CTE)
{
    
    printf ("Robot heading is %f\n",robotheading);

    double lookahead = 0.2; // look-ahead distance
    robotheading= roadHeading - robotheading;
    double Yawdesired= atan2(CTE,lookahead);
    double Yawerror = robotheading-Yawdesired;

    if (Yawerror > PI)
    {
        Yawerror = 2*PI - Yawerror;
    }
    else if (Yawerror < -PI)
    {
        Yawerror = 2*PI + Yawerror;
    }
    printf ("Robot heading is %f\n",robotheading);
    printf ("Roadheading is %f\n",roadHeading);
    printf ("Desired yaw is %f\n",Yawdesired);
    printf ("CTE is %f\n",CTE);
    printf ("Yaw error is %f\n",Yawerror);



    return Yawerror;
}

double longControl (double desiredUx, double Ux)
{
    double K = 1;
    return K*(desiredUx-Ux);
}
double latControl (double Yawerror, double dYawerror)
{
    double Kp = 1; // proportional term was 0.5
    double kd = 0.2; // derivative term
    return Kp * (Yawerror)+kd*dYawerror;
}

// Generating a circle
double *Waypts_xcircle;
double *Waypts_ycircle;
double *Waypts_angle;
 

void circle(double x,double y, double radius,int numpointscircle)
{   
    Waypts_ycircle = new double [numpointscircle];
    Waypts_xcircle = new double [numpointscircle];

    double angle = 0;

    for (int i = 0; i < (numpointscircle); i++) 
    {
            Waypts_xcircle[i] = radius * cos(angle) + x;
            Waypts_ycircle[i] = radius * sin(angle) + y;
            printf ("Waypts_xcircle is %f\n",Waypts_xcircle[i]);
            printf ("Waypts_ycircle is %f\n",Waypts_ycircle[i]);
            angle += 2*PI/(numpointscircle-1);
    }
}

// Main function
int main(int argc, char **argv)
{

double der_Yawerror = 0;
double prev_Yawerror = 0;
currentwaypt=0;


    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 3, pose_callback);
    //ros::Subscriber poseArray_sub = n.subscribe("waypointPoses",3,getwaypts_callback);// name of the message needs to included   
    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(1/dt);    //20Hz update rate


    double x0 = 0; 
    double y0 = 0; 
    double x1 = 0; 
    double y1 = 0; 


    double crossTrack = crosstrack_error(x0,y0,x1,y1);
    double roadHeading = atan2((y1-y0),(x1-x0));
    double prev_headingerror = 0;
    double der_headingerror = 0;
    double deisredUx = 0.2; 

    Numberofwaypoints = 200;
    circle(-0.2,2,2.5,Numberofwaypoints);
    bool StopFlag = false;

    while (ros::ok())// && (currentwaypt <= NumWaypoints-1))
    {


        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
        
        if (Index <= Numberofwaypoints)
        {
            x0= Waypts_xcircle[Index-1];
            y0= Waypts_ycircle[Index-1];
            x1= Waypts_xcircle[Index];
            y1= Waypts_ycircle[Index];
        }

        printf ("X: %f, Y: %f, Yaw: %f \n",X,Y,Yaw); 
        crossTrack = crosstrack_error(x0,y0,x1,y1);
        roadHeading = atan2((y1-y0),(x1-x0));
        printf ("Road heading is %f\n",roadHeading);


        double headingError = heading_error(Yaw,roadHeading, crossTrack);

        der_headingerror= (headingError-prev_headingerror)/dt;
        printf("GOT TO %f, %f, GOING TO %f, %f\n",x0,y0,x1,y1);

        if (sqrt(pow(X-x1,2)+pow(Y-y1,2)) <= 0.25)
        {

            Index++;
            if (Index >=  (Numberofwaypoints))
            {
                StopFlag = true;
                printf("size = %d\n",Index );
            }
        }
        if (StopFlag)
        {       
            printf("Index = %d \n", Index);
            vel.linear.x = 0; //longControl(Ux, desiredUx); // set linear speed
            vel.angular.z = 0;
        }
        else
        {
            vel.linear.x = deisredUx; //longControl(Ux, desiredUx); // set linear speed
            vel.angular.z =latControl(headingError,der_headingerror); // set angular speed            
        }

        prev_headingerror=headingError;

        velocity_publisher.publish(vel); // Publish the command velocity
        printf ("Heading Error is %f\n",headingError);
        printf ("commanded angular velocity is %f\n",vel.angular.z);
    }
    ROS_INFO(" Turtlebot has completed its task! Can go home now!");
    return 0;
}
