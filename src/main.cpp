#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <cmath>

using namespace std;

ros::Publisher publisher;

float gridResolution = 0.5f;
int gridSize = 33;
int grid[33][33];
int robotPositionX = 17;
int robotPositionY = 17;

void createGrid()
{
    for (int i = 0; i < gridSize; i++)
        for (int j = 0; j < gridSize; j++)
            grid[i][j] = 0;
    
    grid[17][17] = 2;
}

void displayGrid()
{
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++)
            cout << grid[i][j] << " ";

        cout << endl;
    }
}

void sendGrid() 
{
    vector<int> gridVector;

    for (int i = gridSize - 1; i >= 0; i--) {
        for (int j = 0; j < gridSize; j++)
            gridVector.push_back(grid[i][j]);
    }

    std_msgs::Int32MultiArray msg;
    msg.data = gridVector;

    ROS_INFO("Publishing grid in topic robot_grid");
    publisher.publish(msg);
}

void bresenhamLine(int finalPositionX, int finalPositionY)
{
    int initialPositionX = 5;
    int initialPositionY = 5;

    bool step = abs(finalPositionY - initialPositionY) > abs(finalPositionX - initialPositionX);
    if (step) {
        swap(initialPositionX, initialPositionY);
        swap(finalPositionX, finalPositionY);
    }

    bool swapped = false;
    if (initialPositionX > finalPositionX) {
        swap(initialPositionX, finalPositionX);
        swap(initialPositionY, finalPositionY);
        swapped = true;
    }

    int deltaX = finalPositionX - initialPositionX;
    int deltaY = abs(finalPositionY - initialPositionY);
    int error = deltaX / 2;

    int yStep = (initialPositionY < finalPositionY) ? 1 : -1;
    int y = initialPositionY;

    for (int x = initialPositionX; x <= finalPositionX; ++x) {
        if ((x >= 0 && x < 11 && y >= 0 && y < 11) && grid[step ? y : x][step ? x : y] == 0) {
            grid[step ? y : x][step ? x : y] = 0;
        }

        error -= deltaY;
        if (error < 0) {
            y += yStep;
            error += deltaX;
        }
    }
}

void setLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    createGrid();
    ROS_INFO("I heard LaserScan");

    for (int i = 0; i < msg->ranges.size(); i++) {
        float range = isinf(msg->ranges[i]) ? msg->range_max : msg->ranges[i];

        if (range < msg->range_min || range > msg->range_max)
            continue;

        float angle = msg->angle_min + i * msg->angle_increment;
        float pointX = robotPositionX + range / gridResolution * cos(angle);
        float pointY = robotPositionY + range / gridResolution * sin(angle);


        int gridX = floor(pointX);
        int gridY = floor(pointY);
    
        bresenhamLine(gridX, gridY);

        if (isinf(msg->ranges[i]) || gridX < 0 || gridX >= gridSize || gridY < 0 || gridY >= gridSize)
            continue;

        if (grid[gridX][gridY] == 2) {
            ROS_INFO("Robot reached destination");
            continue;
        }

        grid[gridX][gridY] = 1;
    }

    displayGrid();
    sendGrid();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "create_grid");
    ros::NodeHandle nh;

    createGrid();

    publisher = nh.advertise<std_msgs::Int32MultiArray>("robot_grid", 1);
    ros::Subscriber scanSub = nh.subscribe("l1br/scan", 1, setLaserScan);

    ros::Rate loop_rate(0.1);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}