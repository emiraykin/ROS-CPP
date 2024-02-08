#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <maze_solver/Maze.h>

using namespace std;

struct Cell {
    int x, y;
    bool visited;
    bool obstacle;

    Cell(int x_, int y_) : x(x_), y(y_), visited(false), obstacle(false) {}
};

class MazeSolver {
private:
    ros::NodeHandle nh_;
    ros::Publisher solution_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber maze_sub_;

    vector<vector<Cell>> maze_;
    stack<Cell> path_;

public:
    MazeSolver() : nh_("~") {
        solution_pub_ = nh_.advertise<std_msgs::String>("maze_solution", 1);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        maze_sub_ = nh_.subscribe("maze", 1, &MazeSolver::mazeCallback, this);
    }

    void mazeCallback(const maze_solver::Maze::ConstPtr& msg) {
        int rows = msg->rows;
        int cols = msg->cols;
        maze_.resize(rows, vector<Cell>(cols, Cell(0, 0)));

        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                maze_[i][j].x = i;
                maze_[i][j].y = j;
                maze_[i][j].obstacle = msg->data[i * cols + j];
            }
        }

        if (solveMaze(0, 0)) {
            publishSolution();
        } else {
            ROS_WARN("No solution found!");
        }
    }

    bool solveMaze(int x, int y) {
        if (x < 0 || x >= maze_.size() || y < 0 || y >= maze_[0].size() || maze_[x][y].obstacle || maze_[x][y].visited)
            return false;

        maze_[x][y].visited = true;
        path_.push(maze_[x][y]);

        if (x == maze_.size() - 1 && y == maze_[0].size() - 1)
            return true;

        if (solveMaze(x + 1, y) || solveMaze(x, y + 1) || solveMaze(x - 1, y) || solveMaze(x, y - 1))
            return true;

        path_.pop();
        return false;
    }

    void publishSolution() {
        string solution;
        while (!path_.empty()) {
            Cell cell = path_.top();
            path_.pop();
            solution += "(" + to_string(cell.x) + ", " + to_string(cell.y) + ") -> ";
        }

        std_msgs::String msg;
        msg.data = solution;
        solution_pub_.publish(msg);

        // Move the robot out of the maze using cmd_vel
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.5; // Forward speed
        cmd_vel.angular.z = 0.5; // Angular speed (turning)
        cmd_vel_pub_.publish(cmd_vel);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "maze_solver_node");
    MazeSolver maze_solver;
    ros::spin();
    return 0;
}
