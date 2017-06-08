#pragma once

#include <stdio.h>  
#include <stdlib.h>  
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.hpp>
using namespace std;
using namespace cv;

class map
{
public:
    map();
    ~map();
    void add_obs(int s, int x, int y);
    void set_map_size(int height,int width);
    void set_startend_pos(int *startendxy);
    int get_num_of_obs();
    int get_size_of_obsi(int i);
    int get_x_of_obsi(int i);
    int get_y_of_obsi(int i);
    bool generate_trajectory();
    void draw();


private:
    vector<int> obs_size;
    vector<Point2d> obs_location;
    vector<Point2d> waypoint;
    int num_of_obs;
    int map_size[2];    
    Point2d start_pos;
    Point2d end_pos;
    
    bool generate_waypoint(Point2d s, Point2d e);
    bool obs_exist(Point2d s, Point2d e, int &obs_num);
    float distance_pointline(Point2d s, Point2d e, Point2d obs);
    float distance_pointpoint(Point2d s, Point2d e);
    bool pass_obsi(Point2d s, Point2d e, int obs_num, Point2d *b);
    void sort_obs();
};

