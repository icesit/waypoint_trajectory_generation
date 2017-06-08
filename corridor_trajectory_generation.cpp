// corridor_trajectory_generation.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "cor_tra_gen.h"
#include <stdio.h>  
#include <stdlib.h>  
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main()
{
	ifstream myMap("D:\\ProgramData\\vs2015\\corridor_trajectory_generation\\Debug\\map.txt");
	if (!myMap) {
		cout << "no such file!" << endl;
		system("pause");
		return 0;
	}

	map _ob;
	string buffer;
	stringstream ss;

	//get map size
	getline(myMap, buffer);
	ss << buffer;
	int width = 0, height = 0;
	ss >> width;
	ss >> height;
	_ob.set_map_size(height, width);
	buffer.clear();
	ss.clear();
	cout << "width is:" << width << ", height is:" << height << endl;

	//get map start and end point
	getline(myMap, buffer);
	ss << buffer;
	int start_end[4] = { 0, 0, 0, 0 };
	for (int i = 0;i < 4;++i) {
		ss >> start_end[i];
		cout << "start end points are:" << start_end[i] << endl;
	}
	_ob.set_startend_pos(start_end);
	buffer.clear();
	ss.clear();

	//get obstacles
	int s = 0, x = 0, y = 0;
	
	while (getline(myMap, buffer)) {
		ss << buffer;
		ss >> x;
		ss >> y;
		ss >> s;
		_ob.add_obs(s, x, y);
		cout << "size is:" << s << ", x is:" << x << ", y is:" << y << endl;
		buffer.clear();
		ss.clear();
	}
	myMap.close();

    _ob.generate_trajectory();
	//draw the map
	_ob.draw();

	system("pause");
    return 0;
}

