#include "cor_tra_gen.h"

//map
map::map()
{
	num_of_obs = 0;
	start_pos = Point2d(0, 0);
	end_pos = Point2d(0, 0);
	map_size[0] = 0;
	map_size[1] = 0;
}

map::~map()
{
}

void map::add_obs(int s, int x,int y)
{
	obs_size.push_back(s);
	obs_location.push_back(Point2d(x, y));
	num_of_obs++;
}

int map::get_num_of_obs()
{
	return num_of_obs;
}

int map::get_size_of_obsi(int i)
{
	return obs_size[i];
}

int map::get_x_of_obsi(int i)
{
	return obs_location[i].x;
}

int map::get_y_of_obsi(int i)
{
	return obs_location[i].y;
}

void map::draw()
{
	Mat _map(map_size[0], map_size[1], CV_8UC3, Scalar(255, 255, 255));
	for (int i = 0;i < num_of_obs;++i)
	{
		rectangle(_map, Point(obs_location[i].x - obs_size[i], obs_location[i].y - obs_size[i]),
			Point(obs_location[i].x + obs_size[i], obs_location[i].y + obs_size[i]),
			Scalar(0, 0, 0), CV_FILLED, 8, 0);
	}
	circle(_map, start_pos, 8, Scalar(255, 0, 0), CV_FILLED);
	circle(_map, end_pos, 8, Scalar(0, 255, 0), CV_FILLED);
    for (int i = 0;i < waypoint.size()-1;++i)
    {
        circle(_map, waypoint[i], 5, Scalar(0, 0, 255), CV_FILLED);
        line(_map, waypoint[i], waypoint[i + 1], Scalar(100, 100, 100), 3);
    }
	imshow("result", _map);
	cvWaitKey(0);
}

void map::set_map_size(int height, int width)
{
	map_size[0] = height;
	map_size[1] = width;
}

void map::set_startend_pos(int *startendxy)
{
	start_pos.x = startendxy[0];
	start_pos.y = startendxy[1];
	end_pos.x = startendxy[2];
	end_pos.y = startendxy[3];
    waypoint.push_back(start_pos);
}

bool map::generate_trajectory()
{
	if (!generate_waypoint(start_pos, end_pos))
		return 0;

	return 1;
}

bool map::generate_waypoint(Point2d s, Point2d e)
{
    int obs_ahead;
    if (!obs_exist(s, e, obs_ahead))
    {
        waypoint.push_back(e);
        return 1;
    }

    //pass obstacle i
    Point2d midp[2];    
    if (pass_obsi(s, e, obs_ahead, midp) && generate_waypoint(s, midp[0]))
    {
        if (abs(midp[0].x - midp[1].x + midp[0].y - midp[1].y) > 0.01)
            waypoint.push_back(midp[1]);

        return generate_waypoint(midp[1], e);
    }
        

	return 0;
}

bool map::obs_exist(Point2d s, Point2d e, int &obs_num)
{
	obs_num = -1;
    if (num_of_obs == 0)
        return 0;
	//direction of e-s
	float direx = e.x - s.x, direy = e.y - s.y,
		abs_dire = sqrt(direx*direx + direy*direy),
		nearest_pos = 10000, temp;
	//cout << "abs_dire is " << abs_dire << endl;
	//calculate distance between obs and line
	for (int i = 0;i < num_of_obs; ++i)
	{
		//cout << "the " << i << "th round is" << 1.42*obs_size[i] << "and the dist is ";
		if (distance_pointline(s, e, obs_location[i]) < 1.42*obs_size[i])
		{
			temp = ((obs_location[i].x - s.x)*direx + (obs_location[i].y - s.y)*direy) / abs_dire;
			//cout << "ͶӰ is" << temp << endl;
			if (temp > 0 && temp < abs_dire && temp < nearest_pos)
			{
				nearest_pos = temp;
				obs_num = i;
			}
		}
	}

	if (obs_num == -1)
		return 0;
	else
		return 1;
}

float map::distance_pointline(Point2d s, Point2d e, Point2d obs)
{
	//ax+by+c=0
	float a, b, c;
	if (abs(s.x - e.x) < 0.001)
	{
		//if points are the same, no line
		if (abs(s.y - e.y) < 0.001)
			return -1;

		b = 0;
		a = 1;
		c = -s.x;
	}
	else if (abs(s.y - e.y) < 0.001)
	{
		a = 0;
		b = 1;
		c = -s.y;
	}
	else
	{
		a = 1;
		b = (e.x - s.x) / (s.y - e.y);
		c = -s.x - b*s.y;
	}
	/*
	cout << abs(a*obs.x + b*obs.y + c) / sqrt(a*a + b*b) << " and line is "<<
		a << "x+" << b << "y+" << c << "=0 from two points" << s.x << " " << s.y <<
		" " << e.x << " " << e.y << endl;
	*/
	//|ax+by+c|/sqrt(a*a+b*b)
	return abs(a*obs.x + b*obs.y + c) / sqrt(a*a + b*b);
}

float map::distance_pointpoint(Point2d s, Point2d e)
{
    return sqrt(pow(s.x - e.x, 2) + pow(e.y - s.y, 2));
}

bool map::pass_obsi(Point2d s, Point2d e, int obs_num, Point2d *b)
{
    //eight position for start and end points
    /*
    1,2,3
    4,o,5
    6,7,8
    */
    if (s.x > (obs_location[obs_num].x - obs_size[obs_num]))
    {
        if (s.x < (obs_location[obs_num].x + obs_size[obs_num]))
        {
            //start at 2
            if (s.y < (obs_location[obs_num].y - obs_size[obs_num]))
            {
                //end at 5,8
                if (e.x > (obs_location[obs_num].x + obs_size[obs_num]))
                {
                    b[0] = Point2d(obs_location[obs_num].x + 1.45*obs_size[obs_num], obs_location[obs_num].y - 1.45*obs_size[obs_num]);
                    b[1] = b[0];return 1;
                }
                //end at 4,6
                else if (e.x < (obs_location[obs_num].x - obs_size[obs_num]))
                {
                    b[0] = Point2d(obs_location[obs_num].x - 1.45*obs_size[obs_num], obs_location[obs_num].y - 1.45*obs_size[obs_num]);
                    b[1] = b[0];return 1;
                }
                //end at 7
                else
                {
                    if (s.x > obs_location[obs_num].x)
                    {
                        b[0] = Point2d(obs_location[obs_num].x + 1.45*obs_size[obs_num], obs_location[obs_num].y - 1.45*obs_size[obs_num]);
                        b[1] = Point2d(obs_location[obs_num].x + 1.45*obs_size[obs_num], obs_location[obs_num].y + 1.45*obs_size[obs_num]);
                        return 1;
                    }
                    else
                    {
                        b[0] = Point2d(obs_location[obs_num].x - 1.45*obs_size[obs_num], obs_location[obs_num].y - 1.45*obs_size[obs_num]);
                        b[1] = Point2d(obs_location[obs_num].x - 1.45*obs_size[obs_num], obs_location[obs_num].y + 1.45*obs_size[obs_num]);
                        return 1;
                    }
                }
            }
        }
    }
    else
    {
        //start at 1
        if(s.y < (obs_location[obs_num].y - obs_size[obs_num]))
        {
            //end at 5,8
            if ((e.x-obs_location[obs_num].x) > (e.y-obs_location[obs_num].y))
            {
                b[0] = Point2d(obs_location[obs_num].x + 1.45*obs_size[obs_num], obs_location[obs_num].y - 1.45*obs_size[obs_num]);
                b[1] = b[0];return 1;
            }
            //end at 7,8
            else
            {
                b[0] = Point2d(obs_location[obs_num].x - 1.45*obs_size[obs_num], obs_location[obs_num].y + 1.45*obs_size[obs_num]);
                b[1] = b[0];return 1;
            }
        }
        //start at 4
        else if(s.y < (obs_location[obs_num].y + obs_size[obs_num]))
        {
            //end at 2,3
            if (e.y < (obs_location[obs_num].y - obs_size[obs_num]))
            {
                b[0] = Point2d(obs_location[obs_num].x - 1.45*obs_size[obs_num], obs_location[obs_num].y - 1.45*obs_size[obs_num]);
                b[1] = b[0];return 1;
            }
            //end at 7,8
            else if (e.y > (obs_location[obs_num].y + obs_size[obs_num]))
            {
                b[0] = Point2d(obs_location[obs_num].x - 1.45*obs_size[obs_num], obs_location[obs_num].y + 1.45*obs_size[obs_num]);
                b[1] = b[0];return 1;
            }
            //end at 5
            else
            {
                if (s.y < obs_location[obs_num].y)
                {
                    b[0] = Point2d(obs_location[obs_num].x - 1.45*obs_size[obs_num], obs_location[obs_num].y - 1.45*obs_size[obs_num]);
                    b[1] = Point2d(obs_location[obs_num].x + 1.45*obs_size[obs_num], obs_location[obs_num].y - 1.45*obs_size[obs_num]);
                    return 1;
                }
                else
                {
                    b[0] = Point2d(obs_location[obs_num].x - 1.45*obs_size[obs_num], obs_location[obs_num].y + 1.45*obs_size[obs_num]);
                    b[1] = Point2d(obs_location[obs_num].x + 1.45*obs_size[obs_num], obs_location[obs_num].y + 1.45*obs_size[obs_num]);
                    return 1;
                }
            }
        }
    }

    return 0;
}

void map::sort_obs()
{

}