#include <Explorer.h>

Explorer::Explorer() : n_for_params("~")
{
    client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
    service = n.advertiseService("get_goal_at_map", &Explorer::callback, this);

    n_for_params.param<int>("region_size", region_size, 21);
    n_for_params.param<double>("initial_radius", initial_radius, 1.5);
    n_for_params.param<double>("white_perc", white_perc, 40);
    n_for_params.param<double>("grey_perc", grey_perc, 45);
    n_for_params.param<double>("black_perc", black_perc, 10);
    n_for_params.param<double>("delta", delta, 0.1);
}



Explorer::~Explorer()
{
    client.shutdown();
    service.shutdown();
}



void Explorer::start_work()
{
    client.waitForExistence();
    ros::spin();
}



bool Explorer::callback(box_finder::GetGoal::Request &req,
                        box_finder::GetGoal::Response &res)
{
    nav_msgs::GetMap srv;
    if (client.call(srv))
        map = srv.response.map;
    else
    {
        ROS_ERROR("Failed to call server dynamic_map");
        ros::shutdown();
    }

    // robot's (j, i)-coordinates
    int cur_j = ROUND((req.robot_pose.x - map.info.origin.position.x) / map.info.resolution);
    int cur_i = ROUND((req.robot_pose.y - map.info.origin.position.y) / map.info.resolution);

    int init_radius_in_cells = ROUND(initial_radius / map.info.resolution);
    init_radius_in_cells = (init_radius_in_cells%2) ? init_radius_in_cells : (init_radius_in_cells + 1);

    // move "scanner" in the "bottom-left" corner
    int exp_j = cur_j - std::floor(init_radius_in_cells / 2);
    int exp_i = cur_i - std::floor(init_radius_in_cells / 2) - 1;

    int k = 0;
    int m = init_radius_in_cells; // ---> j-axes increment/decrement
    int n = init_radius_in_cells + 1; // ---> i-axes increment/decrement

    int half = std::floor(region_size / 2);

    goto right;

    while(((exp_j - half >= 0) && (exp_j + half < map.info.width))  ||
         ((exp_i - n - half >= 0) && (exp_i - n + half < map.info.height)) ||
         ((exp_j - half + m >= 0) && (exp_j + m + half < map.info.width)) ||
         ((exp_i - half + 1 >= 0) && (exp_i + half + 1 < map.info.height)))
    {
        // going down
        if((exp_j - half >= 0) && (exp_j + half < map.info.width))
        {
            for(k = 1, n++; k <= n; k += 1)
            {
                // j coordinate remains the same
                exp_i = exp_i - 1;
                if (((exp_i - half) < 0) || ((exp_i + half) >= map.info.height)) continue;
                if(processRegion(exp_i, exp_j))
                {
                    double x = x_from_j(exp_j);
                    double y = y_from_i(exp_i);
                    int z, size;
                    for (z = 0, size = req.excluded_goals.size(); z < size; z++)
                    {
                        if ((std::pow((x - req.excluded_goals[z].x), 2) +
                             std::pow((y - req.excluded_goals[z].y), 2)) < std::pow(delta, 2)) break;
                    }
                    if(z == size)
                    {
                        res.goal.x = x;
                        res.goal.y = y;
                        return true;
                    }
                }
            }
        }
        else
        {
            n++;
            exp_i -= n;
        }

        right:
        // going right
        if((exp_i - half >= 0) && (exp_i + half < map.info.height))
        {
            for(k = 1, m++; k <= m; k += 1)
            {
                // i coordinate remains the same
                exp_j = exp_j + 1;
                if (((exp_j - half) < 0) || ((exp_j + half) >= map.info.width)) continue;
                if(processRegion(exp_i, exp_j))
                {
                    double x = x_from_j(exp_j);
                    double y = y_from_i(exp_i);
                    int z, size;
                    for (z = 0, size = req.excluded_goals.size(); z < size; z++)
                    {
                        if ((std::pow((x - req.excluded_goals[z].x), 2) +
                            std::pow((y - req.excluded_goals[z].y), 2)) < std::pow(delta, 2)) break;
                    }
                    if(z == size)
                    {
                        res.goal.x = x;
                        res.goal.y = y;
                        return true;
                    }
                }
            }
        }
        else
        {
            m++;
            exp_j += m;
        }

        // going up
        if((exp_j - half >= 0) && (exp_j + half < map.info.width))
        {
            for(k = 1, n++; k <= n; k += 1)
            {
                exp_i = exp_i + 1;
                if (((exp_i - half) < 0) || ((exp_i + half) >= map.info.height)) continue;
                if(processRegion(exp_i, exp_j))
                {
                    double x = x_from_j(exp_j);
                    double y = y_from_i(exp_i);
                    int z, size;
                    for (z = 0, size = req.excluded_goals.size(); z < size; z++)
                    {
                        if ((std::pow((x - req.excluded_goals[z].x), 2) +
                            std::pow((y - req.excluded_goals[z].y), 2)) < std::pow(delta, 2)) break;
                    }
                    if(z == size)
                    {
                        res.goal.x = x;
                        res.goal.y = y;
                        return true;
                    }
                }
            }
        }
        else
        {
            n++;
            exp_i += n;
        }

        // going left
        if((exp_i - half >= 0) && (exp_i + half < map.info.height))
        {
            for(k = 1, m++; k <= m; k += 1)
            {
                exp_j = exp_j - 1;
                if (((exp_j - half) < 0) || ((exp_j + half) >= map.info.width)) continue;
                if(processRegion(exp_i, exp_j))
                {
                    double x = x_from_j(exp_j);
                    double y = y_from_i(exp_i);
                    int z, size;
                    for (z = 0, size = req.excluded_goals.size(); z < size; z++)
                    {
                        if ((std::pow((x - req.excluded_goals[z].x), 2) +
                            std::pow((y - req.excluded_goals[z].y), 2)) < std::pow(delta, 2)) break;
                    }
                    if(z == size)
                    {
                        res.goal.x = x;
                        res.goal.y = y;
                        return true;
                    }
                }
            }
        }
        else
        {
            m++;
            exp_j -= m;
        }

    }

    //if no goal found:
    res.goal.x = req.robot_pose.x;
    res.goal.y = req.robot_pose.y;
    return true;
}



double Explorer::x_from_j(int j)
{
        return map.info.resolution * j + map.info.origin.position.x;
}



double Explorer::y_from_i(int i)
{
        return map.info.resolution * i + map.info.origin.position.y;
}



bool Explorer::processRegion(int i, int j)
{
    // FUNCTION THAT PROCESSES region_sizeXregion_size REGION AND RETURNS TRUE IF IT'S A FRONTIER
    /**************************************************************************/

    // variables that contain an amount of occupied, free and unknown cells
    int free = 0;
    int occupied = 0;
    int unknown = 0;

    // new indices
    int w;
    int h;

    // total number of cells in the region
    region_size = (region_size%2) ? region_size : (region_size + 1);
    int total = region_size * region_size;

    int reg_half = std::floor(region_size / 2);

    for(h = i - reg_half; h <= i + reg_half; h++)
    {
        for(w = j - reg_half; w <= j + reg_half; w++)
        {
            switch(map.data[w + (map.info.width)*h])
            {
                case 100:
                        occupied++;
                        break;
                case 0:
                        free++;
                        break;
                case -1:
                        unknown++;
                        break;
                default:
                        ROS_WARN("THIS CELL IS NEITHER [0, 100], NOR IT IS -1 [%d; %d]", w, h);
            }
        }
    }

    // counting number of required black, white, grey cells
    int white_num = (white_perc  / 100) * total;
    int grey_num =  (grey_perc   / 100) * total;
    int black_num = (black_perc  / 100) * total;

    if (occupied > black_num)
    {
        return false;
    }
    else if ((free >= white_num) && (unknown >= grey_num))
    {
        return true;
    }
    return false;
}
