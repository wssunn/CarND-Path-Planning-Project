
/**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

//staright motion
// double dist_inc = 0.5;
// for (int i = 0; i < 50; ++i)
// {
//   next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
//   next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
// }

//circular path
//starts the new path with whatever previous path points were left over from the last cycle.
//Then we append new waypoints, until the new path has 50 total waypoints
// double pos_x;
// double pos_y;
// double angle;
// int path_size = previous_path_x.size();

// for (int i = 0; i < path_size; ++i)
// {
//   next_x_vals.push_back(previous_path_x[i]);
//   next_y_vals.push_back(previous_path_y[i]);
// }

// if (path_size == 0)
// {
//   pos_x = car_x;
//   pos_y = car_y;
//   angle = deg2rad(car_yaw);
// }
// else
// {
//   pos_x = previous_path_x[path_size - 1];
//   pos_y = previous_path_y[path_size - 1];

//   double pos_x2 = previous_path_x[path_size - 2];
//   double pos_y2 = previous_path_y[path_size - 2];
//   angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
// }

// double dist_inc = 0.5;
// for (int i = 0; i < 50 - path_size; ++i)
// {
//   next_x_vals.push_back(pos_x + (dist_inc)*cos(angle + (i + 1) * (pi() / 100)));
//   next_y_vals.push_back(pos_y + (dist_inc)*sin(angle + (i + 1) * (pi() / 100)));
//   pos_x += (dist_inc)*cos(angle + (i + 1) * (pi() / 100));
//   pos_y += (dist_inc)*sin(angle + (i + 1) * (pi() / 100));
// }