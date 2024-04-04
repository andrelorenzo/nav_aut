#ifndef AREA_PATH_GENERATOR_HPP
#define AREA_PATH_GENERATOR_HPP

#include <vector>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <msg_srv_hunter/msg/polygon.hpp>


using namespace std::chrono_literals;


class AreaPathGenerator {
public:

    AreaPathGenerator(){};
    ~AreaPathGenerator(){};
    geometry_msgs::msg::PoseArray get_intermediate(const geometry_msgs::msg::Point origin, const geometry_msgs::msg::Point end, double step)
    {
        geometry_msgs::msg::PoseArray path;
        geometry_msgs::msg::Pose inter_pose;
        double dx = end.x - origin.x;
        double dy = end.y - origin.y;
        double dd = sqrt(abs(dx*dx) + abs(dy*dy));
        double angle = atan2(dy,dx);
        int n_steps = dd/step;
        tf2::Quaternion q;
        q.setRPY(0,0,(angle));
        q.normalize();
        for(int i = 0; i < n_steps;i++)
        {
            inter_pose.orientation = tf2::toMsg(q);
            inter_pose.position.x = origin.x + (i+1)*step*cos(angle);
            inter_pose.position.y = origin.y + (i+1)*step*sin(angle);
            inter_pose.position.z = 0;

            path.poses.push_back(inter_pose);
        }
        path.header.frame_id = "map";
        inter_pose.orientation = tf2::toMsg(q);
        inter_pose.position.x = end.x;
        inter_pose.position.y = end.y;
        inter_pose.position.z = 0;

        path.poses.push_back(inter_pose);
        
        return path;
    }
    geometry_msgs::msg::PoseArray get_area_path(const geometry_msgs::msg::PoseArray& poly, double step)
    {
        double dx = poly.poses[1].position.x - poly.poses[0].position.x;
        double dy =  poly.poses[1].position.y - poly.poses[0].position.y;
        double a = dy/dx;
        float b = -1.0;
        double c = poly.poses[0].position.y - a * poly.poses[0].position.x;
        float d_max = 0;
        int ind_max_d = 0;
        int last_ind = poly.poses.size()-1;
        for (int i = 2; i <= last_ind;i++)
        {
            double num = a*poly.poses[i].position.x + b*poly.poses[i].position.y + c;
            double den = sqrt(abs(a*a) + abs(b*b));
            double d = abs(num/den);
            if(d > d_max)
            {
                d_max = d;
                ind_max_d = i;
            }
        }
        dx = poly.poses[2].position.x - poly.poses[1].position.x;
        dy =  poly.poses[2].position.y - poly.poses[1].position.y;
        float a2 = dy/dx;
        double c1 = poly.poses[1].position.y - a * poly.poses[1].position.x;
        double c2 = poly.poses[ind_max_d].position.y - a * poly.poses[ind_max_d].position.x;
        geometry_msgs::msg::Point inter1;
        inter1.x = (c2-c1)/(a2-a);
        inter1.y = a2*inter1.x + c2;

        geometry_msgs::msg::Point inter2;

        dx = poly.poses[last_ind].position.x - poly.poses[0].position.x;
        dy =  poly.poses[last_ind].position.y - poly.poses[0].position.y;
        a2 = dy/dx;
        c1 = poly.poses[0].position.y - a * poly.poses[0].position.x;
        c2 = poly.poses[ind_max_d].position.y - a * poly.poses[ind_max_d].position.x;
        inter2.x = (c2-c1)/(a2-a);
        inter2.y = a2*inter2.x + c2;
        geometry_msgs::msg::PoseArray right = get_intermediate(poly.poses[1].position,inter1,step);
        geometry_msgs::msg::PoseArray left = get_intermediate(poly.poses[1].position,inter2,step);

        geometry_msgs::msg::PoseArray fullpath;
        if(right.poses.size() == left.poses.size())
        {
            for(int j = 0;j<right.poses.size();j++)
            {
                geometry_msgs::msg::PoseArray aux_array;
                aux_array = get_intermediate(left.poses[j].position,right.poses[j].position,step);

                fullpath.poses.insert(
                    fullpath.poses.end(),
                    std::make_move_iterator(aux_array.poses.begin()),
                    std::make_move_iterator(aux_array.poses.end())
                    );
            }
        }else{
            // RCLCPP_INFO(this->get_logger(), "Los vectores no son iguales, ya te lo digo yo para que no te salga el p*** segmentation fault");
        }
        return fullpath;
    }

private:

    
    geometry_msgs::msg::Point get_start(const geometry_msgs::msg::PoseStamped& robot_pose,const geometry_msgs::msg::PoseArray& poly)
    {
        double d_min = 3000;
        geometry_msgs::msg::Point closest_point;
        for (int p = 0; p < poly.poses.size();p++)
        {   
            double dx = poly.poses[p].position.x - robot_pose.pose.position.x;
            double dy = poly.poses[p].position.y - robot_pose.pose.position.y;
            double d = sqrt(abs(dx*dx)+abs(dy*dy));
            if (d < d_min)
            {
                d_min = d;
                closest_point.x = poly.poses[p].position.x;
                closest_point.y = poly.poses[p].position.y;
            }
        }
        return closest_point;
    }
    bool is_inside_polygon(geometry_msgs::msg::PoseArray poly, geometry_msgs::msg::Point punto)
    {
        double angle  = 0.0;
        for (int p = 0; p < poly.poses.size();p++)
        {   
            double dx = poly.poses[p].position.x - punto.x;
            double dy = poly.poses[p].position.y - punto.y;
            angle = angle + atan2(dy,dx);            
        }
        double error = abs((2*M_PI) - angle);
        if (error > 0.05)
        {
            return false;
        }
        return true;
    }

};

#endif // AREA_PATH_GENERATOR_HPP