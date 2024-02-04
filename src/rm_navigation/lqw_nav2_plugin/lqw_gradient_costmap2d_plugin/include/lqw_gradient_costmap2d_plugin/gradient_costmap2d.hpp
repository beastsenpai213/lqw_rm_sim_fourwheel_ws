#ifndef GRADIENT_COSTMAP2D_HPP_
#define GRADIENT_COSTMAP2D_HPP_

/***define***/
#include "rclcpp/rclcpp.hpp"

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
// #include "rclcpp/parameter_events_filter.hpp"
/***class***/
//声明命名空间,类名与方法名
//规则:
//1.全局变量不带_尾巴,局部变量带_尾巴
namespace lqw_gradient_costmap2d_plugin
{
    class GradientLayer : public nav2_costmap_2d::Layer
    {
        public:
            GradientLayer();
        
        //功能:初始化
        virtual void onInitialize();
        //功能:更新代价地图的更新范围(而不是更新整一张代价地图) 
        //参数:平面机器人自由度 x,y和yaw;以及最小最大的平面移动速度
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double * min_x, double * min_y, 
            double * max_x, double * max_y);
        //功能:更新代价地图 
        //参数:Costmap2D类型变量
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D & master_grid,
            int min_i, int min_j, int max_i, int max_j);
        //功能:重启
        virtual void reset()
        {
            return;
        }
        //功能:在机器人坐标更新时执行
        virtual void onFootprintChanged();
        //功能:是否清除
        virtual bool isClearable() {return false;}

        private:
            //存储上一次传入的范围极值
            double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
            //全局代价地图是否需要重新计算
            bool need_glaobal_recalculation_;
            //进行梯度代价评价的范围
            int GRADIENT_SIZE = 20;
            //每格梯度增长的代价
            int GRADIENT_FACTOR = 10;
    };
}// namespace lqw_gradient_costmap2d_plugin

#endif
