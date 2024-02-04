#include "lqw_gradient_costmap2d_plugin/gradient_costmap2d.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace lqw_gradient_costmap2d_plugin
{
    //构造函数,为极限值先提供一个极值
    GradientLayer::GradientLayer()
    :last_min_x_(-1 * std::numeric_limits<float>::max()),
     last_min_y_(-1 * std::numeric_limits<float>::max()),
     last_max_x_( 1 * std::numeric_limits<float>::max()),
     last_max_y_( 1 * std::numeric_limits<float>::max())
    {
    }
    //初始化
    void GradientLayer::onInitialize()
    {
        auto node = node_.lock();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);

        need_glaobal_recalculation_ = false;
        current_ = true;
    }

    //功能:更新代价地图的更新范围(而不是更新整一张代价地图) 
    //参数:平面机器人自由度 x,y和yaw;以及最小最大的平面移动速度
    void GradientLayer::updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double * min_x, double * min_y, 
            double * max_x, double * max_y)
    {
        if (need_glaobal_recalculation_){
            //接收该次范围极值作为上一次接收的范围极值
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            //取极值,使得刷新范围为全局代价地图
            *min_x = -1 * std::numeric_limits<float>::max();
            *min_y = -1 * std::numeric_limits<float>::max();
            *max_x = 1 * std::numeric_limits<float>::max();
            *max_y = 1 * std::numeric_limits<float>::max();

            need_glaobal_recalculation_ = false;
        }
        else{
            //临时存储上一次范围极值
            double tmp_min_x = last_min_x_;
            double tmp_min_y = last_min_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;
            //接收该次范围极值作为上一次接收的范围极值
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            //代价地图范围极值更新,使得该地图的范围总是得到"已知"的最大的范围
            *min_x = std::min(tmp_min_x, *min_x);
            *min_y = std::min(tmp_min_y, *min_y);
            *max_x = std::max(tmp_max_x, *max_x);
            *max_y = std::max(tmp_max_y, *max_y);
        }


    }
    //此处为对footprint发生变化时的处理，仅仅是重新生成代价地图
    void GradientLayer::onFootprintChanged()
    {
        need_glaobal_recalculation_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
        "GradientLayer::onFootprintChanged(): num footprint points: %lu",
        layered_costmap_->getFootprint().size());
        
    }
    
    void GradientLayer::updateCosts(
        nav2_costmap_2d::Costmap2D & master_grid,
        int min_i, int min_j, int max_i, int max_j){
        //未启用时直接退出
        if(!enabled_){
            return;
        }
        
        //获取地图数据与地图大小
        unsigned char * master_array = master_grid.getCharMap();//获取地图
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsX();//地图大小

        //修正代价地图计算窗口大小
        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        //一个格子一个格子去计算代价地图
        int gradient_index;//需要在每次循环末尾重置该参数
        for (int j = min_j; j < max_j; j++){
            for (int i = min_i; i < max_i; i++){
                //获取地图中坐标（i，j）点的映射值
                int index = master_grid.getIndex(i,j);
                //设置单位梯度代价
                unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
                //计算
                if (gradient_index <= GRADIENT_SIZE){
                    gradient_index++;
                }
                else{
                    gradient_index = 0;
                }
                master_array[index] = cost;
            }
        }
    }


}// namespace lqw_gradient_costmap2d_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lqw_gradient_costmap2d_plugin::GradientLayer, nav2_costmap_2d::Layer)


