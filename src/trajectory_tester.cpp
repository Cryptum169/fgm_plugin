#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <ros/ros.h>
#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

namespace fgm_plugin {

    /* The rhs of x' = f(x) defined as a class */
    class circle_traj_func : public desired_traj_func{
        double vf_; //Forward vel
        double r_;  //radius of circle

    public:
        circle_traj_func( double vf, double r) : vf_(vf), r_(r) { }
        
        void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
        {
            
            dxdt[ni_state::XD_IND] = -vf_*sin((vf_/r_) * t);
            dxdt[ni_state::YD_IND] = vf_*cos((vf_/r_) * t);
        }
    };
    //]

 
    class TrajectoryGeneratorBridgeTester 
    {
        public:
            typedef ni_state state_type;
            typedef ni_controller traj_func_type;
            typedef trajectory_states<state_type, traj_func_type> traj_type;
            typedef std::shared_ptr<traj_type> traj_type_ptr;
    
    
        private:
            ros::NodeHandle nh_;
            std::string name_;
            ros::Publisher trajectory_publisher_, path_pub_;
            TrajectoryGeneratorBridge<state_type, traj_func_type>  traj_gen_bridge;

        public:
            TrajectoryGeneratorBridgeTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){};
            ~TrajectoryGeneratorBridgeTester(){};
    

        bool init()
        {
            trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/desired_trajectory", 10);
            path_pub_ = nh_.advertise< nav_msgs::Path>("/desired_path", 10);
            return true;
            
        };
        /**
         * Set-up necessary publishers/subscribers
         * @return true, if successful
         */

        pips_trajectory_msgs::trajectory_points generate_trajectory()
        {
            std::string r_key, fw_vel_key;
            double fw_vel = .25;
            double r = 5;
            
            //desired_traj_func::Ptr dtraj = std::make_shared<serpentine_traj_func>(.2,0.15,.2);
            desired_traj_func::Ptr dtraj = std::make_shared<circle_traj_func>(fw_vel,r);
            near_identity ni(100,100,100,.01);    
            traj_func_type::Ptr nc=std::make_shared<traj_func_type>(ni);
            nc->setTrajFunc(dtraj);
            
            
            traj_type_ptr traj = std::make_shared<traj_type>();
            traj->header.frame_id = "base_footprint";
            traj->header.stamp = ros::Time::now();
            traj->trajpntr = nc ;
            traj->params = std::make_shared<traj_params>();
            traj->params->tf=150;
            traj->x0_[ni_state::LAMBDA_IND]=.3;
            traj->x0_.yd=1;
            traj->x0_.xd=1;
            
            traj_gen_bridge.generate_trajectory(traj);
            
            ROS_INFO_STREAM("Size: " << traj->x_vec.size());
            
            std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << "xd" << '\t' << "yd" << std::endl;
            
            for( size_t i=0; i < traj->num_states(); i++ )
            {
                state_type& state = traj->x_vec[i];
                
                double error_x = state.x-state.xd;
                double error_y = state.y-state.yd;
                
                double error = sqrt(error_x*error_x + error_y*error_y);
                //NOTE: This doesn't work, for some reason
                std::cout << std::fixed << std::setw(4) <<std::setprecision(4) << traj->times[i] << "\t" << error << "\t" << state.x << "\t" << state.y << "\t" << state.theta << "\t" << state.v << "\t" << state.w << "\t" << state.lambda << "\t" << state.xd << "\t" << state.yd << std::endl;
            }
            
            
            pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toMsg ();
            trajectory_publisher_.publish(trajectory_msg);
            
            nav_msgs::Path::ConstPtr path_msg = traj->toPathMsg();    
            path_pub_.publish(path_msg);
            
            return trajectory_msg;
        }
    };
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_node_tester");
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName();

    fgm_plugin::TrajectoryGeneratorBridgeTester tester(nh,name);

    ROS_INFO_STREAM("Initiated");
    
    ros::Duration t(2);
    t.sleep();

    ros::Duration d(150);
    if (tester.init())
    {
      while(ros::ok())
      {
        tester.generate_trajectory();
        ros::spinOnce();
        d.sleep();
      }
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise test_ros_interface!");
    }

    ROS_INFO_STREAM("Program exiting");
    return 0;

}