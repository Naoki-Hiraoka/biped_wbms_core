#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>
#include <biped_wbms_core/BipedWbmsCoreConfig.h>
#include <cpp_filters/IIRFilter.h>
#include <Eigen/Eigen>

/*
  <独立thread 500Hz>
  master_poseとslave_wrenchをtopicから取得し続ける

  slave_wrenchにlpf, hpfして流す
  slave_wrench[0:2]は、isManualControlModeで無いなら、0にする

  master_poseをそのまま流す.
  master_poseは、(STATE_BUTTON1 or STATE_BUTTON2)なら、最後にSTATE_NORMALだったときの値で止める. STATE_NORMALに戻ったら、滑らかに補間する.


  <thread>
  isManualControlModeかをサービスコールして取得し続ける


  <thread>
  button1,2をtopicから取得し続ける
  cmd_vel1, 2をtopicから取得し続ける
  以下の状態遷移
  STATE_NORMAL
  STATE_BUTTON1_RLEG (STATE_NORMAL時に(button1がtrue)になったら至る)
      遷移時、master_poseを停止, 両足isReferenceFrame=true
    (button1がfalse)になったら、STATE_NORMALへ遷移
      遷移時、master_poseの補間器開始, setFootSteps(swingEnd=false).
  STATE_BUTTON2_RLEG (STATE_NORMAL時に(button2がtrue)になったら至る)
      遷移時、master_poseを停止, その足isReferenceFrame=false, 反対足isReferenceFrame=true. handFixMode=true
    (button2がfalse)になったら、STATE_NORMALへ遷移
      遷移時、master_poseの補間器開始, setFootSteps(swingEnd=true), waitFootSteps, isManualControlMode=true
  STATE_BUTTON1_LLEG
  STATE_BUTTON2_LLEG
  STATE_CMD_VEL1 (STATE_NORMAL時に(cmd_vel1が非ゼロになったら至る)
      遷移時、goVelocityを呼ぶ. handFixMode=false
      以後、goVelocityを呼び続ける
    (cmd_vel1がゼロ)になったら、STATE_NORMALに遷移
      遷移時、goStopを呼ぶ
  STATE_CMD_VEL2 (STATE_NORMAL時に(cmd_vel2が非ゼロになったら至る)
      遷移時、goVelocityを呼ぶ. handFixMode=true
      以後、goVelocityを呼び続ける
    (cmd_vel2がゼロ)になったら、STATE_NORMALに遷移
      遷移時、goStopを呼ぶ
*/

class BipedWbmsCore {
public:
  BipedWbmsCore(){
    std::vector<std::string> endeffectors{"rleg","lleg"};
    if(this->pnh_.hasParam("endeffectors")) this->pnh_.getParam("endeffectors", endeffectors);
    if(endeffectors.size() < 2 || endeffectors[0] != "rleg" || endeffectors[1] != "lleg"){
      ROS_FATAL("endeffectors.size() < 2 || endeffectors[0] != \"rleg\" || endeffectors[1] != \"lleg\"");
      exit(1);
    }

    this->cfgServer_.setCallback([&](biped_wbms_core::BipedWbmsCoreConfig& config_in, int32_t level){
        if(this->config_.lpf_cutoff_hz != config_in.lpf_cutoff_hz ||
           this->config_.rate != config_in.rate)
          for(int i=0;i<this->slaveWrenchLpf_.size();i++)
            this->slaveWrenchLpf_[i].setParameterAsBiquad(config_in.lpf_cutoff_hz, 1.0/sqrt(2), config_in.rate, this->slaveWrenchLpf_[i].get());
        if(this->config_.hpf_cutoff_hz != config_in.hpf_cutoff_hz ||
           this->config_.rate != config_in.rate)
          for(int i=0;i<this->slaveWrenchLpfForHpf_.size();i++)
            this->slaveWrenchLpfForHpf_[i].setParameterAsBiquad(config_in.hpf_cutoff_hz, 1.0/sqrt(2), config_in.rate, this->slaveWrenchLpfForHpf_[i].get());
        if(this->config_.rate != config_in.rate)
          this->poseWrenchTimer_.setPeriod(ros::Duration(1.0 / config_in.rate));

        this->config_ = config_in;
      }); //setCallbackの中でcallbackが呼ばれる

    for(int i=0;i<endeffectors.size();i++){
      ros::SubscribeOptions masterPoseSubOption = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>
        ("master_" + endeffectors[i] + "_pose",
         1,
         [&,i](const geometry_msgs::PoseStamped::ConstPtr& msg){this->masterPose_[i]=msg;},
         ros::VoidPtr(),
         &(this->poseWrenchCallbackQueue_)
         );
      this->masterPoseSub_.push_back(this->nh_.subscribe(masterPoseSubOption));
      geometry_msgs::PoseStamped* msg = new geometry_msgs::PoseStamped();
      msg->pose.position.x = msg->pose.position.y = msg->pose.position.z = 0.0;
      msg->pose.orientation.x = msg->pose.orientation.y = msg->pose.orientation.z = msg->pose.orientation.w = 0.0;
      this->masterPose_.emplace_back(msg);
    }

    for(int i=0;i<endeffectors.size();i++){
      ros::SubscribeOptions slavePoseSubOption = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>
        ("slave_" + endeffectors[i] + "_pose",
         1,
         [&,i](const geometry_msgs::PoseStamped::ConstPtr& msg){this->slavePose_[i]=msg;},
         ros::VoidPtr(),
         &(this->poseWrenchCallbackQueue_)
         );
      this->slavePoseSub_.push_back(this->nh_.subscribe(slavePoseSubOption));
      geometry_msgs::PoseStamped* msg = new geometry_msgs::PoseStamped();
      msg->pose.position.x = msg->pose.position.y = msg->pose.position.z = 0.0;
      msg->pose.orientation.x = msg->pose.orientation.y = msg->pose.orientation.z = msg->pose.orientation.w = 0.0;
      this->slavePose_.emplace_back(msg);
    }

    for(int i=0;i<endeffectors.size();i++){
      ros::SubscribeOptions slaveWrenchSubOption = ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>
        ("slave_" + endeffectors[i] + "_wrench",
         1,
         [&,i](const geometry_msgs::WrenchStamped::ConstPtr& msg){this->slaveWrench_[i]=msg;},
         ros::VoidPtr(),
         &(this->poseWrenchCallbackQueue_)
         );
      this->slaveWrenchSub_.push_back(this->nh_.subscribe(slaveWrenchSubOption));
      geometry_msgs::WrenchStamped* msg = new geometry_msgs::WrenchStamped();
      msg->wrench.force.x = msg->wrench.force.y = msg->wrench.force.z = 0.0;
      msg->wrench.torque.x = msg->wrench.torque.y = msg->wrench.torque.z = 0.0;
      this->slaveWrench_.emplace_back(msg);
    }

    for(int i=0;i<endeffectors.size();i++){
      this->masterPosePub_.push_back(this->pnh_.advertise<geometry_msgs::PoseStamped>("master_" + endeffectors[i] + "_pose",1));
      this->slavePosePub_.push_back(this->pnh_.advertise<geometry_msgs::PoseStamped>("slave_" + endeffectors[i] + "_pose",1));
      this->slaveWrenchLpf_.push_back(cpp_filters::IIRFilter<Eigen::VectorXd>());
      this->slaveWrenchLpf_.back().setParameterAsBiquad(this->config_.lpf_cutoff_hz, 1.0/std::sqrt(2), config_.rate, Eigen::VectorXd::Zero(6));
      this->slaveWrenchLpfForHpf_.push_back(cpp_filters::IIRFilter<Eigen::VectorXd>());
      this->slaveWrenchLpfForHpf_.back().setParameterAsBiquad(this->config_.hpf_cutoff_hz, 1.0/std::sqrt(2), config_.rate, Eigen::VectorXd::Zero(6));
      this->slaveWrenchPub_.push_back(this->pnh_.advertise<geometry_msgs::WrenchStamped>("slave_" + endeffectors[i] + "_wrench",1));
    }

    ros::TimerOptions poseWrenchTimerOption(ros::Duration(1.0/this->config_.rate),
                                            [&](const ros::TimerEvent& event){this->poseWrenchTimerCallBack(event);},
                                            &(this->poseWrenchCallbackQueue_));
    this->poseWrenchTimer_ = this->nh_.createTimer(poseWrenchTimerOption);
    this->poseWrenchSpinner_ = std::make_shared<ros::AsyncSpinner>(1,&(poseWrenchCallbackQueue_));
    this->poseWrenchSpinner_->start();
  }
protected:
  void poseWrenchTimerCallBack(const ros::TimerEvent& event){
    for(int i=0;i<this->masterPose_.size();i++){
      this->masterPosePub_[i].publish(*(this->masterPose_[i]));
    }
    for(int i=0;i<this->slavePose_.size();i++){
      this->slavePosePub_[i].publish(*(this->slavePose_[i]));
    }
    for(int i=0;i<this->slaveWrench_.size();i++){
      const Eigen::VectorXd wrench = (Eigen::VectorXd(6) <<
                                      this->slaveWrench_[i]->wrench.force.x,
                                      this->slaveWrench_[i]->wrench.force.y,
                                      this->slaveWrench_[i]->wrench.force.z,
                                      this->slaveWrench_[i]->wrench.torque.x,
                                      this->slaveWrench_[i]->wrench.torque.y,
                                      this->slaveWrench_[i]->wrench.torque.z).finished();
      const Eigen::VectorXd w_hp = wrench - this->slaveWrenchLpfForHpf_[i].passFilter( wrench );
      const Eigen::VectorXd w_lp = this->slaveWrenchLpf_[i].passFilter( wrench );
      const Eigen::VectorXd w_shaped = wrench * this->config_.gain + w_hp * this->config_.hpf_gain + w_lp * this->config_.lpf_gain;

      geometry_msgs::WrenchStamped msg;
      msg.header = this->slaveWrench_[i]->header;
      msg.wrench.force.x = w_shaped[0];
      msg.wrench.force.y = w_shaped[1];
      msg.wrench.force.z = w_shaped[2];
      msg.wrench.torque.x = w_shaped[3];
      msg.wrench.torque.y = w_shaped[4];
      msg.wrench.torque.z = w_shaped[5];
      this->slaveWrenchPub_[i].publish(msg);
    }
 }
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_ = ros::NodeHandle("~");

  biped_wbms_core::BipedWbmsCoreConfig config_;
  dynamic_reconfigure::Server<biped_wbms_core::BipedWbmsCoreConfig> cfgServer_;

  std::vector<ros::Subscriber> masterPoseSub_;
  std::vector<geometry_msgs::PoseStamped::ConstPtr> masterPose_;
  std::vector<ros::Subscriber> slavePoseSub_;
  std::vector<geometry_msgs::PoseStamped::ConstPtr> slavePose_;
  std::vector<ros::Subscriber> slaveWrenchSub_;
  std::vector<geometry_msgs::WrenchStamped::ConstPtr> slaveWrench_;
  std::vector<ros::Publisher> masterPosePub_;
  std::vector<ros::Publisher> slavePosePub_;
  std::vector<cpp_filters::IIRFilter<Eigen::VectorXd> > slaveWrenchLpf_;
  std::vector<cpp_filters::IIRFilter<Eigen::VectorXd> > slaveWrenchLpfForHpf_;
  std::vector<ros::Publisher> slaveWrenchPub_;
  ros::Timer poseWrenchTimer_;
  ros::CallbackQueue poseWrenchCallbackQueue_;
  std::shared_ptr<ros::AsyncSpinner> poseWrenchSpinner_;
};

int main(int argc, char** argv){
  ros::init(argc,argv, "biped_wbms_core");
  BipedWbmsCore bipedWbmsCore;
  ros::spin();
  return 0;
}
