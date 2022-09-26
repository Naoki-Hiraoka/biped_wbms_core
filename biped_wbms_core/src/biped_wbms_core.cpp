#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>
#include <biped_wbms_core/BipedWbmsCoreConfig.h>
#include <cpp_filters/IIRFilter.h>
#include <cpp_filters/TwoPointInterpolator.h>
#include <Eigen/Eigen>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_goVelocity.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_goStop.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_setFootSteps.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_setFootStepsWithParam.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_waitFootSteps.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_getAutoStabilizerParam.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_setAutoStabilizerParam.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_getFootStepState.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_startWholeBodyMasterSlave.h>
#include <auto_stabilizer/OpenHRP_AutoStabilizerService_stopWholeBodyMasterSlave.h>

/*
  <独立thread 500Hz>
  master_poseとslave_wrenchをtopicから取得し続ける

  slave_wrenchにlpf, hpfして流す
  slave_wrench[0:2]は、isManualControlModeで無いなら、0にする

  master_poseをそのまま流す.
  master_poseは、(STATE_STEPBUTTON or STATE_LIFTBUTTON)なら、最後にSTATE_NORMALだったときの値で止める. STATE_NORMALに戻ったら、滑らかに補間する.


  <thread> (サービスコールには少し時間がかかるのでスレッドを分けている)
  isManualControlModeかをサービスコールして取得し続ける


  <thread> (サービスコールには少し時間がかかるのでスレッドを分けている)
  button1,2をtopicから取得し続ける
  cmd_vel1, 2をtopicから取得し続ける
  以下の状態遷移
  STATE_NORMAL
  STATE_STEPBUTTON_RLEG (STATE_NORMAL時に(button1がtrue)になったら至る)
      遷移時、master_poseを停止, その足isReferenceFrame=false, 反対足isReferenceFrame=true
    (button1がfalse)になったら、STATE_NORMALへ遷移
      遷移時、master_poseの補間器開始, setFootSteps(swingEnd=false).
  STATE_LIFTBUTTON_RLEG (STATE_NORMAL時に(button2がtrue)になったら至る)
      遷移時、master_poseを停止, その足isReferenceFrame=false, 反対足isReferenceFrame=true. handFixMode=true
    (button2がfalse)になったら、STATE_NORMALへ遷移
      遷移時、master_poseの補間器開始, setFootSteps(swingEnd=true), waitFootSteps, isManualControlMode=true
  STATE_STEPBUTTON_LLEG
  STATE_LIFTBUTTON_LLEG
  STATE_CMDVEL (STATE_NORMAL時に(cmd_vel1が非ゼロになったら至る)
      遷移時、handFixMode=false
      以後、goVelocityを呼び続ける
    (cmd_vel1がゼロ)になったら、STATE_NORMALに遷移
      遷移時、goStopを呼ぶ
  STATE_HANDFIXEDCMDVEL (STATE_NORMAL時に(cmd_vel2が非ゼロになったら至る)
      遷移時handFixMode=true
      以後、goVelocityを呼び続ける
    (cmd_vel2がゼロ)になったら、STATE_NORMALに遷移
      遷移時、goStopを呼ぶ
  STATE_IDLE(STATE_NORMAL時に(button3がfalse->true)になったら至る)
      遷移時、stopWholeBodyMasterSlaveを呼ぶ
    (button3がfalse->true)になったら, STATE_NORMALに遷移
      遷移時、startWholeBodyMasterSlaveを呼ぶ
*/

#define RLEG 0
#define LLEG 0
#define NUM_LEGS 2

auto_stabilizer::OpenHRP_AutoStabilizerService_Footstep poseMsgToIdl(const geometry_msgs::Pose& msg){
  auto_stabilizer::OpenHRP_AutoStabilizerService_Footstep ret;
  ret.pos[0] = msg.position.x;
  ret.pos[1] = msg.position.y;
  ret.pos[2] = msg.position.z;
  ret.rot[0] = msg.orientation.w;
  ret.rot[1] = msg.orientation.x;
  ret.rot[2] = msg.orientation.y;
  ret.rot[3] = msg.orientation.z;
  return ret;
}

Eigen::Transform<double, 3, Eigen::AffineCompact> poseMsgToEigen(const geometry_msgs::Pose& msg){
  Eigen::Transform<double, 3, Eigen::AffineCompact> ret;
  ret.translation()[0] = msg.position.x;
  ret.translation()[1] = msg.position.y;
  ret.translation()[2] = msg.position.z;
  ret.linear() = Eigen::Quaterniond(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z).toRotationMatrix();
  return ret;
};

geometry_msgs::Pose poseEigenToMsg(const Eigen::Transform<double, 3, Eigen::AffineCompact>& eigen){
  geometry_msgs::Pose ret;
  ret.position.x = eigen.translation()[0];
  ret.position.y = eigen.translation()[1];
  ret.position.z = eigen.translation()[2];
  Eigen::Quaterniond q(eigen.linear());
  ret.orientation.w = q.w();
  ret.orientation.x = q.x();
  ret.orientation.y = q.y();
  ret.orientation.z = q.z();
  return ret;
};

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
        if(this->config_.getparam_rate != config_in.getparam_rate)
          this->getParamTimer_.setPeriod(ros::Duration(1.0 / config_in.getparam_rate));
        if(this->config_.footstep_rate != config_in.footstep_rate)
          this->footstepTimer_.setPeriod(ros::Duration(1.0 / config_in.footstep_rate));

        this->config_ = config_in;
      }); //setCallbackの中でcallbackが呼ばれる

    this->goVelocityClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_goVelocity>("AutoStabilizerServiceROSBridge/goVelocity");
    this->goStopClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_goStop>("AutoStabilizerServiceROSBridge/goStop");
    this->setFootStepsClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_setFootSteps>("AutoStabilizerServiceROSBridge/setFootSteps");
    this->setFootStepsWithParamClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_setFootStepsWithParam>("AutoStabilizerServiceROSBridge/setFootStepsWithParam");
    this->waitFootStepsClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_waitFootSteps>("AutoStabilizerServiceROSBridge/waitFootSteps");
    this->getAutoStabilizerParamClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam>("AutoStabilizerServiceROSBridge/getAutoStabilizerParam");
    this->setAutoStabilizerParamClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam>("AutoStabilizerServiceROSBridge/setAutoStabilizerParam");
    this->getFootStepStateClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_getFootStepState>("AutoStabilizerServiceROSBridge/getFootStepState");
    this->startWholeBodyMasterSlaveClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_startWholeBodyMasterSlave>("AutoStabilizerServiceROSBridge/startWholeBodyMasterSlave");
    this->stopWholeBodyMasterSlaveClient_ = this->nh_.serviceClient<auto_stabilizer::OpenHRP_AutoStabilizerService_stopWholeBodyMasterSlave>("AutoStabilizerServiceROSBridge/stopWholeBodyMasterSlave");


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
      this->masterPoseFilter_.push_back(cpp_filters::TwoPointInterpolatorSE3(Eigen::Transform<double, 3, Eigen::AffineCompact>::Identity(), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), cpp_filters::HOFFARBIB));
      this->masterPosePub_.push_back(this->nh_.advertise<geometry_msgs::PoseStamped>("master_" + endeffectors[i] + "_pose_converted",1));
      this->slavePosePub_.push_back(this->nh_.advertise<geometry_msgs::PoseStamped>("slave_" + endeffectors[i] + "_pose_converted",1));
      this->slaveWrenchLpf_.push_back(cpp_filters::IIRFilter<Eigen::VectorXd>());
      this->slaveWrenchLpf_.back().setParameterAsBiquad(this->config_.lpf_cutoff_hz, 1.0/std::sqrt(2), config_.rate, Eigen::VectorXd::Zero(6));
      this->slaveWrenchLpfForHpf_.push_back(cpp_filters::IIRFilter<Eigen::VectorXd>());
      this->slaveWrenchLpfForHpf_.back().setParameterAsBiquad(this->config_.hpf_cutoff_hz, 1.0/std::sqrt(2), config_.rate, Eigen::VectorXd::Zero(6));
      this->slaveWrenchPub_.push_back(this->pnh_.advertise<geometry_msgs::WrenchStamped>("slave_" + endeffectors[i] + "_wrench_converted",1));
    }

    ros::TimerOptions poseWrenchTimerOption(ros::Duration(1.0/this->config_.rate),
                                            [&](const ros::TimerEvent& event){this->poseWrenchTimerCallBack(event);},
                                            &(this->poseWrenchCallbackQueue_));
    this->poseWrenchTimer_ = this->nh_.createTimer(poseWrenchTimerOption);
    this->poseWrenchSpinner_ = std::make_shared<ros::AsyncSpinner>(1,&(poseWrenchCallbackQueue_));
    this->poseWrenchSpinner_->start();

    for(int i=0;i<NUM_LEGS;i++){
      this->isManualControlMode_.push_back(false);
    }
    ros::TimerOptions getParamTimerOption(ros::Duration(1.0/this->config_.getparam_rate),
                                            [&](const ros::TimerEvent& event){this->getParamTimerCallBack(event);},
                                            &(this->getParamCallbackQueue_));
    this->getParamTimer_ = this->nh_.createTimer(getParamTimerOption);
    this->getParamSpinner_ = std::make_shared<ros::AsyncSpinner>(1,&(getParamCallbackQueue_));
    this->getParamSpinner_->start();

    for(int i=0;i<NUM_LEGS;i++){
      ros::SubscribeOptions stepButtonSubOption = ros::SubscribeOptions::create<std_msgs::Bool>
        (endeffectors[i] + "_step_button",
         1,
         [&,i](const std_msgs::Bool::ConstPtr& msg){this->stepButton_[i]=msg;},
         ros::VoidPtr(),
         &(this->footstepCallbackQueue_)
         );
      this->stepButtonSub_.push_back(this->nh_.subscribe(stepButtonSubOption));
      std_msgs::Bool* msg = new std_msgs::Bool();
      msg->data = false;
      this->stepButton_.emplace_back(msg);
    }

    for(int i=0;i<NUM_LEGS;i++){
      ros::SubscribeOptions liftButtonSubOption = ros::SubscribeOptions::create<std_msgs::Bool>
        (endeffectors[i] + "_lift_button",
         1,
         [&,i](const std_msgs::Bool::ConstPtr& msg){this->liftButton_[i]=msg;},
         ros::VoidPtr(),
         &(this->footstepCallbackQueue_)
         );
      this->liftButtonSub_.push_back(this->nh_.subscribe(liftButtonSubOption));
      std_msgs::Bool* msg = new std_msgs::Bool();
      msg->data = false;
      this->liftButton_.emplace_back(msg);
    }

    {
      ros::SubscribeOptions cmdVelSubOption = ros::SubscribeOptions::create<geometry_msgs::Twist>
        ("cmd_vel",
         1,
         [&](const geometry_msgs::Twist::ConstPtr& msg){this->cmdVel_=msg;},
         ros::VoidPtr(),
         &(this->footstepCallbackQueue_)
         );
      this->cmdVelSub_ = this->nh_.subscribe(cmdVelSubOption);
      geometry_msgs::Twist* msg = new geometry_msgs::Twist();
      msg->linear.x = msg->linear.y = msg->linear.z = 0.0;
      msg->angular.x = msg->angular.y = msg->angular.z = 0.0;
      this->cmdVel_ = geometry_msgs::Twist::ConstPtr(msg);
    }

    {
      ros::SubscribeOptions handFixedCmdVelSubOption = ros::SubscribeOptions::create<geometry_msgs::Twist>
        ("hand_fixed_cmd_vel",
         1,
         [&](const geometry_msgs::Twist::ConstPtr& msg){this->handFixedCmdVel_=msg;},
         ros::VoidPtr(),
         &(this->footstepCallbackQueue_)
         );
      this->handFixedCmdVelSub_ = this->nh_.subscribe(handFixedCmdVelSubOption);
      geometry_msgs::Twist* msg = new geometry_msgs::Twist();
      msg->linear.x = msg->linear.y = msg->linear.z = 0.0;
      msg->angular.x = msg->angular.y = msg->angular.z = 0.0;
      this->handFixedCmdVel_ = geometry_msgs::Twist::ConstPtr(msg);
    }

    {
      ros::SubscribeOptions startStopButtonSubOption = ros::SubscribeOptions::create<std_msgs::Bool>
        ("startstop_button",
         1,
         [&](const std_msgs::Bool::ConstPtr& msg){this->startStopButtonPrev_ = this->startStopButton_; this->startStopButton_=msg;},
         ros::VoidPtr(),
         &(this->footstepCallbackQueue_)
         );
      this->startStopButtonSub_ = this->nh_.subscribe(startStopButtonSubOption);
      std_msgs::Bool* msg = new std_msgs::Bool();
      msg->data = false;
      this->startStopButton_ = this->startStopButtonPrev_ = std_msgs::Bool::ConstPtr(msg);
    }


    ros::TimerOptions footstepTimerOption(ros::Duration(1.0/this->config_.footstep_rate),
                                          [&](const ros::TimerEvent& event){this->footstepTimerCallBack(event);},
                                          &(this->footstepCallbackQueue_));
    this->footstepTimer_ = this->nh_.createTimer(footstepTimerOption);
    this->footstepSpinner_ = std::make_shared<ros::AsyncSpinner>(1,&(footstepCallbackQueue_));
    this->footstepSpinner_->start();

  }
protected:
  void poseWrenchTimerCallBack(const ros::TimerEvent& event){
    for(int i=0;i<this->masterPose_.size();i++){
      Eigen::Transform<double, 3, Eigen::AffineCompact> masterPose;
      if((i == RLEG && (this->state_ == STATE_STEP_BUTTON_RLEG || this->state_ == STATE_LIFT_BUTTON_RLEG)) ||
         (i == LLEG && (this->state_ == STATE_STEP_BUTTON_LLEG || this->state_ == STATE_LIFT_BUTTON_LLEG))){
        masterPose = this->masterPoseFilter_[i].getGoal();
      }else{
        masterPose = poseMsgToEigen(this->masterPose_[i]->pose);
      }
      if(this->masterPoseFilter_[i].isEmpty()) this->masterPoseFilter_[i].reset(masterPose);
      else{
        this->masterPoseFilter_[i].setGoal(poseMsgToEigen(this->masterPose_[i]->pose), this->masterPoseFilter_[i].remain_time());
        this->masterPoseFilter_[i].interpolate(1.0 / this->config_.rate);
      }
      geometry_msgs::PoseStamped msg = *(this->masterPose_[i]);
      msg.pose = poseEigenToMsg(this->masterPoseFilter_[i].value());
      this->masterPosePub_[i].publish(msg);
    }
    for(int i=0;i<this->slavePose_.size();i++){
      this->slavePosePub_[i].publish(*(this->slavePose_[i]));
    }
    for(int i=0;i<this->slaveWrench_.size();i++){
      Eigen::VectorXd wrench = (Eigen::VectorXd(6) <<
                                this->slaveWrench_[i]->wrench.force.x,
                                this->slaveWrench_[i]->wrench.force.y,
                                this->slaveWrench_[i]->wrench.force.z,
                                this->slaveWrench_[i]->wrench.torque.x,
                                this->slaveWrench_[i]->wrench.torque.y,
                                this->slaveWrench_[i]->wrench.torque.z).finished();
      if(i<NUM_LEGS && !this->isManualControlMode_[i]) wrench.setZero();
      const Eigen::VectorXd w_hp = wrench - this->slaveWrenchLpfForHpf_[i].passFilter( wrench );
      const Eigen::VectorXd w_lp = this->slaveWrenchLpf_[i].passFilter( wrench );
      Eigen::VectorXd w_shaped = wrench * this->config_.gain + w_hp * this->config_.hpf_gain + w_lp * this->config_.lpf_gain;
      if(i<NUM_LEGS && !this->isManualControlMode_[i]) w_shaped.setZero();

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

  void getParamTimerCallBack(const ros::TimerEvent& event){
    auto_stabilizer::OpenHRP_AutoStabilizerService_getFootStepState getSrv;
    if(!this->getFootStepStateClient_.call(getSrv)){
      ROS_ERROR("failed to call service getFootStepState");
    }else{
      for(int i=0;i<NUM_LEGS;i++){
        this->isManualControlMode_[i] = getSrv.response.i_param.is_manual_control_mode[i];
      }
    }
  }

  void footstepTimerCallBack(const ros::TimerEvent& event){
    if(this->state_ == STATE_NORMAL){
      if(!this->startStopButtonPrev_->data && this->startStopButton_->data){
        this->startStopButtonPrev_ = this->startStopButton_; // これをしないと次の周期でも反応してしまう
        this->state_ = STATE_IDLE;
        ROS_INFO("STATE_IDLE");
        auto_stabilizer::OpenHRP_AutoStabilizerService_stopWholeBodyMasterSlave srv;
        if(!this->stopWholeBodyMasterSlaveClient_.call(srv)){
          ROS_ERROR("failed to call service stopWholeBodyMasterSlave");
        }
      }else if((std::abs(this->cmdVel_->linear.x) > 0.05) || (std::abs(this->cmdVel_->linear.y) > 0.05) || (std::abs(this->cmdVel_->angular.z) > 0.05)){
        this->state_ = STATE_CMDVEL;
        ROS_INFO("STATE_CMDVEL");
      }else if((std::abs(this->handFixedCmdVel_->linear.x) > 0.05) || (std::abs(this->handFixedCmdVel_->linear.y) > 0.05) || (std::abs(this->handFixedCmdVel_->angular.z) > 0.05)){
        this->state_ = STATE_HANDFIXED_CMDVEL;
        ROS_INFO("STATE_HANDFIXED_CMDVEL");
      }else if(this->stepButton_[RLEG]->data){
        this->state_ = STATE_STEP_BUTTON_RLEG;
        ROS_INFO("STATE_STEP_BUTTON_RLEG");
        {
          auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam getSrv;
          if(!this->getAutoStabilizerParamClient_.call(getSrv)){
            ROS_ERROR("failed to call service getAutoStabilizerParam");
          }else{
            if(getSrv.response.i_param.reference_frame[RLEG] != false || getSrv.response.i_param.reference_frame[LLEG] != true){
              auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam setSrv;
              setSrv.request.i_param = getSrv.response.i_param;
              setSrv.request.i_param.reference_frame[RLEG] = false;
              setSrv.request.i_param.reference_frame[LLEG] = true;
              if(!this->setAutoStabilizerParamClient_.call(setSrv)){
                ROS_ERROR("failed to call service setAutoStabilizerParam");
              }
            }
          }
        }
      }else if(this->stepButton_[LLEG]->data){
        this->state_ = STATE_STEP_BUTTON_LLEG;
        ROS_INFO("STATE_STEP_BUTTON_LLEG");
        {
          auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam getSrv;
          if(!this->getAutoStabilizerParamClient_.call(getSrv)){
            ROS_ERROR("failed to call service getAutoStabilizerParam");
          }else{
            if(getSrv.response.i_param.reference_frame[RLEG] != true || getSrv.response.i_param.reference_frame[LLEG] != false){
              auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam setSrv;
              setSrv.request.i_param = getSrv.response.i_param;
              setSrv.request.i_param.reference_frame[RLEG] = true;
              setSrv.request.i_param.reference_frame[LLEG] = false;
              if(!this->setAutoStabilizerParamClient_.call(setSrv)){
                ROS_ERROR("failed to call service setAutoStabilizerParam");
              }
            }
          }
        }
      }else if(this->liftButton_[RLEG]->data){
        this->state_ = STATE_LIFT_BUTTON_RLEG;
        ROS_INFO("STATE_LIFT_BUTTON_RLEG");
        {
          auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam getSrv;
          if(!this->getAutoStabilizerParamClient_.call(getSrv)){
            ROS_ERROR("failed to call service getAutoStabilizerParam");
          }else{
            if(getSrv.response.i_param.reference_frame[RLEG] != false || getSrv.response.i_param.reference_frame[LLEG] != true || getSrv.response.i_param.is_hand_fix_mode != true){
              auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam setSrv;
              setSrv.request.i_param = getSrv.response.i_param;
              setSrv.request.i_param.reference_frame[RLEG] = false;
              setSrv.request.i_param.reference_frame[LLEG] = true;
              setSrv.request.i_param.is_hand_fix_mode = true;
              if(!this->setAutoStabilizerParamClient_.call(setSrv)){
                ROS_ERROR("failed to call service setAutoStabilizerParam");
              }
            }
          }
        }
      }else if(this->liftButton_[LLEG]->data){
        this->state_ = STATE_LIFT_BUTTON_LLEG;
        ROS_INFO("STATE_LIFT_BUTTON_LLEG");
        {
          auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam getSrv;
          if(!this->getAutoStabilizerParamClient_.call(getSrv)){
            ROS_ERROR("failed to call service getAutoStabilizerParam");
          }else{
            if(getSrv.response.i_param.reference_frame[RLEG] != true || getSrv.response.i_param.reference_frame[LLEG] != false || getSrv.response.i_param.is_hand_fix_mode != true){
              auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam setSrv;
              setSrv.request.i_param = getSrv.response.i_param;
              setSrv.request.i_param.reference_frame[RLEG] = true;
              setSrv.request.i_param.reference_frame[LLEG] = false;
              setSrv.request.i_param.is_hand_fix_mode = true;
              if(!this->setAutoStabilizerParamClient_.call(setSrv)){
                ROS_ERROR("failed to call service setAutoStabilizerParam");
              }
            }
          }
        }
      }


    }else if(this->state_ == STATE_CMDVEL){
      if((std::abs(this->cmdVel_->linear.x) > 0.05) || (std::abs(this->cmdVel_->linear.y) > 0.05) || (std::abs(this->cmdVel_->angular.z) > 0.05)){
        {
          auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam getSrv;
          if(!this->getAutoStabilizerParamClient_.call(getSrv)){
            ROS_ERROR("failed to call service getAutoStabilizerParam");
          }else{
            if(getSrv.response.i_param.is_hand_fix_mode != false){
              auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam setSrv;
              setSrv.request.i_param = getSrv.response.i_param;
              setSrv.request.i_param.is_hand_fix_mode = false;
              if(!this->setAutoStabilizerParamClient_.call(setSrv)){
                ROS_ERROR("failed to call service setAutoStabilizerParam");
              }
            }
          }
        }
        {
          auto_stabilizer::OpenHRP_AutoStabilizerService_goVelocity srv;
          srv.request.vx = std::isfinite(this->cmdVel_->linear.x) ? this->cmdVel_->linear.x : 0;
          srv.request.vy = std::isfinite(this->cmdVel_->linear.y) ? this->cmdVel_->linear.y : 0;
          srv.request.vth = std::isfinite(this->cmdVel_->angular.z) ? this->cmdVel_->angular.z : 0;
          if(!this->goVelocityClient_.call(srv)){
            ROS_ERROR("failed to call service goVelocity");
          }
        }
      }else{
        auto_stabilizer::OpenHRP_AutoStabilizerService_goStop srv;
        if(!this->goStopClient_.call(srv)){
          ROS_ERROR("failed to call service goStop");
        }
        this->state_ = STATE_NORMAL;
        ROS_INFO("STATE_NORMAL");
      }


    }else if(this->state_ == STATE_HANDFIXED_CMDVEL){
      if((std::abs(this->handFixedCmdVel_->linear.x) > 0.05) || (std::abs(this->handFixedCmdVel_->linear.y) > 0.05) || (std::abs(this->handFixedCmdVel_->angular.z) > 0.05)){
        {
          auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam getSrv;
          if(!this->getAutoStabilizerParamClient_.call(getSrv)){
            ROS_ERROR("failed to call service getAutoStabilizerParam");
          }else{
            if(getSrv.response.i_param.is_hand_fix_mode != true){
              auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam setSrv;
              setSrv.request.i_param = getSrv.response.i_param;
              setSrv.request.i_param.is_hand_fix_mode = true;
              if(!this->setAutoStabilizerParamClient_.call(setSrv)){
                ROS_ERROR("failed to call service setAutoStabilizerParam");
              }
            }
          }
        }
        auto_stabilizer::OpenHRP_AutoStabilizerService_goVelocity srv;
        srv.request.vx = std::isfinite(this->handFixedCmdVel_->linear.x) ? this->handFixedCmdVel_->linear.x : 0;
        srv.request.vy = std::isfinite(this->handFixedCmdVel_->linear.y) ? this->handFixedCmdVel_->linear.y : 0;
        srv.request.vth = std::isfinite(this->handFixedCmdVel_->angular.z) ? this->handFixedCmdVel_->angular.z : 0;
        if(!this->goVelocityClient_.call(srv)){
          ROS_ERROR("failed to call service goVelocity");
        }
      }else{
        auto_stabilizer::OpenHRP_AutoStabilizerService_goStop srv;
        if(!this->goStopClient_.call(srv)){
          ROS_ERROR("failed to call service goStop");
        }
        this->state_ = STATE_NORMAL;
        ROS_INFO("STATE_NORMAL");
      }


    }else if(this->state_ == STATE_STEP_BUTTON_RLEG){
      if(!this->stepButton_[RLEG]->data){
        auto_stabilizer::OpenHRP_AutoStabilizerService_setFootSteps srv;
        srv.request.fs.resize(2);
        srv.request.fs[0] = poseMsgToIdl(this->masterPose_[LLEG]->pose);
        srv.request.fs[0].leg = "lleg";
        srv.request.fs[1] = poseMsgToIdl(this->masterPose_[RLEG]->pose);
        srv.request.fs[1].leg = "rleg";
        if(!this->setFootStepsClient_.call(srv)){
          ROS_ERROR("failed to call service setFootSteps");
        }
        this->masterPoseFilter_[RLEG].setGoal(masterPoseFilter_[RLEG].getGoal(), 1.0);
        this->state_ = STATE_NORMAL;
        ROS_INFO("STATE_NORMAL");
      }


    }else if(this->state_ == STATE_STEP_BUTTON_LLEG){
      if(!this->stepButton_[LLEG]->data){
        auto_stabilizer::OpenHRP_AutoStabilizerService_setFootSteps srv;
        srv.request.fs.resize(2);
        srv.request.fs[0] = poseMsgToIdl(this->masterPose_[RLEG]->pose);
        srv.request.fs[0].leg = "rleg";
        srv.request.fs[1] = poseMsgToIdl(this->masterPose_[LLEG]->pose);
        srv.request.fs[1].leg = "lleg";
        if(!this->setFootStepsClient_.call(srv)){
          ROS_ERROR("failed to call service setFootSteps");
        }
        this->masterPoseFilter_[LLEG].setGoal(masterPoseFilter_[LLEG].getGoal(), 1.0);
        this->state_ = STATE_NORMAL;
        ROS_INFO("STATE_NORMAL");
      }


    }else if(this->state_ == STATE_LIFT_BUTTON_RLEG){
      if(!this->liftButton_[RLEG]->data){
        auto_stabilizer::OpenHRP_AutoStabilizerService_setFootStepsWithParam srv;
        srv.request.fs.resize(2);
        srv.request.sps.resize(2);
        srv.request.fs[0] = poseMsgToIdl(this->masterPose_[LLEG]->pose);
        srv.request.fs[0].leg = "lleg";
        srv.request.fs[1] = poseMsgToIdl(this->masterPose_[RLEG]->pose);
        srv.request.fs[1].leg = "rleg";
        srv.request.sps[1].step_height = this->config_.step_height;
        srv.request.sps[1].step_time = this->config_.step_time;
        srv.request.sps[1].swing_end = true;
        if(!this->setFootStepsWithParamClient_.call(srv)){
          ROS_ERROR("failed to call service setFootStepsWithParam");
        }else{
          auto_stabilizer::OpenHRP_AutoStabilizerService_waitFootSteps srv;
          if(!this->waitFootStepsClient_.call(srv)){
            ROS_ERROR("failed to call service waitFootSteps");
          }else{
            auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam getSrv;
            if(!this->getAutoStabilizerParamClient_.call(getSrv)){
              ROS_ERROR("failed to call service getAutoStabilizerParam");
            }else{
              if(getSrv.response.i_param.is_manual_control_mode[RLEG] != true || getSrv.response.i_param.is_manual_control_mode[LLEG] != false){
                auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam setSrv;
                setSrv.request.i_param = getSrv.response.i_param;
                setSrv.request.i_param.is_manual_control_mode[RLEG] = true;
                setSrv.request.i_param.is_manual_control_mode[LLEG] = false;
                if(!this->setAutoStabilizerParamClient_.call(setSrv)){
                  ROS_ERROR("failed to call service setAutoStabilizerParam");
                }
              }
            }
          }
        }
        this->masterPoseFilter_[RLEG].setGoal(masterPoseFilter_[RLEG].getGoal(), 1.0);
        this->state_ = STATE_NORMAL;
        ROS_INFO("STATE_NORMAL");
      }


    }else if(this->state_ == STATE_LIFT_BUTTON_LLEG){
      if(!this->liftButton_[LLEG]->data){
        auto_stabilizer::OpenHRP_AutoStabilizerService_setFootStepsWithParam srv;
        srv.request.fs.resize(2);
        srv.request.sps.resize(2);
        srv.request.fs[0] = poseMsgToIdl(this->masterPose_[RLEG]->pose);
        srv.request.fs[0].leg = "rleg";
        srv.request.fs[1] = poseMsgToIdl(this->masterPose_[LLEG]->pose);
        srv.request.fs[1].leg = "lleg";
        srv.request.sps[1].step_height = this->config_.step_height;
        srv.request.sps[1].step_time = this->config_.step_time;
        srv.request.sps[1].swing_end = true;
        if(!this->setFootStepsWithParamClient_.call(srv)){
          ROS_ERROR("failed to call service setFootStepsWithParam");
        }else{
          auto_stabilizer::OpenHRP_AutoStabilizerService_waitFootSteps srv;
          if(!this->waitFootStepsClient_.call(srv)){
            ROS_ERROR("failed to call service waitFootSteps");
          }else{
            auto_stabilizer::OpenHRP_AutoStabilizerService_getAutoStabilizerParam getSrv;
            if(!this->getAutoStabilizerParamClient_.call(getSrv)){
              ROS_ERROR("failed to call service getAutoStabilizerParam");
            }else{
              if(getSrv.response.i_param.is_manual_control_mode[RLEG] != false || getSrv.response.i_param.is_manual_control_mode[LLEG] != true){
                auto_stabilizer::OpenHRP_AutoStabilizerService_setAutoStabilizerParam setSrv;
                setSrv.request.i_param = getSrv.response.i_param;
                setSrv.request.i_param.is_manual_control_mode[RLEG] = false;
                setSrv.request.i_param.is_manual_control_mode[LLEG] = true;
                if(!this->setAutoStabilizerParamClient_.call(setSrv)){
                  ROS_ERROR("failed to call service setAutoStabilizerParam");
                }
              }
            }
          }
        }
        this->masterPoseFilter_[LLEG].setGoal(masterPoseFilter_[LLEG].getGoal(), 1.0);
        this->state_ = STATE_NORMAL;
        ROS_INFO("STATE_NORMAL");
      }
    }else if(this->state_ == STATE_IDLE){
      if(!this->startStopButtonPrev_->data && this->startStopButton_->data){
        this->startStopButtonPrev_ = this->startStopButton_; // これをしないと次の周期でも反応してしまう
        this->state_ = STATE_NORMAL;
        ROS_INFO("STATE_NORMAL");
        auto_stabilizer::OpenHRP_AutoStabilizerService_startWholeBodyMasterSlave srv;
        if(!this->startWholeBodyMasterSlaveClient_.call(srv)){
          ROS_ERROR("failed to call service startWholeBodyMasterSlave");
        }
      }
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
  std::vector<cpp_filters::TwoPointInterpolatorSE3> masterPoseFilter_;
  std::vector<ros::Publisher> masterPosePub_;
  std::vector<ros::Publisher> slavePosePub_;
  std::vector<cpp_filters::IIRFilter<Eigen::VectorXd> > slaveWrenchLpf_;
  std::vector<cpp_filters::IIRFilter<Eigen::VectorXd> > slaveWrenchLpfForHpf_;
  std::vector<ros::Publisher> slaveWrenchPub_;
  ros::Timer poseWrenchTimer_;
  ros::CallbackQueue poseWrenchCallbackQueue_;
  std::shared_ptr<ros::AsyncSpinner> poseWrenchSpinner_;

  std::vector<bool> isManualControlMode_;
  ros::Timer getParamTimer_;
  ros::CallbackQueue getParamCallbackQueue_;
  std::shared_ptr<ros::AsyncSpinner> getParamSpinner_;

  enum State{
    STATE_NORMAL,
    STATE_STEP_BUTTON_RLEG,
    STATE_LIFT_BUTTON_RLEG,
    STATE_STEP_BUTTON_LLEG,
    STATE_LIFT_BUTTON_LLEG,
    STATE_CMDVEL,
    STATE_HANDFIXED_CMDVEL,
    STATE_IDLE
  } state_ = STATE_IDLE;
  std::vector<ros::Subscriber> stepButtonSub_;
  std::vector<std_msgs::Bool::ConstPtr> stepButton_;
  std::vector<ros::Subscriber> liftButtonSub_;
  std::vector<std_msgs::Bool::ConstPtr> liftButton_;
  ros::Subscriber cmdVelSub_;
  geometry_msgs::Twist::ConstPtr cmdVel_;
  ros::Subscriber handFixedCmdVelSub_;
  geometry_msgs::Twist::ConstPtr handFixedCmdVel_;
  ros::Subscriber startStopButtonSub_;
  std_msgs::Bool::ConstPtr startStopButton_;
  std_msgs::Bool::ConstPtr startStopButtonPrev_;
  ros::Timer footstepTimer_;
  ros::CallbackQueue footstepCallbackQueue_;
  std::shared_ptr<ros::AsyncSpinner> footstepSpinner_;

  ros::ServiceClient goVelocityClient_;
  ros::ServiceClient goStopClient_;
  ros::ServiceClient setFootStepsClient_;
  ros::ServiceClient setFootStepsWithParamClient_;
  ros::ServiceClient waitFootStepsClient_;
  ros::ServiceClient getAutoStabilizerParamClient_;
  ros::ServiceClient setAutoStabilizerParamClient_;
  ros::ServiceClient getFootStepStateClient_;
  ros::ServiceClient startWholeBodyMasterSlaveClient_;
  ros::ServiceClient stopWholeBodyMasterSlaveClient_;
};

int main(int argc, char** argv){
  ros::init(argc,argv, "biped_wbms_core");
  BipedWbmsCore bipedWbmsCore;
  ros::spin();
  return 0;
}
