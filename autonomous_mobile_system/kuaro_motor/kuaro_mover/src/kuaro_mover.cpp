#include <kuaro_mover/kuaro_mover.hpp>

namespace kuaro_mover {

// 初期化部分
  // コンストラクタ
  KuaroMover::KuaroMover(void):
  move_mode_loader_("move_core", "move_core::MoveMode"),
  move_safety_loader_("move_core", "move_core::MoveSafety"),
  th_rad_(0.0), updateflag_(false)
  {
    ros::NodeHandle private_nh("~");

    std::string move_safer;
    private_nh.param("move_safety_system", move_safer, std::string("move_safety_system/ProtoType1"));
    private_nh.param("sub_serial_topic", sub_serial_topic_, std::string("/serial_receive"));
    private_nh.param("pub_serial_topic", pub_serial_topic_, std::string("/serial_send"));
    private_nh.param("loop_rate", loop_rate_, int(20));//[Hz]

    serial_sub_ = private_nh.subscribe(sub_serial_topic_,10,&KuaroMover::serial_Callback,this);
    serial_pub_ = private_nh.advertise<std_msgs::String>(pub_serial_topic_,10);
    //TinyPowerからオドメトリを取ってくるコマンド送信のタイマー関数
    timer_ = private_nh.createTimer(ros::Duration(ENCODER_GET_TIME), boost::bind(&KuaroMover::cmd_timer, this));

    // 初期化
    sub_command_twist_.init(0.0);
    pub_odom_pose_.init(0.0);
    pub_odom_twist_.init(0.0);

    // 20190827 Not Yet CREATED
    // //create a move safety system
    // try {
    //   safety_system_ = move_safety_loader_.createInstance(move_safer);
    //   ROS_INFO("Created move safety system %s", move_safer.c_str());
    //   safety_system_->initialize(move_safety_loader_.getName(move_safer));
    // } catch (const pluginlib::PluginlibException& ex) {
    //   ROS_FATAL("Failed to create the %s safer, are you sure it is properly registered and that the containing library is built? Exception: %s", move_safer.c_str(), ex.what());
    //   exit(1);
    // }
    // load any user specified move_modes, and if that fails load the defaults
    if(!loadMoveCores(private_nh)){
      loadDefaultMode();
    }
    // check which move mode uses use_encode_info
    makeMoveMode_encoders();

    ROS_INFO(" [KuaroMover] move_modes's size is %d", (int)move_modes_.size());
  }
  // デストラクタ
  KuaroMover::~KuaroMover(void)
  {
    move_modes_.clear();
    safety_system_.reset();
  }
  // インタフェースクラスの初期化(複数あるので動きとしては特殊)
  bool KuaroMover::loadMoveCores(ros::NodeHandle node)
  {
    //!!! YOU SHOULD ARRANGE MOVE_MODES IN ORDER OF YOUR PRIORITY!!!!
    XmlRpc::XmlRpcValue mode_list;
    if(node.getParam("move_modes", mode_list)){
      if(mode_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(unsigned int i = 0; i < mode_list.size(); ++i){
          if(mode_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(mode_list[i].hasMember("name") && mode_list[i].hasMember("type")){
              //同じ名前の機能が定義されていないかチェック
              for(unsigned int j = i + 1; j < mode_list.size(); j++){
                if(mode_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(mode_list[j].hasMember("name") && mode_list[j].hasMember("type")){
                    std::string name_i = mode_list[i]["name"];
                    std::string name_j = mode_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR(" [KuaroMover] A move mode with the name %s already exists, this is not allowed. Using the default move modes instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }else{
              ROS_ERROR(" [KuaroMover] Move modes must have a name and a type and this does not. Using the default move modes instead.");
              return false;
            }
          }else{
            ROS_ERROR(" [KuaroMover] Move modes must be specified as maps, but they are XmlRpcType %d. We'll use the default move modes instead.",
                mode_list[i].getType());
            return false;
          }
        }

        //ここからインスタンス化を行っていく
        for(unsigned int i = 0; i < mode_list.size(); ++i){
          try{
            //使えないクラスがないかチェック
            if(!move_mode_loader_.isClassAvailable(mode_list[i]["type"])){
              std::vector<std::string> classes = move_mode_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(mode_list[i]["type"] == move_mode_loader_.getName(classes[i])){
                  //名前の変更を促す
                  ROS_WARN(" [KuaroMover] Move mode specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(mode_list[i]["type"]).c_str(), classes[i].c_str());
                  mode_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<move_core::MoveMode> mode(move_mode_loader_.createInstance(mode_list[i]["type"]));

            //使える状態かチェック
            if(mode.get() == NULL){
              ROS_ERROR(" [KuaroMover] The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //初期化する
            // 20190815 or 16から以下を書き進める.初期化の仕方を定義.加えて,defaultloadをmove_baseに習って作成
            // mode->initialize(mode_list[i]["name"]);
            ROS_INFO(" [KuaroMover] Will load this plugin '%s' ", move_mode_loader_.getName(mode_list[i]["type"]).c_str());
            mode->initialize(move_mode_loader_.getName(mode_list[i]["type"]));
            move_modes_.push_back(mode);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR(" [KuaroMover] Failed to load a plugin. Using default move modes. Error: %s", ex.what());
            return false;
          }
        }
      }else{
        ROS_ERROR(" [KuaroMover] The move modes specification must be a list, but is of XmlRpcType %d. We'll use the default move modes instead.",
            mode_list.getType());
        return false;
      }
    }else{
      //もし設定がされていなければ,defaultmodeを読み込む
      return false;
    }

    //defaultmodeではないmodeを読み込めた状態
    ROS_INFO(" [KuaroMover] Success loading plugin");
    return true;
  }

  void KuaroMover::loadDefaultMode(void)
  {
    move_modes_.clear();
    try{
      ROS_INFO(" [KuaroMover] Load Default plugin");
      //first, we'll load a move_mode to mode_stop_and_go
      boost::shared_ptr<move_core::MoveMode> stop_and_go(move_mode_loader_.createInstance("stop_and_go/ModeStopAndGo"));
      stop_and_go->initialize(move_mode_loader_.getName("stop_and_go/ModeStopAndGo"));
      move_modes_.push_back(stop_and_go);

      //next, we'll load a move_mode to mode_joycon
      boost::shared_ptr<move_core::MoveMode> joycon(move_mode_loader_.createInstance("joy_con/ModeJoyCon"));
      joycon->initialize(move_mode_loader_.getName("joy_con/ModeJoyCon"));
      move_modes_.push_back(joycon);
      //
      // //if mode_move_vel_append_ is true, we'll append move_base to move_modes_
      // boost::shared_ptr<move_core::MoveMode> move_base(move_mode_loader_.createInstance("mode_move_vel/ModeMoveVel"));
      // if(mode_move_vel_append_){
      //   move_base->initialize("mode_move_vel");
      //   move_modes_.push_back(move_base);
      // }
      //
      // //if mode_wireless_append_ is true, we'll append wireless to move_modes_
      // boost::shared_ptr<move_core::MoveMode> wireless(move_mode_loader_.createInstance("mode_wireless/ModeWireLess"));
      // if(mode_wireless_append_){
      //   wireless->initialize("mode_wireless");
      //   move_modes_.push_back(wireless);
      // }
    }catch(pluginlib::PluginlibException& ex){
      ROS_FATAL(" [KuaroMover] Failed to load a plugin. This should not happen on default move modes. Error: %s", ex.what());
    }
  }

  void KuaroMover::makeMoveMode_encoders(void)
  {
    ROS_INFO(" [KuaroMover] Check which mode will use encode information.");
    move_mode_encoders_.clear();
    for(unsigned int i=0; i<move_modes_.size(); i++){
      if(move_modes_[i]->check_encode_flag()){
        move_mode_encoders_.push_back(move_modes_[i]);
      }
    }
  }

// メインの部分
  void KuaroMover::run(void)
  {
    ros::Rate loop(loop_rate_);
    ROS_INFO(" [KuaroMover]  start processing 'run'");
    while(ros::ok()){
      updateflag__init();// データ更新前にflagを初期化(false)
      // spinをしてデータ取得
      ros::spinOnce();
      // pluginからデータを取得
      if( check_update_data(sub_command_twist_)){
        ROS_INFO(" [KuaroMover] Scheduled to add safety system processing");
        // safety_system_->change_safety_value(sub_command_twist_); 開発でき次第追加
        // データを取得して渡す
        send_command(sub_command_twist_);
      }else{
        ROS_INFO(" [KuaroMover] No data....");
      }
      // エンコーダ情報を渡す
      // serial_Callbackの中に内蔵してもいいと思う...moveVel作成時に検討
      if(updateflag_){
        for(unsigned int i=0; i<move_mode_encoders_.size(); i++){
          move_mode_encoders_[i]->use_encode_info(pub_odom_pose_, pub_odom_twist_);
        }
      }
      // sleepを入れる
      loop.sleep();
    }
  }
  // データ更新前にflagを初期化する処理
  void KuaroMover::updateflag__init(void)
  {
    updateflag_ = false;//　エンコーダ情報の更新を確認するため
    // // 以下,インターフェースclassの更新確認用
    // for(unsigned int i=0; i<move_modes_.size(); i++){
    //   move_modes_[i]->set_update_flag();
    // }
  }

  bool KuaroMover::check_update_data(ambi_core::TwistParam& twist_data)
  {
    static unsigned int pre_i = 0;//previous i
    bool change_flag = false;//If it is true, it means that the mode used has been determined
    ROS_INFO(" [KuaroMover] Check update data....");
    // check whether any classes update data, and we'll get the data.
    for(unsigned int i=0; i<move_modes_.size(); i++){
      if(move_modes_[i]->check_update_flag(change_flag)){
        if(!change_flag){
          change_flag = true;
          twist_data = move_modes_[i]->get_command_value();
          if( pre_i <= i ){
            pre_i = i;
            return true;
          }
          pre_i = i;
        }
      }
    }
    if(change_flag){
      return true;
    }else{
      pre_i = 0;
      // if we couldn't catch any true flags, we'll prepare data.
      ambi_core::TwistParam default_twist;
      default_twist.init(0.0);
      twist_data = default_twist;
      return false;
    }
  }

// KUARO制御部分
  //call_back関数stringをTinyPowerにシリアル通信で送る
  void KuaroMover::send_command(const ambi_core::TwistParam& sub)
  {
    ROS_INFO(" [KuaroMover] Send Command : Linear is %lf, Angular is %lf....", sub.linear_x, sub.angular_z);
    std_msgs::String msg2tinypower;
    std::string str = serial_linear(sub.linear_x);

    if(sub.linear_x>=0.0){
      str+=serial_angular(sub.angular_z);
    }else{
      str+=serial_angular(-sub.angular_z);
    }
    msg2tinypower.data = str;
    serial_pub_.publish(msg2tinypower);
  }
  //前進速度制御　コマンド例（0.5[m/s]前進：VCX0.5）a
  std::string KuaroMover::serial_linear(const double& linear_vel)
  {
    std::string cmd = "\rVCX" + std::to_string(linear_vel)+"\r";
    return cmd;
  }
  //回頭速度制御　コマンド例（1[rad/s]旋回：VCR1）
  std::string KuaroMover::serial_angular(const double&  angular_vel)
  {
    std::string cmd = "\rVCR" + std::to_string(angular_vel)+"\r";
    return cmd;
  }
  // MMVコマンドをTinyPowerに送るタイマー関数
  void KuaroMover::cmd_timer(void)
  {
    std_msgs::String msg;
    msg.data = "\rMVV\r";
    serial_pub_.publish(msg);
  }
  // String型のメッセージを受け取り、オカテックのパラメータに代入(odomデータの取得)
  void KuaroMover::serial_Callback(const std_msgs::String::ConstPtr& serials)
  {
    std::string str = serials->data;
    static bool ustri_flag = false;
    static bool read_flag = false;
    static std::string extraction;

    for (unsigned int it = 0; it < (unsigned int)str.size(); it++){
      if(ustri_flag && str[it] == '>'){
        read_flag = true;
        break;
      }
      if (ustri_flag && str[it] != '\n' && str[it] != '\r'){
        extraction += str[it];
      }
      if (str[it] == ':'){
        ustri_flag = true;
      }
    }
    if(!read_flag){
      return;
    }
    double read_data1_ = 0;
    double read_data2_ = 0;
    int read_data3_ = 0;
    int read_data4_ = 0;
    updateflag_ = true;// データの更新
    ustri_flag = false;
    read_flag = false;
    //std::cout <<"extraction"<< extraction << std::endl;
    sscanf(extraction.c_str(), "%lf,%lf,%d,%d", &read_data1_, &read_data2_, &read_data3_, &read_data4_);
    //std::cout<<"read_data:"<<read_data1_<<","<<read_data2_<<","<<read_data3_<<","<<read_data4_<<std::endl;

    //set the position
    pub_odom_pose_.posi_x += read_data1_*cos(th_rad_)*ENCODER_GET_TIME;
    pub_odom_pose_.posi_y += read_data1_*sin(th_rad_)*ENCODER_GET_TIME;
    th_rad_ = th_rad_+read_data2_*ENCODER_GET_TIME;
    pub_odom_pose_.orien_z = th_rad_;

    //set the velocity
    pub_odom_twist_.linear_x = read_data1_;
    pub_odom_twist_.angular_z = read_data2_;
    extraction.clear();
  }

};
