#include <joy_con_core/joy_con_param.hpp>

namespace joy_con_core{
  JoyConParam::JoyConParam(void):initialize_flag_(false), set_flag_(false){}

  JoyConParam::JoyConParam(const std::vector<std::string>& axes_params, const std::vector<std::string>& buttons_params, const std::vector<float>& axes_data, const std::vector<int>& buttons_data):
  initialize_flag_(false), set_flag_(false)
  {
    initialize(axes_params, buttons_params, axes_data, buttons_data);
  }
  

  JoyConParam::~JoyConParam(){}

  void JoyConParam::set(std::vector<float> axes_data, std::vector<int> buttons_data){
    axes_ = axes_data;
    buttons_ = buttons_data;
    set_flag_ = true;
  }

  void JoyConParam::set_initvalue(void){
    axes_ = init_axes_data_;
    buttons_ = init_buttons_data_;
    set_flag_ = true;
  }

  void JoyConParam::clear(void){
    axes_.clear();
    buttons_.clear();
    set_flag_ = false;
  }

  bool JoyConParam::check_set(void){
    return set_flag_;
  }

  int JoyConParam::axes(std::string data_name)
  {
    return axes_[ axes_map_[data_name] ];
  }

  float JoyConParam::buttons(std::string data_name)
  {
    return buttons_[ buttons_map_[data_name] ];
  }

  float JoyConParam::a_act(std::string data_name)
  {
    return axes_[ action_map_[data_name] ];
  }

  int JoyConParam::b_act(std::string data_name)
  {
    return buttons_[ action_map_[data_name] ];
  }

  float JoyConParam::param(std::string data_name)
  {
    return param_flaot_map_[data_name];
  }

  std::string JoyConParam::param_str(std::string data_name)
  {
    return param_str_map_[data_name];
  }

  void JoyConParam::initialize(const std::vector<std::string>& axes_params, const std::vector<std::string>& buttons_params, const std::vector<float>& axes_data, const std::vector<int>& buttons_data)
  {
    if(!initialize_flag_){
      // make axes_map_ and buttons_map_ from axes_params amd buttons_params
      unsigned int end = axes_params.size();
      for(unsigned int i=0; i!=end; i++){
        axes_map_[axes_params[i]] = i;
      }
      end = buttons_params.size();
      for(unsigned int i=0; i!=end; i++){
        buttons_map_[buttons_params[i]] = i;
      }
      init_axes_data_ = axes_data;
      init_buttons_data_ = buttons_data;
      set_initvalue();

      pub_exp_.initialize("/kuaro_motor/joy_con", "exp", "This Joy-Con's Axes and Buttons", helper_node::STATIC);
      pub_param_.initialize("/kuaro_motor/joy_con", "operation", "Method of operation", helper_node::STATIC);
      initialize_flag_ = true;
    }
  }

  void JoyConParam::set_param(std::string name, const std::vector<std::pair<std::string,float> >& param_float, std::vector<std::pair<std::string,std::string> >& param_str)
  {
    ros::NodeHandle node("~/"+name);
    std::vector<std::pair<std::string,std::string> > param_f2s;
    unsigned int end = param_float.size();
    param_f2s.resize(end);
    for(unsigned int i=0; i!=end; i++){
      float box;
      const std::string& p_f_f = param_float[i].first;
      const float& p_f_s = param_float[i].second;
      node.param(p_f_f, box, p_f_s);
      param_flaot_map_[p_f_f] = box;
      param_f2s[i] = std::make_pair(p_f_f, std::to_string(p_f_s));
    }
    end = param_str.size();
    for(unsigned int i=0; i!=end; i++){
      std::string box;
      node.param(param_str[i].first, box, param_str[i].second);
      param_str_map_[param_str[i].first] = box;
    }
    pub_param_.set("Correspondence", param_str);
    pub_param_.set("Parameters", param_f2s);
  }

  void JoyConParam::set_action(const std::vector<std::pair<std::string,std::string> >& correspondence)
  {
    unsigned int end = correspondence.size();
    for(unsigned int i=0; i!=end; i++){
      std::string search = correspondence[i].second;
      action_name_map_[correspondence[i].first] = search;
      // action_map_ is like axes_map_ and buttons_map_.
      action_map_[correspondence[i].first] = [&]()->int{if(this->axes_map_.count(search)){return this->axes_map_[search];}else if(this->buttons_map_.count(search)) {return this->buttons_map_[search];}}();
    }
    set_joy_con_key("axes", axes_map_);
    set_joy_con_key("buttons", buttons_map_);
  }

  void JoyConParam::explain_method_of_operation(void)
  {
    ROS_INFO(" JoyConParam ");
    std::cout << "-------------------------------" <<std::endl;
    {
      std::cout << "  --- Method of operation ---" <<std::endl;
      auto begin = action_name_map_.begin();
      auto end = action_name_map_.end();
      for(auto itr = begin; itr != end; itr++){
        std::cout << "  " << itr->first << " : " << itr->second << std::endl;
      }
    }
    {
      std::cout << "  --- List of parameters ---" <<std::endl;
      auto begin = param_flaot_map_.begin();
      auto end = param_flaot_map_.end();
      for(auto itr = begin; itr != end; itr++){
        std::cout << "  " << itr->first << " : " << itr->second << std::endl;
      }
    }
    {
      auto begin = param_str_map_.begin();
      auto end = param_str_map_.end();
      for(auto itr = begin; itr != end; itr++){
        std::cout << "  " << itr->first << " : " << itr->second << std::endl;
      }
    }
    std::cout << "-------------------------------\n" <<std::endl;
    pub_param_.publish(helper_node::KEEP);
  }

  void JoyConParam::explain_joy_con(void)
  {
    ROS_INFO(" JoyConParam ");
    std::cout << "-----------------------------------------" <<std::endl;
    std::cout << "  --- This Joy-Con's Axes and Buttons ---" <<std::endl;
    display_joy_con_key("axes", axes_map_);
    display_joy_con_key("buttons", buttons_map_);
    std::cout << "-----------------------------------------\n" <<std::endl;
    pub_exp_.publish(helper_node::KEEP);
  }

  void JoyConParam::display_joy_con_key(std::string current_type, const std::map<std::string, int>& current_type_map)
  {
    // 20190902はここから
    // keyとvalueを逆にして配列化→表示
    std::vector<std::string> box;
    unsigned int size = current_type_map.size();
    box.resize(size);
    auto begin = current_type_map.begin();
    auto end = current_type_map.end();
    for(auto itr = begin; itr != end; itr++){
      box[itr->second] = itr->first;
    }
    for(unsigned int i = 0;i != size; i++ ){

      std::cout << "  " << current_type << "[" << i << "] : " << box[i]<< std::endl;
    }
  }

  void JoyConParam::set_joy_con_key(std::string current_type, const std::map<std::string, int>& current_type_map)
  {
    // 20190902はここから
    // keyとvalueを逆にして配列化→表示
    std::vector<std::pair<std::string, std::string> > box;
    unsigned int size = current_type_map.size();
    box.resize(size);
    auto begin = current_type_map.begin();
    auto end = current_type_map.end();
    for(auto itr = begin; itr != end; itr++){
      std::stringstream ss;
      ss << current_type << "[" << itr->second << "]";
      box[itr->second] = std::make_pair(ss.str(), itr->first);
    }
    pub_exp_.set("Correspondence", box);
  }
};
