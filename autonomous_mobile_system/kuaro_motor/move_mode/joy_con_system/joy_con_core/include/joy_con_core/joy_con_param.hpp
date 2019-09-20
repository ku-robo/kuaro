// メンバ変数には_を最後につけること

#ifndef JOY_CON_PARAM
#define JOY_CON_PARAM

// standard
#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <utility>

// ros
#include <ros/ros.h>
// self-made
#include <helper_node/helper_pub.hpp>

namespace joy_con_core{

  struct JoyConParam{
  public:
    std::vector<float> axes_;
    std::vector<int> buttons_;
    std::vector<float> init_axes_data_;
    std::vector<int> init_buttons_data_;
    std::vector< std::pair<std::string, std::string> > controller_params_, variable_params_;

    /*
    * @brief Constructor for JoyConParam
     */
    JoyConParam(void);
    /*
    * @brief Constructor for JoyConParam
    * @param axes_params : The corresponding string for each axes
    * @param buttons_params : The corresponding string for each button
     */
    JoyConParam(const std::vector<std::string>& axes_params, const std::vector<std::string>& buttons_params, const std::vector<float>& axes_data, const std::vector<int>& buttons_data);
    /*
    * @brief Operator=.
    * @param joy_data : to copy axes and buttons's values
     */
    JoyConParam& operator = (const JoyConParam& joy_data){
      this->axes_ = joy_data.axes_;
      this->buttons_ = joy_data.buttons_;
      this->init_axes_data_ = joy_data.init_axes_data_;
      this->init_buttons_data_ = joy_data.init_buttons_data_;
      this->axes_map_ = joy_data.axes_map_;
      this->buttons_map_ = joy_data.buttons_map_;
      this->action_map_ = joy_data.action_map_;
      this->param_flaot_map_ = joy_data.param_flaot_map_;
      this->param_str_map_ = joy_data.param_str_map_;
      return *this;
    }
    /*
    * @brief set axes and buttons's data on axes_, buttons_.
    * @param axes_data : axes's data
    * @param buttons_data : buttons's data
     */
    void set(std::vector<float> axes_data, std::vector<int> buttons_data);
    /*
    * @brief set initialize value of axes and buttons's data on axes_, buttons_.
     */
    void set_initvalue(void);
    /*
    * @brief clear axes and buttons's data on axes_, buttons_.
     */
    void clear(void);
    /*
    * @brief
     */
    bool check_set(void);
    /*
    * @brief return value corresponding to data_name.
    * @param data_name : axes's data name
     */
    int axes(std::string data_name);
    /*
    * @brief return value corresponding to data_name.
    * @param data_name : buttons's data name
     */
    float buttons(std::string data_name);
    /*
    * @brief return value corresponding to data_name.
    * @param data_name : action of a_act's data name
     */
    float a_act(std::string data_name);
    /*
    * @brief return value corresponding to data_name.
    * @param data_name : action of b_act's data name
     */
    int b_act(std::string data_name);
    /*
    * @brief return value corresponding to data_name.
    * @param data_name : param_float's data name
     */
    float param(std::string data_name);
    /*
    * @brief return value corresponding to data_name.
    * @param data_name : param_str's data name
     */
    std::string param_str(std::string data_name);
    /*
    * @brief initialize. make
     */
    virtual void initialize(const std::vector<std::string>& axes_params, const std::vector<std::string>& buttons_params, const std::vector<float>& axes_data, const std::vector<int>& buttons_data);
    /*
    * @brief set launch param
    * @param name : node_name
    * @param param_float : float param values. pair's first is param name, pair's second is default value.
    * @param param_str : string param values. same as above
     */
    virtual void set_param(std::string name, const std::vector<std::pair<std::string,float> >& param_float, std::vector<std::pair<std::string,std::string> >& param_str);
    /*
    * @brief set actions about joy-con's parts.
    * @param correspondence : correspondence between action and joy-con's parts
     */
    void set_action(const std::vector<std::pair<std::string,std::string> >& correspondence);
    /*
    * @brief explain Method of operation
     */
    virtual void explain_method_of_operation(void);
    /*
    * @brief display joy-con's all parts
     */
    void explain_joy_con(void);
    /*
    * @brief Destructor for ModeJoyCon
     */
    virtual ~JoyConParam(void);
  protected:
    /*
    * @brief display joy-con's parts
    * @param current_type : now joy-con's parts type.
    * @param current_type_map : current_type's map
     */
    void display_joy_con_key(std::string current_type, const std::map<std::string, int>& current_type_map);

    void set_joy_con_key(std::string current_type, const std::map<std::string, int>& current_type_map);

    // Must initialize
    bool initialize_flag_, set_flag_;

    // Mustn't initialize
    std::map<std::string, int> axes_map_, buttons_map_, action_map_;
    std::map<std::string, float> param_flaot_map_;
    std::map<std::string, std::string> param_str_map_, action_name_map_;

    helper_node::HelperPub pub_exp_, pub_param_;
  };

};

#endif
