#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include <aero_std/AeroMoveitInterface.hh>
#include "task_editor/TaskController.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
//////////////////////////////

/** 
* @brief aero_stdに関連したタスクを制御するためのコントローラ
* @details SEED-Lifter-Moverであれば、リフターの制御のみ行う。
* シナリオ(start.py)からはサービスにより呼び出される。
* サービスの定義は(TaskController.srv)に記述されている。
*/
class TaskController{
public:
  /**
  * @brief コンストラクタ
  * @param _nh ROSのノードハンドル
  */
  TaskController(ros::NodeHandle _nh);

  /**
  * @brief TaskControllerサービスが呼ばれた時のコールバック
  * @param _req task_controllerサービスのリクエスト
  * @param _res task_controllerサービスのレスポンス
  */
  bool taskControllerCallback(task_editor::TaskController::Request &_req, task_editor::TaskController::Response &_res);

  /** @brief 初期姿勢への移動。使用していない */
  bool initMotion();

  /**
  * @brief リフターの絶対位置制御
  * @param _x x（前後）方向の絶対位置距離[m]
  * @param _z z（上下）方向の絶対位置距離[m]
  * @param _time 時間[msec]
  */
  bool moveLifter(float _x,float _z,float _time);

  /** @brief リフターを曲げる。使用していない */
  bool downLifter();

  /** @brief リフターを伸ばす。使用していない */
  bool upLifter();

private:
  ros::NodeHandle nh_;  ///< @brief ROSのノードハンドル
  ros::ServiceServer task_controller_server_; ///< @brief task_controller_サーバーの定義
  aero::interface::AeroMoveitInterface::Ptr robot_; ///< @brief aero_stdを使うためのポインター定義
  std::map<aero::joint, double> joint_angles_;  ///< @brief 関節名のmap
};

#endif
