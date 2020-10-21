// 20200925
// ファイバリオン用Dynamixelドライバノード
// jointstates2dynamixel

#include "ros/ros.h"
#include "ros/time.h"

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "sensor_msgs/JointState.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define SV_CNT 3

std_msgs::String joint_name[SV_CNT];
std_msgs::Float64 joint_pos[SV_CNT];
std_msgs::Float64 joint_vel[SV_CNT];
std_msgs::Float64 joint_eff[SV_CNT];

//コールバックがあるとグローバルに読み込み
void monitorJointState_callback(const sensor_msgs::JointState::ConstPtr& jointstate)
{
  int i,j;
  for(i=0;i<SV_CNT;i++){
    for(j=0;j<SV_CNT;j++){
      if(joint_name[i].data == jointstate->name[j]){
        joint_pos[i].data = jointstate->position[j];    // ポジション読み出し
      }
    }
  }
}

int main(int argc, char **argv)
{
  int i,j;

  const char* port_name = "/dev/ttyUSB0";
  int baud_rate = 1000000;

  uint16_t model_number = 0;
  uint8_t dxl_id[SV_CNT] = {2, 3, 4};

  DynamixelWorkbench dxl_wb;

  const char *log;
  bool result = false;

  int32_t goal_position[SV_CNT] = {2048, 2048, 2048};
  const uint8_t handler_index = 0;

  joint_name[0].data = "face_pitch";
  joint_name[1].data = "face_yaw";
  joint_name[2].data = "mouth";

  // メイン関数の引数処理
  if (argc < 2)
  {
    printf("Please set '-port_name' arguments for connected Dynamixels\n");
    return 0;
  }
  else
  {
    port_name = argv[1];
  }

  // Dynamixel Worlbench 初期化
  result = dxl_wb.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init\n");

    return 0;
  }
  else
    printf("Succeed to init(%d)\n", baud_rate);  

  // Dynamixel 初期化
  for (int cnt = 0; cnt < SV_CNT; cnt++)
  {
    result = dxl_wb.ping(dxl_id[cnt], &model_number, &log);

    if (result == false)
    {
      printf("%s\n", log);
      printf("Failed to ping\n");
    }
    else
    {
      printf("Succeeded to ping\n");
      printf("id : %d, model_number : %d\n", dxl_id[cnt], model_number);
    }
    
     result = dxl_wb.jointMode(dxl_id[cnt], 48, 0, &log);
     if (result == false)
     {
       printf("%s\n", log);
       printf("Failed to change joint mode\n");
     }
     else
     {
       printf("Succeed to change joint mode\n");
     }
    
    result = dxl_wb.torqueOn(dxl_id[cnt]);
    if (result == false) printf("torqueOn ERROR \n");
    
  }

  // syncライトパケットのヘッダ生成
  result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to add sync write handler\n");
  }

  //--------------------------------------------------------------------------------------------------------

  result = dxl_wb.syncWrite(handler_index, &goal_position[0], &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to sync write position\n");
  }

  ros::init(argc, argv, "js2dxl");    // ノードの初期化
  ros::NodeHandle nh;                 // ノードハンドラ

  ros::Subscriber sub_joints;         // サブスクライバの作成
  sub_joints = nh.subscribe("/robot/joint_states", 1, monitorJointState_callback);

  ros::Rate loop_rate(100);           // 制御周期100Hz

  while (ros::ok()) {                 // 無限ループ

    for(i=0;i<SV_CNT;i++){
      goal_position[i] = 2047 + 2048 * ( joint_pos[i].data / (2 * M_PI) ) ;
    }

    result = dxl_wb.syncWrite(handler_index, &goal_position[0], &log);
    if (result == false)
    {
      printf("%s\n", log);
      printf("Failed to sync write position\n");
    }

    ros::spinOnce();                  // コールバック関数を呼ぶ
    loop_rate.sleep();                // 待ち
  }
  return 0;
}