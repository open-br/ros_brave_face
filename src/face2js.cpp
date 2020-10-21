// 20200925
// ファイバリオン用頭部、顔追従プログラムノード (口はジョイスティックで動く)
// Face2jointstates

#include "ros/ros.h"
#include "ros/time.h"

#include "opencv_apps/FaceArrayStamped.h"
#include "sensor_msgs/JointState.h"

#include <sensor_msgs/Joy.h>

#define SV_CNT 3

double face_x = 0;
double face_y = 0;
double mouse = 0;
sensor_msgs::JointState js_robot;

void face_callback(const opencv_apps::FaceArrayStamped::ConstPtr& facearraystamped)
{
  face_x = 0; // 未検出のときは0
  face_y = 0; // 未検出のときは0

  if ( facearraystamped->faces.size() > 0 ){
    face_x = facearraystamped->faces[0].face.x;
    face_y = facearraystamped->faces[0].face.y;
  }
}

void joy_callback(const sensor_msgs::Joy& joy_msg)
{
  mouse = joy_msg.axes[1];
}

int main(int argc, char **argv)
{
  int i;
  ros::init(argc, argv, "face2js");           // ノードの初期化
  ros::NodeHandle nh;                         // ノードハンドラ

  // サブスクライバの作成
  ros::Subscriber sub_face = nh.subscribe("/face_detection/faces", 10, face_callback);
  ros::Subscriber sub_joy = nh.subscribe("joy", 10, joy_callback);

  // joint_stateのパブリッシャ
  ros::Publisher pub_js= nh.advertise<sensor_msgs::JointState>("/robot/joint_states",10);

  // JointStateの初期化
  js_robot.header.frame_id = "base_link";
  js_robot.name.resize(SV_CNT);
  js_robot.name[0] = "face_pitch";
  js_robot.name[1] = "face_yaw";
  js_robot.name[2] = "mouth";
  js_robot.position.resize(SV_CNT);
  for(i=0;i<SV_CNT;i++){                       // 値初期化
    js_robot.position[i] = 0.0;
  }

  ros::Rate loop_rate(30);                    // 制御周期30Hz
  
  while(ros::ok()) {

    if ((face_x == 0)&&(face_y == 0)){        // 顔を未検出のとき (たまたま(0,0)のときは、(^^;A))
      ROS_INFO("Not found face");
    }
    else{                                     // 顔を検出したとき
      ROS_INFO("found face !! | Face = (%f , %f)",face_x,face_y);

      //JointStateの生成
      js_robot.header.stamp = ros::Time::now();
      js_robot.position[0] = M_PI * ((face_y - 240) / 240 ) /6 ;  // ピッチ -30度〜30度
      js_robot.position[1] = -M_PI * ((face_x - 320) / 320 ) /3 ; // ヨー -60度〜60度
    }

    js_robot.position[2] = M_PI * mouse / 18  ;                   // 口の動き -10度〜10度

    // joint stateのパブリッシュ
    pub_js.publish(js_robot);

    ros::spinOnce();  // コールバック関数を呼ぶ
    loop_rate.sleep();
  }

  return 0;
}

