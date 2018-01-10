/*
 * jointserial.cpp
 *
 *  Created on: Jan 7, 2018
 *      Author: llb
 */

#include <ros/ros.h>
#include <serial/serial.h>
//ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <j3_moveit_config/joint_msg.h>
#include <boost/asio.hpp>                  //包含boost库函数

#include <stdlib.h>
#include<time.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作
using namespace std;
serial::Serial ser; //声明串口对象
//io_service Object
io_service m_ios;

//Serial port Object
serial_port *pSerialPort;

//For save com name
//any_type m_port;

//Serial_port function exception
boost::system::error_code ec;


std::string serial_port_name;
int serial_baudrate = 115200;
unsigned char AA=1;
unsigned char aa;
//回调函数
void write_callback(const j3_moveit_config::joint_msg::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Writing to serial port" <<msg->id<<msg->r);
    unsigned char mid=(char) msg->id;
    double r=msg->r;
    double pi=3.1415926535;
    double per;
    if(mid==1){
      //-90~90
      per=2.0*r/pi;
    }else{
      //-135~135
      per=4.0/3.0*r/pi;
    }

    int posi=1500+int(per*1000.0);
    posi=posi<500?500:posi;
    posi=posi>2500?2500:posi;
    string sendData="#00";
    string c2str(1,'0'+mid);
    sendData+=c2str;
    sendData+="P";
    //string si=itoa(posi);
    ostringstream os;
    os << posi;
    string si = os.str();
    for (char i=0;i<4-si.length();i++){
      sendData+="0";
    }
    sendData+=si;
    sendData+="T0100!";
    //ROS_INFO("-------------%s",sendData.c_str());
    //#000P1500T1000!
    //{#000P1500T1000!#001P1500T1000!}
    //发送串口数据
    try
    {
      size_t len = write( *pSerialPort, buffer( sendData,sendData.length() ), ec );
      ROS_INFO("positon command send: %s",sendData.c_str());
    }catch (boost::system::system_error e){
      ROS_ERROR_STREAM("serail write err ");

    }
}
unsigned char buf[1];                      //定义字符串长度
unsigned char data[255];//
unsigned int dn=0;
unsigned int i=0,j=0;
unsigned char isWaittingPosi=0;
unsigned char waitId=0;
unsigned char waittingId=0;
unsigned char maxId=2;//input a wrong maxid can run test code when time is out.
unsigned char waitDida=0;
unsigned char waitDidaMax=50;//wait dida
int main (int argc, char** argv)
{

    //初始化节点
    ros::init(argc, argv, "arm_jointserial");
    //声明节点句柄
    ros::NodeHandle nh;

    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("/arm_motors", 1000, write_callback);
    //发布主题
    ros::Publisher joint_motor_pub = nh.advertise<j3_moveit_config::joint_msg>("/arm_motors", 1000);
    j3_moveit_config::joint_msg joint_motor_msg;
    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);

    sensor_msgs::JointState joint_state_msg;
    /*
      <param name="shoulder_zhuan_joint" value="0" />
      <param name="upper_arm_joint" value="1" />
      <param name="fore_arm_joint" value="2" />
      <param name="hand_wan_joint" value="3" />
      <param name="hand_zhuan_joint" value="4" />

      - arm_base_to_arm_round_joint_stevo0
      - shoulder_2_to_arm_joint_stevo1
      - big_arm_round_to_joint_stevo2
      - arm_joint_stevo2_to_arm_joint_stevo3
      - wrist_to_arm_joint_stevo4
      - arm_joint_stevo4_to_arm_joint_stevo5
*/
    string jointnames[3] = {"arm_base_to_arm_round_joint_stevo0",
                           "shoulder_2_to_arm_joint_stevo1",
                           "big_arm_round_to_joint_stevo2"

    };

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port_name, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);


    pSerialPort = new serial_port( m_ios );
    if ( pSerialPort ){
        //init_port( port_name, 8 );
        //Open Serial port object
        pSerialPort->open( serial_port_name, ec );
        //Set port argument
        pSerialPort->set_option( serial_port::baud_rate( serial_baudrate ), ec );
        pSerialPort->set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );
        pSerialPort->set_option( serial_port::parity( serial_port::parity::none ), ec );
        pSerialPort->set_option( serial_port::stop_bits( serial_port::stop_bits::one ), ec);
        pSerialPort->set_option( serial_port::character_size( 8 ), ec);
        m_ios.run();
    }

    ROS_INFO("-------------zzz joint serail is running .");

    ros::Rate zzzrat(100.0);


    while(nh.ok()){
      //check time is out.
      if(waitDida>waitDidaMax){
        waitDida=0;
        isWaittingPosi=0;
        ROS_INFO("error: time is out when conect motor %d.",waittingId );

        //test random position
        if(0){
          srand(time(0));
          joint_motor_msg.id=3;
          joint_motor_msg.r=3.14* (((float)rand()/RAND_MAX)-0.5)*2.0;
          joint_motor_pub.publish(joint_motor_msg);
        }
      }
      //send new request
      if(isWaittingPosi==0){
        //#000PRAD!
        //send position request ,result is #000P1500!

       /* stringstream ss;
        int n = int (waitId);
        string str;
        ss<<n;
        ss>>str;
        */
        string sendData="#00";
        string c2str(1,'0'+waitId);//<10
        sendData+=c2str;
        sendData+="PRAD!";
        try
        {
          size_t len = write( *pSerialPort, buffer( sendData,sendData.length() ), ec );
          ROS_INFO("positon request send: %s",sendData.c_str());
          isWaittingPosi=1;
          waittingId=waitId;
          //
          waitId+=1;
          waitId=(waitId>maxId)?0:waitId;
        }catch (boost::system::system_error e){
            ROS_ERROR_STREAM("serail write err ");

        }
      }
      //read
      while(1){
    	//ROS_INFO("test longbow");
        try
        {
          read (*pSerialPort,buffer(buf));
          data[dn]=buf[0];
          dn++;
          if(dn>=10){
        	//  ROS_INFO("attention" );
        	  break;
          }
         // ROS_INFO("%d", buf[0]);//打印接受到的字符串//打印接受到的字符串
        }catch (boost::system::system_error e){
          //ROS_ERROR_STREAM("read err ");
          break;
        }
        //ROS_INFO("still in " );
      }
      //ROS_INFO("llb  im out now" );
      //parse package//{'#'...'!'}
      vector <string> result;
      string str="";
      unsigned char findHead=0;
      unsigned char findTail=0;
      unsigned char ti=0;
      for(i=0;i<dn;i++){
        if((data[i])=='#'){
          findHead=1;
          findTail=0;
          str="";
          for(ti=i;ti<dn;ti++){
            str+=data[ti];
            if((data[ti])=='!'){
              findTail=1;
              break;
            }
          }
          if(findTail){
            i=ti;
            result.push_back(str);
            //here save result

          }else{
            break;
          }
        }else{
          //ignore
        }
      }
      if(i>0){
        ROS_INFO("read buffer lenght=%d",i);
      }
      //buffer move to 0
      for(j=0;i<dn;i++,j++){
        data[j]=data[i];
      }
      dn=j;
      //parase strings
      int count = result.size();
      for (int ri = 0; ri < count;ri++)
      {
        cout << result[ri] << endl;
        string s1,s2,s3;
        s1=result[ri];
        //#000P1500!
        //need check package data here!!!!
        size_t iPos = s1.find("P");
        if(s1.length()==10&&iPos==4){
          s2 = s1.substr(iPos-3, 3);
          s3 = s1.substr(iPos+1, 4);
          int id=atoi(s2.c_str());
          int posi=atoi(s3.c_str());
          ROS_INFO("get position result.  id= %d ,posi=%d",id,posi );

          joint_state_msg.header.stamp = ros::Time::now();
          joint_state_msg.name.resize(1);
          joint_state_msg.position.resize(1);
          joint_state_msg.name[0] =jointnames[id];
          double per=double (posi-1500)/1000.0;
          double pi=3.1415926535;
          double r;
          if(id==1){
            //-90~90
            r=per*pi/2.0;
          }else{
            //-135~135
            r=per*pi/4.0*3.0;
          }
          joint_state_msg.position[0] = r ;

          joint_pub.publish(joint_state_msg);
          ros::spinOnce();
          //publishmsg()
          //.....to do something
        }else{
          ROS_INFO("warnning. package data cant be parsed :%s",s1.c_str() );
        }

        isWaittingPosi=0;
        waitDida=0;

      }

      zzzrat.sleep();
      //ROS_INFO("loop---------------");
      waitDida+=1;
    }

}



