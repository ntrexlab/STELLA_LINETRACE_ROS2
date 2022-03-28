#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <termios.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace cv;

bool control_flag = true;
  
class LineTrace: public rclcpp::Node
{
public:
  LineTrace()
    : Node("line_trace")
  {
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    pub = image_transport::create_publisher(this, "image", custom_qos);
    sub = image_transport::create_subscription(this, "camera", std::bind(&LineTrace::imageCallback,this,std::placeholders::_1), "raw", custom_qos);
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);    
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try
    {
      Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      //ROI
      points[0] = Point(0,0);
      points[1] = Point(320,0);
      points[2] = Point(320,120);
      points[3] = Point(0,120);
      const Point* ppt[1] = {points};
      int npt[]={4};
      fillPoly(image,ppt,npt,1,Scalar(0,0,0),LINE_8);
      
      //COLOR FILTER
      cvtColor(image,image,COLOR_BGR2HSV);
      Mat result_image,red_mask,red_mask_1,red_mask_2;
      Scalar lower_red_1 = Scalar(170,50,50);
      Scalar upper_red_1 = Scalar(180,255,255);
      inRange(image,lower_red_1,upper_red_1,red_mask_1);
      Scalar lower_red_2 = Scalar(0,50,50);
      Scalar upper_red_2 = Scalar(10,255,255);
      inRange(image,lower_red_2,upper_red_2,red_mask_2);
      red_mask = red_mask_1 | red_mask_2;
      bitwise_and(image,image,result_image,red_mask);
      cvtColor(result_image,result_image,COLOR_HSV2BGR);
      
      //Histogram
      cvtColor(result_image,result_image,COLOR_BGR2GRAY);
      Mat imgNormal;
      double minVal, maxVal;
      Point minLoc, maxLoc;

      minMaxLoc(result_image, &minVal, &maxVal, &minLoc, &maxLoc);
      imgNormal = (255 / maxVal) * result_image;
      
      //calculation Lane
      imgNormal = imgNormal(Range(imgNormal.rows / 2, imgNormal.rows), Range(0, imgNormal.cols));
      vector<vector<Point>> PixelContaioner;

      vector<int> sumArray;
      for (int i=0; i < imgNormal.cols;i++)
      {
        Mat oneColumn = imgNormal.col(i);
        int sumResult = sum(oneColumn)[0];
        sumArray.push_back(sumResult);
      }

      int base;
      vector<int>::const_iterator begin = sumArray.begin();
      vector<int>::const_iterator last = sumArray.begin() + sumArray.size();
      vector<int> all(begin,last);
      int MaxIndex = max_element(all.begin(), all.end()) - all.begin();
      base = MaxIndex;

      vector<vector<Rect>> rectWindowInfo;
      Mat nonZeroPos;
      int numWindow = 10;
      int window_height = imgNormal.rows / numWindow;
      findNonZero(imgNormal,nonZeroPos);
      int win_yHigh;
      int win_yLow;
      int win_x_low;
      int margin = 30;
      int linePos = 0;
      int linePosCount = 0;

      for (int i=0; i < numWindow; i++)
      {
        win_yHigh = imgNormal.rows - (i)*window_height;
        win_yLow = imgNormal.rows - (i+1) * window_height;
        win_x_low = base - margin;

        Rect curWindow(win_x_low, win_yLow, 2 * margin, window_height);
        rectWindowInfo.push_back({curWindow});

        vector<Point> WindowPoint_IDX;
        int xLow = curWindow.x;
        int xHigh = xLow + curWindow.width;
        int yLow = curWindow.y;
        int yHigh = yLow + curWindow.height;

        for (int i=0;i < nonZeroPos.rows;i++)
        {
            int nonZeroX = nonZeroPos.at<Point>(i).x;
            int nonZeroY = nonZeroPos.at<Point>(i).y;
        
            Point onePt = {0,0};
            onePt = {nonZeroX, nonZeroY};
            
            if (nonZeroX >= xLow && nonZeroX < xHigh && nonZeroY >= yLow && nonZeroY < yHigh)
            {
                WindowPoint_IDX.push_back(onePt);
            }

        }
        PixelContaioner.push_back(WindowPoint_IDX);        
        if (WindowPoint_IDX.size() > 20)//minNumPix =20;
        {
            int sum= 0;
            int mean = 0;

            for (int i=0; i < WindowPoint_IDX.size();i++)
            {
                sum = sum + WindowPoint_IDX[i].x;
            }
        
            mean = (int)sum / WindowPoint_IDX.size();

            base = mean;
            linePos = linePos + base;
            linePosCount = linePosCount +1;
        }

        else
        {
        }
      }
      
      if (linePosCount == 0)
      {
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = preAngular;
  	
      }
      else
      {
          linePos = linePos/linePosCount;
          if (linePos < 150)
          {
              twist.linear.x = 0.1;
              twist.linear.y = 0.0;
              twist.linear.z = 0.0;
              twist.angular.x = 0.0;
              twist.angular.y = 0.0;
              twist.angular.z = 0.3 * (160-linePos)/160;
          }
          else if(linePos > 170)
          {
              twist.linear.x = 0.1;
              twist.linear.y = 0.0;
              twist.linear.z = 0.0;
              twist.angular.x = 0.0;
              twist.angular.y = 0.0;
              twist.angular.z = -0.3 * (linePos-160)/160;
          }
          else
          {
              twist.linear.x = 0.15;
              twist.linear.y = 0.0;
              twist.linear.z = 0.0;
              twist.angular.x = 0.0;
              twist.angular.y = 0.0;
              twist.angular.z = 0.0;
          }
          preAngular = twist.angular.z; 

      }
      
      if(control_flag == false)
      {
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;
      }
      
      pub_cmd_vel->publish(twist);      
      cvtColor(imgNormal,imgNormal,COLOR_GRAY2BGR);

      //drawRectangle
      for(int i=0;i<rectWindowInfo.size();i++)
      {
        rectangle(imgNormal, rectWindowInfo[i][0], Scalar(255,255,255), 3, LINE_AA);
      }
  
      sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgNormal).toImageMsg();
      pub.publish(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

private:
  image_transport::Publisher pub;
  image_transport::Subscriber sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
  Point points[4];
  geometry_msgs::msg::Twist twist; 
  double preAngular=0.0;
};

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  
    
    int c = getchar();  
    
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

void Control_Linetrace()
{
  while(1)
  {
    int c = getch();
    if(c==' ')
    {
      if(control_flag == true)control_flag = false;
      else control_flag = true;
    }   
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  thread key_thread(Control_Linetrace);
  rclcpp::spin(make_shared<LineTrace>());
  rclcpp::shutdown();

  return 0;
}

