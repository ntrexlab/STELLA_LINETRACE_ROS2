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

      //Unwarp Image
      Mat Matx;

      vector<Point2f> srcRectCoord;
      vector<Point2f> dstRectCoord;

      Point p1s = Point2f(0,160);
      Point p2s = Point2f(320,160);
      Point p3s = Point2f(-80,200);
      Point p4s = Point2f(400,200);

      Point p1d = Point2f(0, 0);
      Point p2d = Point2f(320, 0);
      Point p3d = Point2f(0, 240);
      Point p4d = Point2f(320, 240);

      srcRectCoord = {p1s, p2s, p3s, p4s};
      dstRectCoord = {p1d, p2d, p3d, p4d};

      Matx = getPerspectiveTransform(srcRectCoord,dstRectCoord);
      warpPerspective(image, image, Matx, Size(320,240), INTER_LINEAR);

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
      imgNormal = imgNormal(Range(imgNormal.rows/2, imgNormal.rows), Range(0, imgNormal.cols));
      //vector<vector<Point>> leftPixelContaioner;
      //vector<vector<Point>> rightPixelContaioner;

      vector<int> sumArray;
      for (int i=0; i < imgNormal.cols;i++)
      {
        Mat oneColumn = imgNormal.col(i);
        int sumResult = sum(oneColumn)[0];
        sumArray.push_back(sumResult);
      }

      int leftbase, rightbase;
      int midPoint = sumArray.size()/2;

      vector<int>::const_iterator begin = sumArray.begin();
      vector<int>::const_iterator last = sumArray.begin() + sumArray.size();
      vector<int> left(begin, begin + midPoint);
      vector<int> right(begin + midPoint, last);
      int leftMaxIndex = max_element(left.begin(), left.end()) - left.begin();
      int rightMaxIndex = max_element(right.begin(), right.end()) - right.begin();
      leftbase = leftMaxIndex;
      rightbase = rightMaxIndex + midPoint;
      
      int leftflag = 0;
      int rightflag = 0;

      vector<vector<Rect>> rectWindowInfo;
      Mat nonZeroPos;
      int numWindow = 10;
      int window_height = imgNormal.rows / numWindow;
      findNonZero(imgNormal,nonZeroPos);
      int win_yHigh;
      int win_yLow;
      int win_xLeft_low;
      int win_xRight_low;
      int margin = 30;
      int leftlinePos = 0;
      int leftlinePosCount = 0;
      int rightlinePos = 0;
      int rightlinePosCount = 0;
      int middlelinePos = 0;

      for (int i=0; i < numWindow; i++)
      {
        win_yHigh = imgNormal.rows - (i)*window_height;
        win_yLow = imgNormal.rows - (i+1) * window_height;
        win_xLeft_low = leftbase - margin;
        win_xRight_low = rightbase - margin;

        Rect curLeftWindow(win_xLeft_low, win_yLow, 2 * margin, window_height);
        Rect curRightWindow(win_xRight_low, win_yLow, 2 * margin, window_height);
        rectWindowInfo.push_back({curLeftWindow,curRightWindow});

        vector<Point> leftWindowPoint_IDX;
        vector<Point> rightWindowPoint_IDX;

        int left_xLow = curLeftWindow.x;
        int left_xHigh = left_xLow + curLeftWindow.width;
        int left_yLow = curLeftWindow.y;
        int left_yHigh = left_yLow + curLeftWindow.height;
        int right_xLow = curRightWindow.x;
        int right_xHigh = right_xLow + curRightWindow.width;
        int right_yLow = curRightWindow.y;
        int right_yHigh = right_yLow + curRightWindow.height;

        for (int i=0;i < nonZeroPos.rows;i++)
        {
            int nonZeroX = nonZeroPos.at<Point>(i).x;
            int nonZeroY = nonZeroPos.at<Point>(i).y;
        
            Point onePt = {0,0};
            onePt = {nonZeroX, nonZeroY};
            
            if (nonZeroX >= left_xLow && nonZeroX < left_xHigh && nonZeroY >= left_yLow && nonZeroY < left_yHigh)
            {
                leftWindowPoint_IDX.push_back(onePt);
            }
            if (nonZeroX >= right_xLow && nonZeroX < right_xHigh && nonZeroY >= right_yLow && nonZeroY < right_yHigh)
            {
                rightWindowPoint_IDX.push_back(onePt);
            }
        }

        //leftPixelContaioner.push_back(leftWindowPoint_IDX);
        //rightPixelContaioner.push_back(rightWindowPoint_IDX);
         
        if (leftWindowPoint_IDX.size() > 20)//minNumPix = 20;
        {
            int sum= 0;
            int mean = 0;

            for (int i=0; i < leftWindowPoint_IDX.size();i++)
            {
                sum = sum + leftWindowPoint_IDX[i].x;
            }
        
            mean = (int)sum / leftWindowPoint_IDX.size();

            leftbase = mean;
            leftlinePos = leftlinePos + leftbase;
            leftlinePosCount = leftlinePosCount +1;
            if(leftlinePosCount == 1)
            {
                leftflag = leftbase;
            }
        }

        else
        {
        }

        if (rightWindowPoint_IDX.size() > 20)//minNumPix = 20;
        {
            int sum= 0;
            int mean = 0;

            for (int i=0; i < rightWindowPoint_IDX.size();i++)
            {
                sum = sum + rightWindowPoint_IDX[i].x;
            }
        
            mean = (int)sum / rightWindowPoint_IDX.size();

            rightbase = mean;
            rightlinePos = rightlinePos + rightbase;
            rightlinePosCount = rightlinePosCount +1;
            if(rightlinePosCount == 1)
            {
                rightflag = rightbase;
            }            
        }

        else
        {
        }
      }
      
      if (leftlinePosCount == 0 && rightlinePosCount == 0)
      { 	  
          twist.linear.x = 0.1;
	        twist.linear.y = 0.0;
	        twist.linear.z = 0.0;
	        twist.angular.x = 0.0;
	        twist.angular.y = 0.0;
	        twist.angular.z = preAngular;
	  
      }
      else if (leftlinePosCount != 0 && rightlinePosCount == 0)
      { 	
          leftlinePos = leftlinePos/leftlinePosCount;
          //LEFT TURN
          if (leftlinePos < leftflag)
          {
              middlelinePos = leftlinePos/2;
              twist.angular.z = 0.6 * (160-middlelinePos)/160;
          }
          //RIGHT TURN
          else
          {
              middlelinePos = (leftlinePos + 320)/2;
              twist.angular.z = - 0.6 * (middlelinePos-160)/160;
          }  
          twist.linear.x = 0.1;
	        twist.linear.y = 0.0;
	        twist.linear.z = 0.0;
	        twist.angular.x = 0.0;
	        twist.angular.y = 0.0;
          preAngular = twist.angular.z;
      }
      else if (leftlinePosCount == 0 && rightlinePosCount != 0)
      { 	
          rightlinePos = rightlinePos/rightlinePosCount;
          //RIGHT TURN
          if (rightlinePos > rightflag)
          {
              middlelinePos = (rightlinePos + 320)/2;
              twist.angular.z = - 0.6 * (middlelinePos-160)/160;
          }
          //LEFT TURN
          else
          {
              middlelinePos = rightlinePos/2 ;
              twist.angular.z = 0.6 * (160-middlelinePos)/160;
          }  
          twist.linear.x = 0.1;
	        twist.linear.y = 0.0;
	        twist.linear.z = 0.0;
	        twist.angular.x = 0.0;
	        twist.angular.y = 0.0;
          preAngular = twist.angular.z;        
      }
      else
      {
          leftlinePos = leftlinePos/leftlinePosCount;
          rightlinePos = rightlinePos/rightlinePosCount;
          
          //1LINE
          if ((leftlinePosCount < 5 && rightlinePosCount < 5) || rightlinePos - leftlinePos < 150)
          {
              twist.linear.x = 0.1;
	            twist.linear.y = 0.0;
	            twist.linear.z = 0.0;
	            twist.angular.x = 0.0;
	            twist.angular.y = 0.0;
	            twist.angular.z = preAngular;
          }
          
          //2LINE
          else
          {
              middlelinePos = (leftlinePos + rightlinePos)/2;
              
              if (middlelinePos < 150)
              {
                  twist.linear.x = 0.1;
                  twist.linear.y = 0;
                  twist.linear.z = 0;
                  twist.angular.x = 0;
                  twist.angular.y = 0;
                  twist.angular.z = 0.3 * (160-middlelinePos)/160;
              }
              else if(middlelinePos > 170)
              {
                  twist.linear.x = 0.1;
                  twist.linear.y = 0;
                  twist.linear.z = 0;
                  twist.angular.x = 0;
                  twist.angular.y = 0;
                  twist.angular.z = - 0.3 * (middlelinePos-160)/160;
              }
              else
              {
                  twist.linear.x = 0.1;
                  twist.linear.y = 0;
                  twist.linear.z = 0;
                  twist.angular.x = 0;
                  twist.angular.y = 0;
                  twist.angular.z = 0;
              }
	            preAngular = twist.angular.z;
          }
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
      sleep(0.1);
      cvtColor(imgNormal,imgNormal,COLOR_GRAY2BGR);

      //drawRectangle
      for(int i=0;i<rectWindowInfo.size();i++)
      {
        rectangle(imgNormal, rectWindowInfo[i][0], Scalar(255,255,255), 3, LINE_AA);
        rectangle(imgNormal, rectWindowInfo[i][1], Scalar(255,255,255), 3, LINE_AA);
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

