#代码说明

##需求接口
struct MyInfo
{
  present position
  present sangle
  my id
  corperation nodes[3]
  leader id[2] //fixed by upper controller
  my status{leader, nodes, triangle}
  my destination
}

struct FriendsInfo
{
  present position
  present sangle
  id
  status
  destination
}

Goto_Polar_Axis(r,theta)
|_
    Update_My_Axis(leader flag), get 2 distance and leader's sangle, return if_success
    |_
      Transfer_Dist(void), calculate angle in polar system, get my angle(r, theta), return if_success

    Calculate_Motion(void), calculate turning angle and distance, then get speed and approximate time

    use motor_function, read systick to get time, (before finish!)

    PID_Position_Adjustion(r, theta), use PID to get the proper positon, return if_success

return if_success

##平台代码
leader节点配三个uart，和两个dw&九轴通信
自动回发测距就行，自己有一个方向，先别变，速度慢一点

tri节点配两个uart，和dw&九轴通信
主要是goto函数的设计，之后follow模式不要知道leader的速度，用dist信息（一个）做pid
dist信息自身需要有滤波机制

两个tri避免collision，考虑delay

##需要速度接口！！！给出m/s数值的！！
入带编码器的电机
后期，如果不做小平台demo，考虑装麦克纳姆轮做全向移动平台

Updated by haldak 3.3