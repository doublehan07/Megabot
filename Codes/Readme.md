#����˵��

##����ӿ�
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

##ƽ̨����
leader�ڵ�������uart��������dw&����ͨ��
�Զ��ط������У��Լ���һ�������ȱ�䣬�ٶ���һ��

tri�ڵ�������uart����dw&����ͨ��
��Ҫ��goto��������ƣ�֮��followģʽ��Ҫ֪��leader���ٶȣ���dist��Ϣ��һ������pid
dist��Ϣ������Ҫ���˲�����

����tri����collision������delay

##��Ҫ�ٶȽӿڣ���������m/s��ֵ�ģ���
����������ĵ��
���ڣ��������Сƽ̨demo������װ�����ķ����ȫ���ƶ�ƽ̨

Updated by haldak 3.3