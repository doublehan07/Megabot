#´úÂëËµÃ÷

##ÐèÇó½Ó¿Ú
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

##Æ½Ì¨´úÂë
leader½ÚµãÅäÈý¸öuart£¬ºÍÁ½¸ödw&¾ÅÖáÍ¨ÐÅ
×Ô¶¯»Ø·¢²â¾à¾ÍÐÐ£¬×Ô¼ºÓÐÒ»¸ö·½Ïò£¬ÏÈ±ð±ä£¬ËÙ¶ÈÂýÒ»µã

tri½ÚµãÅäÁ½¸öuart£¬ºÍdw&¾ÅÖáÍ¨ÐÅ
Ö÷ÒªÊÇgotoº¯ÊýµÄÉè¼Æ£¬Ö®ºófollowÄ£Ê½²»ÒªÖªµÀleaderµÄËÙ¶È£¬ÓÃdistÐÅÏ¢£¨Ò»¸ö£©×öpid
distÐÅÏ¢×ÔÉíÐèÒªÓÐÂË²¨»úÖÆ

Á½¸ötri±ÜÃâcollision£¬¿¼ÂÇdelay

##ÐèÒªËÙ¶È½Ó¿Ú£¡£¡£¡¸ø³öm/sÊýÖµµÄ£¡£¡
Èë´ø±àÂëÆ÷µÄµç»ú
ºóÆÚ£¬Èç¹û²»×öÐ¡Æ½Ì¨demo£¬¿¼ÂÇ×°Âó¿ËÄÉÄ·ÂÖ×öÈ«ÏòÒÆ¶¯Æ½Ì¨

Updated by haldak 3.3