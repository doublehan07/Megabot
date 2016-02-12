#include "map.h"

char map[256][256] ;

void saveMap(void){
	savePeoplePosition(ReceiveAI.ID);
}

void savePeoplePosition(char id){
	switch(id){
		case 0:
			map[ReceiveAI.PeopleA[0]][ReceiveAI.PeopleA[1]] = 1;break;
		case 1:
			map[ReceiveAI.PeopleB[0]][ReceiveAI.PeopleB[1]] = 1;break;
		default:
			break;
	}
}

