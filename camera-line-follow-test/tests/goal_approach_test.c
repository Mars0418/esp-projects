#include "quarter_goal_pose.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned char image[38400];
static void read_frame(const char *path){FILE*f=fopen(path,"rb");assert(f);assert(fread(image,1,sizeof(image),f)==sizeof(image));fclose(f);}
int main(int argc,char**argv){
    assert(argc==3); black_marker_result_t goal;quarter_goal_pose_result_t corner;
    black_marker_vision_init(160,120);black_marker_vision_set_logging(false);
    quarter_goal_pose_set_single_corner_mode(true);quarter_goal_pose_set_logging(false);
    read_frame(argv[1]);black_marker_vision_process(image,160,120,&goal);
    quarter_goal_pose_process(image,160,120,&goal,&corner);
    assert(corner.found && corner.position_valid && !corner.odometry_predicted && corner.confidence>=60);
    assert(abs(corner.origin_x_raw-79)<=2 && abs(corner.origin_y_raw-100)<=2);
    float bx,by,gap;
    assert(quarter_goal_pose_project_ground_pixel(81,48,&bx,&by));
    assert(quarter_goal_pose_ball_gap(image,160,120,&goal,bx,by,&gap) && gap>200);
    read_frame(argv[2]);black_marker_vision_process(image,160,120,&goal);
    assert(quarter_goal_pose_project_ground_pixel(60,0,&bx,&by));
    assert(quarter_goal_pose_ball_gap(image,160,120,&goal,bx,by,&gap) && gap<=120 && gap>0);
    assert(!quarter_goal_pose_ball_gap(image,160,120,&goal,bx+1000,by,&gap));
    goal.predicted=true;assert(!quarter_goal_pose_ball_gap(image,160,120,&goal,bx,by,&gap));
    puts("PASS: actual corner fit, far/no-stop, near/stop, sideways exclusion and no predicted goal");
}
