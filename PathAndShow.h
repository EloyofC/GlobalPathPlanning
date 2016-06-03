/*
  Descripition : This file is the interface between pathplanning module and GUI part
  Author : Green
  Date : 16/06/02
*/

struct Obstacles
{
  int obstaclecounts;
  struct SingleObstacle *obstaclemembers;
};

struct SingleObstacle
{
  int vertexcounts;
  struct PointCoor *points;
};

struct PointCoor
{
  int cor_x;
  int cor_y;
};
