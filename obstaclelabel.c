#include "obstaclelabel.h"
//#include ""
//static Environment GlobalEnv;
static int *result;

static void *Malloc(int size);
static void InitConnectInfo(struct BwConnectMem *ConnectInfo, const int width, const int height, const int mode);
static struct PixelLists *AddConnGroup(struct BwConnectMem *ConnectInfo);
static void AddConnMem(int x, int y, struct PixelLists *CurrentConnGroup);
static void RegionGrow(const int iExpandCell, const int jExpandCell, const Environment GlobalEnv, struct BwConnectMem *ConnectInfo, struct PixelLists *CurrentConnGroup);
static int IsValidObstacle(Environment GlobalEnv, int index_x, int index_y);
static int IsNeighbourAllObstacle(const Environment GlobalEnv, int index_x, int index_y);
static void PrintAllPixelLists(struct PixelLists *ConnectGroup, const int ConnCounts);


struct BwConnectMem
{
  int ConnKind;
  int ImageWidth;
  int ImageHeight;
  int ObjectCounts;
  struct PixelLists *ConnectGroup;
};

/*元素为链表的链表*/
struct PixelLists
{
  int CurrentObjectCounts;
  struct SinglePixelList *ConnectMem;
  struct PixelLists *next;
};

/*简单的链表*/
struct SinglePixelList
{
  int cor_x;
  int cor_y;
  struct SinglePixelList *next;
};

static void *Malloc(int size)
{
  void *MallocResult;

  MallocResult = malloc(size);
  if (MallocResult == NULL) {
    fprintf(stderr, "Out of space");
    exit(0);
  } else {
    return MallocResult;
  }
}

/* whether valid and is a obstacle */
static int IsValidObstacle(Environment GlobalEnv, int index_x, int index_y)
{
  if (IsValidEnvPoint(GlobalEnv, index_x, index_y))
    return IsEnvObstacle(GetEnvMember(GlobalEnv, index_x, index_y));
  else
    return 0;
}

static int IsNeighbourAllObstacle(const Environment GlobalEnv, int index_x, int index_y)
{
  int IsObstacle;

  IsObstacle = IsValidObstacle(GlobalEnv, index_x + 1, index_y) && IsValidObstacle(GlobalEnv, index_x - 1, index_y) && IsValidObstacle(GlobalEnv, index_x, index_y + 1) && IsValidObstacle(GlobalEnv, index_x, index_y - 1);
  return IsObstacle;
}

BwConnMems LabObstacleborders(const Environment GlobalEnv, const int mode)
{
  int validmemcount;
  struct BwConnectMem *labels;
  struct PixelLists *TempConnGroup = NULL;
  struct SinglePixelList *TempConnMem = NULL;
  struct SinglePixelList *PrevConnMem = NULL;

  labels = LabObstacles(GlobalEnv, mode);
  for (TempConnGroup=labels->ConnectGroup; TempConnGroup != NULL; TempConnGroup = TempConnGroup->next) {
    for (validmemcount=0,TempConnMem=TempConnGroup->ConnectMem; TempConnMem != NULL;) {
      if (IsNeighbourAllObstacle(GlobalEnv, TempConnMem->cor_x, TempConnMem->cor_y)) {
        /* if the Mem needs to be removed */
        if (TempConnMem == TempConnGroup->ConnectMem) {
          /* if the Mem needs to be removed is the first */
          TempConnGroup->ConnectMem = TempConnMem->next;
          free(TempConnMem);
          TempConnMem = TempConnGroup->ConnectMem;
        } else {
          PrevConnMem->next = TempConnMem->next;
          free(TempConnMem);
          TempConnMem = PrevConnMem->next;
        }
      } else {                  /* if it is a valid Mem */
        if(TempConnMem == TempConnGroup->ConnectMem) /* if it is the valid first Mem */
          PrevConnMem = TempConnMem;
        else
          PrevConnMem = PrevConnMem->next;
        TempConnMem = TempConnMem->next;
        validmemcount++;
      }
    }
    TempConnGroup->CurrentObjectCounts = validmemcount;
  }

  return labels;
}

static void InitConnectInfo(struct BwConnectMem *ConnectInfo, const int width, const int height, const int mode)
{
  ConnectInfo->ConnKind = mode;
  ConnectInfo->ImageWidth = width;
  ConnectInfo->ImageHeight = height;
  ConnectInfo->ObjectCounts = 0;
  ConnectInfo->ConnectGroup = NULL;
}

void DeleteLabelRes(BwConnMems ConnectInfo)
{
  struct PixelLists *TempConnectGroup, *CurrentConnectGroup;
  struct SinglePixelList *TempConnectMem, *CurrentConnectMem;

  for (CurrentConnectGroup=ConnectInfo->ConnectGroup; CurrentConnectGroup != NULL; ) {
    for (CurrentConnectMem=CurrentConnectGroup->ConnectMem; CurrentConnectMem != NULL; ) {
      TempConnectMem = CurrentConnectMem;
      CurrentConnectMem = CurrentConnectMem->next;
      free(TempConnectMem);
    }
    TempConnectGroup = CurrentConnectGroup;
    CurrentConnectGroup = CurrentConnectGroup->next;
    free(TempConnectGroup);
  }
  free(ConnectInfo);
}

/* New Group is insert in the front */
static struct PixelLists *AddConnGroup(struct BwConnectMem *ConnectInfo)
{
  struct PixelLists *NewConnectGroup;
  struct PixelLists *TempConnectGroup;

  NewConnectGroup = Malloc(sizeof(struct PixelLists));
  ConnectInfo->ObjectCounts += 1;
  TempConnectGroup = ConnectInfo->ConnectGroup;
  ConnectInfo->ConnectGroup = NewConnectGroup;
  NewConnectGroup->next = TempConnectGroup;
  NewConnectGroup->ConnectMem = NULL;
  NewConnectGroup->CurrentObjectCounts = 0;
  return NewConnectGroup;
}

/* New Mem is inserted in the front */
static void AddConnMem(int x, int y, struct PixelLists *CurrentConnGroup)
{
  struct SinglePixelList *TempConnectMem;
  struct SinglePixelList *NewConnectMem;

  NewConnectMem = Malloc(sizeof(struct SinglePixelList));
  CurrentConnGroup->CurrentObjectCounts += 1;
  TempConnectMem = CurrentConnGroup->ConnectMem;
  CurrentConnGroup->ConnectMem = NewConnectMem;
  NewConnectMem->cor_x = x;
  NewConnectMem->cor_y = y;
  NewConnectMem->next = TempConnectMem;
}

static void RegionGrow(const int iExpandCell, const int jExpandCell, const Environment GlobalEnv, struct BwConnectMem *ConnectInfo, struct PixelLists *CurrentConnGroup)
{
  int width = ConnectInfo->ImageWidth;
  int height = ConnectInfo->ImageHeight;
  int mode = ConnectInfo->ConnKind;
  int ConnCount = ConnectInfo->ObjectCounts;

  int xFourConn[4] = {-1, 0, 1, 0};
  int yFourConn[4] = {0, 1, 0, -1};
  int xEightConn[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  int yEightConn[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  int *x,*y,end = mode;
  x = (int*)Malloc(sizeof(int) * mode);
  memset(x,0,sizeof(int) * mode);

  y = (int*)Malloc(sizeof(int) * mode);
  memset(y,0,sizeof(int) * mode);

  if( 4 == mode) {
      memcpy( x,xFourConn,sizeof(int) * mode);
      memcpy( y,yFourConn,sizeof(int) * mode);
    }
  else if( 8 == mode) {

      memcpy( x,xEightConn,sizeof(int) * mode);
      memcpy( y,yEightConn,sizeof(int) * mode);
    }
  else {
      printf("mode is error\n");
    }
  //定义堆栈的起点和终点
  int nStart;
  int nEnd;
  //当前正在处理像素
  int nCurrX;
  int nCurrY;
  //定义堆栈存储坐标
  int *pnGrowQueX;
  int *pnGrowQueY;
  //图像的横纵坐标，用来对当前的4邻域进行遍历
  int xx;
  int yy;
  int k;

  pnGrowQueX = (int *)Malloc(width * height * sizeof(int));
  pnGrowQueY = (int *)Malloc(width * height * sizeof(int));

  nStart = 0;
  nEnd = 0;


  pnGrowQueX[nEnd] = jExpandCell;
  pnGrowQueY[nEnd] = iExpandCell;

  while(nStart <= nEnd) {
    nCurrX = pnGrowQueX[nStart];
    nCurrY = pnGrowQueY[nStart];

    for(k = 0; k < end; k ++) {
      xx = nCurrX + x[k];
      yy = nCurrY + y[k];
      //判断当前邻域中的点是否在图像内部，并且是否在连通域内，以及是否被标记过
      if(xx < width && xx >= 0 && yy < height && yy >= 0 && IsEnvObstacle(GetEnvMember(GlobalEnv, xx, yy)) && *(result + yy * width + xx) == 0) {
        AddConnMem(xx, yy, CurrentConnGroup);
        *(result + yy * width + xx) = ConnCount;
        nEnd++;
        pnGrowQueX[nEnd] = xx;
        pnGrowQueY[nEnd] = yy;

      }
    }
    nStart++;

  }

  free(pnGrowQueX);
  free(pnGrowQueY);
  pnGrowQueX = NULL;
  pnGrowQueY = NULL;
}


BwConnMems LabObstacles(const Environment GlobalEnv, const int mode)
{
  struct BwConnectMem *ConnectInfo;
  struct PixelLists *NewConnectGroup = NULL;

  int width = GetEnvironmentLength(GlobalEnv);
  int height = GetEnvironmentHeight(GlobalEnv);

  int i = 0;
  int j = 0;
  ConnectInfo = Malloc(sizeof(struct BwConnectMem));

  result = (int *)malloc(width * height * sizeof(int));
  memset(result,0,width * height * sizeof(int));

  InitConnectInfo(ConnectInfo, width, height, mode);
  for(i = 0; i < height; i ++) {
    for(j = 0; j < width; j++) {

      /* i,j 在联通域内，并且没被标记过 */
      /* The j, i is reversed? */
      if(IsEnvObstacle(GetEnvMember(GlobalEnv, j, i)) && *(result + i * width + j) == 0) {
        NewConnectGroup = AddConnGroup(ConnectInfo);
        *(result + i * width + j) = ConnectInfo->ObjectCounts;
        AddConnMem(j, i, NewConnectGroup);
        RegionGrow(i, j, GlobalEnv, ConnectInfo, NewConnectGroup);
      }
    }
  }

  free(result);
  return ConnectInfo;
}

static void PrintAllPixelLists(struct PixelLists *ConnectGroup, const int ConnCounts)
{
  int i;
  struct PixelLists *TempConnGroup = NULL;
  struct SinglePixelList *TempConnMem = NULL;

  for (i=1,TempConnGroup=ConnectGroup; TempConnGroup != NULL; TempConnGroup = TempConnGroup->next, i++) {
    printf("The %d class with %d Mems:", i, TempConnGroup->CurrentObjectCounts);
    for (TempConnMem=TempConnGroup->ConnectMem; TempConnMem != NULL; TempConnMem = TempConnMem->next) {
      printf (" Cor is: (%d, %d) ", TempConnMem->cor_x, TempConnMem->cor_y);
    }
    printf("\n");
  }
}

void PrintAllConnInfo(const BwConnMems ConnectInfo)
{
  if (ConnectInfo != NULL) {
    printf("Kind of Connectivity:%d\n", ConnectInfo->ConnKind);
    printf("ImageWidth:%d\n", ConnectInfo->ImageWidth);
    printf("ImageHeight:%d\n", ConnectInfo->ImageHeight);
    printf("ObjectCounts:%d\n", ConnectInfo->ObjectCounts);
    PrintAllPixelLists(ConnectInfo->ConnectGroup, ConnectInfo->ObjectCounts);
  }
}
