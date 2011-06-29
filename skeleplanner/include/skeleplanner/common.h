#include "CImg.h"
#include <algorithm>


#define BLACKTHRESH 764
//#define GREYTHRESH 400
#define GREYTHRESH 764
#define INF 1000000000

extern const unsigned char black[];
extern const unsigned char blue[];
extern const unsigned char red[];
extern const unsigned char green[];
extern const unsigned char white[];
extern const int bridgemap[][2];

struct Point {
  int x, y;
        
  Point(int _x, int _y) : x(_x), y(_y) {};
};
    
struct Rectangle {
  int x1, y1, x2, y2;
        
  Rectangle(int _x1, int _y1, int _x2, int _y2) : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {};
};

inline int colorSum(int x, int y, cimg_library::CImg<unsigned char> * image)
{
  return std::max(1, image->atXY(x,y,0) + image->atXY(x,y,1) + image->atXY(x,y,2));
}

inline float dist(int x1, int y1, int x2, int y2)
{
  return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}
