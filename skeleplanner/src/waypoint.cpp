/* (c) Peter Neufeld 2011 */

#include "skeleplanner/waypoint.h"

#include <iostream>
#include <cstring>

using namespace std;

#define WAYPOINTRAD 3
//#define WAYPOINTSPACE 40
#define WAYPOINTSPACE 20

#define IMPASSABLE_THRESH 127
#define PASSABLE_THRESH 127

inline float dist(int x1, int y1, int x2, int y2)
{
  return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

inline bool inBounds(int x, int y, const costmap_2d::Costmap2D &costmap) {
  return x >= 0 && y >= 0 && x < costmap.getSizeInCellsX() && y < costmap.getSizeInCellsY();
}

//calculate the minimum space around a point
int calcSpace(int x, int y, const costmap_2d::Costmap2D &costmap, int ** memo)
{
  if(memo != NULL && memo[x][y] != -1)
    return memo[x][y];

  int rtn = INT_MAX;
  int startrad = 0;
    
  //DPify!
  if(memo != NULL && false)
  {
    //note: int/float dist trunc. errors mean be conservative by 3(more?)
    startrad = INT_MAX;
    for(int i = -1; i <= 1; i++)
      for(int j = -1; j <= 1; j++) {
        if(!inBounds(x+i, y+j, costmap))
          continue;
        if((i != 0 || j != 0) && memo[x+i][y+j] != -1) {
          startrad = min(startrad, memo[x+i][y+j]-3);
        }
      }

      if(startrad == INT_MAX)
        startrad = 0;
      startrad = max(startrad, 0);
  }
    
  for(int rad = startrad; rad < INT_MAX; rad++)
  {
      for(int i = x - rad; i <= x + rad; i++)
      {
        for(int j = y - rad; j <= y + rad; j += (i == x-rad || i == x+rad ? 1 : max(1, 2*rad)))
        {
          if(!inBounds(i, j, costmap))
            continue;
          if(costmap.getCost(i, j) > IMPASSABLE_THRESH)
            rtn = min((float)rtn, dist(i, j, x, y));
        }
      }
        
      if(rtn != INT_MAX && rad > rtn) {
        if(memo != NULL)
          memo[x][y] = rtn;
        return rtn;
      }
  }
  return rtn;
}

bool straightClear(int x1, int y1, int x2, int y2, const costmap_2d::Costmap2D &costmap, int ** memo)
{
  if(x2-x1 == 0)
  {
    //annoying vertical line
    int yinc = (y2-y1 > 0 ? 1 : -1);
    for(int y = y1; y != y2; y += yinc)
    {
      if(calcSpace(x1, y, costmap, memo) <= 1)
        return false;
    }
    return true;
  }
    
  //ensure left-to-right
  if(x2-x1 < 0) {
    int tx = x1;
    int ty = y1;
    x1 = x2;
    y1 = y2;
    x2 = tx;
    y2 = ty;
  }

  float slope = (float)(y2-y1)/(x2-x1);
  float xinc = 1;
  if(abs(slope) > 1)
    xinc = abs(1.0 / slope);

  //dont check the points themselves (?)
  for(float x = 0; x <= (x2-x1); x += xinc)
  {
    if(calcSpace((int)(x1+x), (int)(y1+(slope*x)), costmap, memo) <= 1)
    {
      return false;
    }
  }

  return true;
}

MapWaypoint waypointBest(int x, int y, int ** memo, const costmap_2d::Costmap2D &costmap, const vector<MapWaypoint*> &waypoints)
{
  int newm = -1;
  int newx = -1;
  int newy = -1;

  for(int rad = WAYPOINTSPACE-1; rad >= 0; rad--)
  {
    for(int i = x - rad; i <= x + rad; i++)
    {
      for(int j = y - rad; j <= y + rad; j += (i == x-rad || i == x+rad ? 1 : max(1, 2*rad)))
      {
        if(!inBounds(i, j, costmap))
          continue;

        if(dist(i, j, x, y) > WAYPOINTSPACE-1)
          continue;

        //make sure a straight line clear
        if(!straightClear(i, j, x, y, costmap, memo))
          continue;

        //straightline clear on orig image
        int tmps = calcSpace(i, j, costmap, memo);
        int otmps = tmps;
        int innerWaypoints = 0;
        for(vector<MapWaypoint*>::const_iterator w = waypoints.begin(); w != waypoints.end(); w++)
        {
          tmps = min(tmps, (int)(dist(i, j, (*w)->x, (*w)->y)));
          if(dist(i, j, (*w)->x, (*w)->y) <= WAYPOINTSPACE)
            innerWaypoints++;
        }
                  
        //don't use new waypoints that are clustered around current ones
        if(innerWaypoints >= 2)
          continue;
                      
        if(tmps > newm)
        {
          newm = tmps;
          newx = i;
          newy = j;

          if(newm >= rad)
            break;
        }
      }

      if(newm >= rad)
        break;
    }
    if(newm >= rad)
      break;
  }
    
  if(newm == -1)
    return MapWaypoint(-1, -1, -1);

  MapWaypoint rtn(newx, newy, newm);

  return rtn;
} 

//this function takes a starting point and creates a topological map of the image from that point.
//the function stops when it cannot make a new waypoint without placing one in an area already covered by waypoints
//however you can stop is short by specifying a maxPoints. Make maxPoints large (INT_MAX) if you want it to find
//the complete topological map.
//The other functions are workhorse functions, and are not meant for use outside of this function.
vector<Waypoint*> * topoFromPoint(double worldx, double worldy, const costmap_2d::Costmap2D &costmap, bool showDebug, Topostore * memory)
{
  unsigned x, y;
  MapWaypoint * home;
  vector<MapWaypoint*> *rtn;
  vector<Waypoint*> *worldRtn;
  vector<bool> * ignore;
  int ** memo;
  
  //initially, give memory a NULL home (indeed, NULL everything) to have it initialized
  if(memory == NULL || memory->home == NULL)
  {
    costmap.worldToMap(worldx, worldy, x, y);
    home = new MapWaypoint(x, y, 0);
    rtn = new vector<MapWaypoint*>();
    worldRtn = new vector<Waypoint*>();
    rtn->push_back(home);
    worldRtn->push_back(new Waypoint(worldx, worldy, 0));
    ignore = new vector<bool>();
    ignore->push_back(false);

    //for the DP
    memo = new int*[costmap.getSizeInCellsX()];
    for(int i = 0; i < costmap.getSizeInCellsX(); i++)
    {
      memo[i] = new int[costmap.getSizeInCellsY()];
      for(int j = 0; j < costmap.getSizeInCellsY(); j++)
        memo[i][j] = -1;
    }
    
    if(memory != NULL)
    {
        memory->home = home;
        memory->rtn = rtn;
        memory->worldRtn = worldRtn;
        memory->ignore = ignore;
        memory->memo = memo;
    }
  }else{
    home = memory->home;
    rtn = memory->rtn;
    worldRtn = memory->worldRtn;
    ignore = memory->ignore;
    memo = memory->memo;
  }

  //add waypoints
  for(unsigned i = 0; true; ++i)
  {
    if(showDebug && (i+1) % 5 == 0)
      cout << "Finding Waypoint " << (i+1) << "..." << endl;

    MapWaypoint *maxWaypoint = NULL;
    Waypoint *worldMax = NULL;
    MapWaypoint *newway = new MapWaypoint(0, 0, 0);
    Waypoint *newworld = new Waypoint(0, 0, 0);
    while(maxWaypoint == NULL)
    {
      int bestx, besty, besti;
      //always find new max, as new waypoints can change the max. Assume [0]==home
      for(int j = 0; j < rtn->size(); j++)
      {
        MapWaypoint * tmp = (*rtn)[j];
        Waypoint *worldtmp = (*worldRtn)[j];
        //if(!ignore[j] && (maxWaypoint == NULL || (tmp->space >= maxWaypoint->space && costmap.getCost(tmp->x, tmp->y) < PASSABLE_THRESH)))
        if(!(*ignore)[j] && (maxWaypoint == NULL || (tmp->space >= maxWaypoint->space && costmap.getCost(tmp->x, tmp->y) < PASSABLE_THRESH)))
        {
          maxWaypoint = tmp;
          worldMax = worldtmp;
          besti = j;
        }
      }
          
      //no more space?
      if(maxWaypoint == NULL)
        break;
                  
      *newway = waypointBest(maxWaypoint->x, maxWaypoint->y, memo, costmap, *rtn);
      if(newway->x == -1)
      {
        (*ignore)[besti] = true;
        maxWaypoint = NULL;
        worldMax = NULL;
        continue;
      }
    }
        
    if(maxWaypoint == NULL)
      break;
                
    rtn->push_back(newway);
    newworld->space = newway->space;
    costmap.mapToWorld(newway->x, newway->y, newworld->x, newworld->y);
    worldRtn->push_back(newworld);
    ignore->push_back(false);

    //only recalc waypointBest for nearby waypoints
    for(int j = 1; j < rtn->size()-1; j++)
    {
      MapWaypoint * tmp = (*rtn)[j];

      if(tmp->x == newway->x && tmp->y == newway->y)
      {
        if(showDebug)
          cout << "WARNING: waypoint overlap" << endl;
        continue;
      }

      if(dist(tmp->x, tmp->y, newway->x, newway->y) <= WAYPOINTSPACE)
        tmp->space = waypointBest(tmp->x, tmp->y, memo, costmap, *rtn).space;
    }

    //debug
    //GUIimage->draw_line(maxWaypoint->x, maxWaypoint->y, newway->x, newway->y, green, 0.5);
    maxWaypoint->neighbors.push_back(newway);
    newway->neighbors.push_back(maxWaypoint);
    worldMax->neighbors.push_back(newworld);
    newworld->neighbors.push_back(worldMax);

    //if we've hit our frontier, stop finding waypoints.
    //if(colorSum(newway->x, newway->y, &image) < GREYTHRESH && colorSum(newway->x, newway->y, &image) > BLACKTHRESH)

  }

/*
  XXX Do we now have a memory leak with this corrupted?

  for(std::vector<MapWaypoint*>::iterator i = rtn->begin(); i != rtn->end(); ++i) {
    delete *i;
  }
  //delete rtn;
  */

  return worldRtn;
}

//workhorse function for frontierRatings
Line leastSquares(vector<Waypoint*> points)
{
    int n = points.size();
    double sum_x = 0;
    double sum_y = 0;
    double sum_xx = 0;
    double sum_xy = 0;
    
    for(vector<Waypoint*>::iterator i = points.begin(); i != points.end(); i++)
    {
        sum_x = sum_x + (*i)->x;
        sum_y = sum_y + (*i)->y;

        sum_xx = sum_xx + ((*i)->x * (*i)->x);
        sum_xy = sum_xy + ((*i)->x * (*i)->y);
    }
    
    double m = (-sum_x*sum_y+n*sum_xy)/(n*sum_xx-sum_x*sum_x);
    double b = (-sum_x*sum_xy+sum_xx*sum_y)/(n*sum_xx-sum_x*sum_x);

    return Line(m, b);
}

//workhorse function for frontierRatings
Point openPoint(WeightedFrontier frontier, const costmap_2d::Costmap2D &costmap)
{
    int x = frontier.frontier.pose.position.x;
    int y = frontier.frontier.pose.position.y;
    
    for(int rad = 0; rad <= frontier.frontier.size; rad++)
    {
        for(int i = x - rad; i <= x + rad; i++)
        {
            for(int j = y - rad; j <= y + rad; j += (i == x-rad || i == x+rad ? 1 : max(1, 2*rad)))
            {
                int tx, ty;
                costmap.worldToMap(i, j, tx, ty);
                //if(calcSpace(i, j, image, NULL) > 1)
                if(costmap.getCost(tx, ty) < PASSABLE_THRESH)
                    return Point(i, j);
            }
        }
    }
    
    //give up and return the center
    return Point(x, y);
}

//workhorse function for frontierRatings
inline double perpenDist(double m, double b, double x, double y)
{
    return abs(y - (m*x) - b) / sqrt((m*m) + 1);
}

//this is the this least reliable, but the fastest. I have not copied to slower and more reliable version.
vector<FrontierStats*> *frontierRatings(vector<WeightedFrontier> frontiers, const costmap_2d::Costmap2D &costmap, vector<Waypoint*> topo, int showDebug)
{
    int counter = 1;
    
    vector<FrontierStats*> * rtn = new vector<FrontierStats*>();
    
    for(vector<WeightedFrontier*>::iterator i = frontiers.begin(); i != frontiers.end(); i++)
    {
        if(showDebug >= 1)
            cout << "Analyzing frontier " << counter++ << "/" << frontiers.size() << endl;
    
        WeightedFrontier frontier = (*i);
        Point startingPoint = openPoint(frontier, costmap);
        
        double bestDist = INF;
        Waypoint * best = NULL;
        //find the closest waypoint
        for(vector<Waypoint*>::iterator j = topo.begin(); j != topo.end(); j++)
        {
            if((*j)->x == startingPoint.x && (*j)->y == startingPoint.y)
                continue;
        
            if(dist((*j)->x, (*j)->y, startingPoint.x, startingPoint.y) < bestDist)
            {
                bestDist = dist((*j)->x, (*j)->y, startingPoint.x, startingPoint.y);
                best = (*j);
            }
        }
        
        vector<Waypoint*> waypoints;
        queue<Waypoint*> bfs;
        bfs.push(best);
        //bfs from that waypoint
        for(int j = 1; j < FRONTIERDEPTH && !bfs.empty(); )
        {
            Waypoint * current = bfs.front();
            bfs.pop();
            
            for(vector<Waypoint*>::iterator k = current->neighbours.begin(); k != current->neighbours.end(); k++)
            {
                bool alreadyVisited = false;
                for(vector<Waypoint*>::iterator m = waypoints.begin(); m != waypoints.end(); m++)
                    if((*m) == (*k))
                    {
                        alreadyVisited = true;
                        break;
                    }
                
                if(!alreadyVisited)
                {
                    j++;
                    waypoints.push_back(*k);
                    bfs.push(*k);
                }
            }
        }
        
        Line bestFit = leastSquares(waypoints);

        double frontierDelta = perpenDist(bestFit.m, bestFit.b, startingPoint.x, startingPoint.y);
                
        double lineDeltas = 0.0;
        //do not include the home waypoint, or else we double count deltas
        for(vector<Waypoint*>::iterator j = waypoints.begin() + 1; j != waypoints.end(); j++)
            lineDeltas += perpenDist(bestFit.m, bestFit.b, (*j)->x, (*j)->y);
        lineDeltas /= (waypoints.size() - 1);
            
        //so atm, frontier delta is just as weighted as line deltas
        rtn->push_back(new FrontierStats(frontierDelta, lineDeltas));
        
    }
    
    return rtn;
}



