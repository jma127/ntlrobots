#include <algorithm>
#include <cassert>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <sys/time.h>

using namespace std;
typedef long long int lli;

#define sright 'R'
#define sleft 'L'
#define sforward 'F'
#define donothing 'N';
#define swall '#'
#define sempty '.'
#define sexit 'x'
#define snorth 'N'
#define seast 'E'
#define ssouth 'S'
#define swest 'W'
#define big 10000000
#define maxn 100000
#define debugmode 0
#define submitmode 1
#define debug(a) if (!submitmode && debugmode) fprintf a
#define asrt(a) if (!submitmode && debugmode) assert a

struct Node {
  int id;
  int x, y;
  int dir;
  int numConnected;
  int connections[3];
  int dist;
  int nextNode;
  char nextMove;
};

struct NodeContainer {
  int nodeid;
  int dist;
  bool operator< (const NodeContainer & other) const {
    return dist > other.dist;
  }
};

struct Robot {
  int id;
  int nodeid;
  bool done;
};

struct Exit {
  int id;
  int nodeids[4];
};

char dirSyms[4] = {'N', 'E', 'S', 'W'};
int addY[4] = {-1, 0, 1, 0};
int addX[4] = {0, 1, 0, -1};
int rows;
int cols;
double speed;
int numNodes;
Node nodes[maxn];
int numExits;
Exit exits[maxn];
bool isExit[maxn];
bool isWall[maxn];
int numRobots;
Robot robots[maxn];
string ans;

int getIndex (int y, int x, int dir) {
  return 4 * cols * y + 4 * x + dir;
}

void printState () {
  if (!submitmode && debugmode) {
    for (int count = 0; count < numNodes; count ++) {
      if (nodes[count].id != -1) {
        asrt((nodes[count].id == count));
        debug((stderr, "Node\t%d:\ty %d\tx %d\tdir %d\n", count, nodes[count].y, nodes[count].x, nodes[count].dir));
        debug((stderr, "\tD: %d\tN: %d\tM: %c\n", nodes[count].dist, nodes[count].nextNode, nodes[count].nextMove));
        for (int count2 = 0; count2 < nodes[count].numConnected; count2 ++)
          debug((stderr, "\tConn: %d\tid: %d\n", count2, nodes[count].connections[count2]));
      }
    }
    for (int count = 0; count < numExits; count ++) {
      asrt((exits[count].id == count));
      debug((stderr, "Exit\t%d:\tids\t%d\t%d\t%d\t%d\n", count,
             exits[count].nodeids[0],
             exits[count].nodeids[1],
             exits[count].nodeids[2],
             exits[count].nodeids[3]));
    }
    for (int count = 0; count < numRobots; count ++) {
      asrt((robots[count].id == count));
      debug((stderr, "Robot\t%d:\tid: %d\n", count, robots[count].nodeid));
    }
  }
}

void dijkstra (int start) {
  nodes[start].dist = 0;
  nodes[start].nextNode = -1;
  nodes[start].nextMove = donothing;
  priority_queue <NodeContainer> q;
  NodeContainer startContainer = {start, 0};
  q.push(startContainer);
  while (!q.empty()) {
    NodeContainer cont = q.top();
    q.pop();
    Node & node = nodes[cont.nodeid];
    int leftDir = (node.dir + 3) % 4, rightDir = (node.dir + 1) % 4;
    for (int count = 0; count < node.numConnected; count ++) {
      int newdis = node.dist + 1, newid = node.connections[count];
      if (newdis < nodes[newid].dist) {
        nodes[newid].dist = newdis;
        nodes[newid].nextNode = node.id;
        if (nodes[newid].dir == leftDir)
          nodes[newid].nextMove = sright;
        else if (nodes[newid].dir == rightDir)
          nodes[newid].nextMove = sleft;
        else
          nodes[newid].nextMove = sforward;
        NodeContainer newcont = {newid, newdis};
        q.push(newcont);
      }
    }
  }
}

string getDirections (int start) {
  string s;
  while (true) {
    char move = nodes[start].nextMove;
    int next = nodes[start].nextNode;
    if (next == -1)
      break;
    s.push_back(move);
    start = next;
  }
  return s;
}

int execDirections (int start, string directions) {
  int y = nodes[start].y, x = nodes[start].x, dir = nodes[start].dir;
  for (int count = 0; count < directions.length(); count ++) {
    if (directions[count] == sleft) {
      dir += 3;
      dir %= 4;
    }
    else if (directions[count] == sright) {
      dir ++;
      dir %= 4;
    }
    else if (directions[count] == sforward) {
      int newY = y + addY[dir], newX = x + addX[dir];
      if (!isWall[getIndex(newY, newX, 0)]) {
        y = newY;
        x = newX;
      }
    }
  }
  return getIndex(y, x, dir);
}

void solveProblem () {
  while (true) {
    int robotsLeft = numRobots;
    for (int count = 0; count < numRobots; count ++) {
      if (robots[count].done)
        robotsLeft --;
    }
    if (robotsLeft == 0)
      break;
    int bestRobot;
    string bestSequence;
    double bestTime = big;
    for (int count = 0; count < numRobots; count ++) {
      if (robots[count].done)
        continue;
      Robot & curRobot = robots[count];
      double curTime = 0.0;
      string curDirections = getDirections(curRobot.nodeid);
      curTime += curDirections.length();
      for (int count2 = 0; count2 < numRobots; count2 ++) {
        if (robots[count2].done)
          continue;
        int newPos = execDirections(robots[count2].nodeid, curDirections);
        curTime += nodes[newPos].dist;
      }
      if (curTime < bestTime) {
        bestRobot = count;
        bestSequence = curDirections;
        bestTime = curTime;
      }
    }
    for (int count = 0; count < numRobots; count ++) {
      if (robots[count].done)
        continue;
      robots[count].nodeid = execDirections(robots[count].nodeid, bestSequence);
      if (isExit[robots[count].nodeid])
        robots[count].done = true;
    }
    ans += bestSequence;
  }
}

class SynchronousControl {
public:
  string evacuateAll (vector <string> initCave, double sp) {
    rows = initCave.size();
    cols = initCave[0].length();
    speed = sp;
    for (int count = 0; count < rows; count ++) {
      for (int count2 = 0; count2 < cols; count2 ++) {
        char c = initCave[count][count2];
        if (c != swall) {
          for (int count3 = 0; count3 < 4; count3 ++) {
            int id = getIndex(count, count2, count3);
            nodes[id].id = id;
            nodes[id].y = count;
            nodes[id].x = count2;
            nodes[id].dir = count3;
            nodes[id].numConnected = 2;
            nodes[id].connections[0] = getIndex(count, count2, (count3 + 3) % 4);
            nodes[id].connections[1] = getIndex(count, count2, (count3 + 1) % 4);
            int newY = count - addY[count3], newX = count2 - addX[count3];
            if (initCave[newY][newX] != swall) {
              nodes[id].connections[2] = getIndex(newY, newX, count3);
              nodes[id].numConnected ++;
            }
            nodes[id].dist = big;
            nodes[id].nextNode = -1;
          }
          if (c == sempty);
          else if (c == sexit) {
            exits[numExits].id = numExits;
            for (int count3 = 0; count3 < 4; count3 ++) {
              int id = getIndex(count, count2, count3);
              exits[numExits].nodeids[count3] = id;
              isExit[id] = true;
            }
            numExits ++;
          }
          else {
            int dir = 0;
            if (c == seast)
              dir = 1;
            if (c == ssouth)
              dir = 2;
            if (c == swest)
              dir = 3;
            robots[numRobots].id = numRobots;
            robots[numRobots].nodeid = getIndex(count, count2, dir);
            robots[numRobots].done = false;
            numRobots ++;
          }
        }
        else {
          for (int count3 = 0; count3 < 4; count3 ++) {
            int id = getIndex(count, count2, count3);
            nodes[id].id = -1;
            isWall[id] = true;
          }
        }
        numNodes += 4;
      }
    }
    for (int count = 0; count < numExits; count ++) {
      for (int count2 = 0; count2 < 4; count2 ++)
        dijkstra(exits[count].nodeids[count2]);
    }
    printState();
    solveProblem();
    fflush(stderr);
    return ans;
  }
};

#if !submitmode
int main (int argc, char ** argv) {
  int numLines;
  scanf("%d", &numLines);
  vector <string> c;
  for (int count = 0; count < numLines; count ++) {
    string s;
    cin >> s;
    c.push_back(s);
  }
  double v;
  scanf("%lf", &v);
  SynchronousControl s;
  string returned = s.evacuateAll(c, v);
  cout << returned << endl;
  fflush(stdout);
  return 0;
}
#endif