#include <limits>
#include <iostream>
#include <cstdlib>  // atoi

#include "Maze.h"
#include "MazeDefinitions.h"
#include "PathFinder.h"
#include "BitVector256.h"

#include <string>
#include <stack>
using namespace std;

const unsigned midp = MazeDefinitions::MAZE_LEN / 2;
const unsigned param = MazeDefinitions::MAZE_LEN;
const int MAX =10000000;
class Location
{
public:
    Location() {
        x = 0;
        y = 0;
    }
    Location(const Location& loc)
    {
        x = loc.x;
        y = loc.y;
    }

    int getx()
    {
     return x;
    }
    int gety() {
        return y;
    }
    void setx(int a)
    {
        x=a;
    }
    void sety(int b)
    {
        y=b;
    }
private:
    int x;
    int y;
};


int manhattanDistance(Location a, Location b)
{
    int x = a.getx() - b.getx();
    int y = a.gety() - b.gety();
    
    int dis = abs(x)+abs(y);
    //cout << "x: " << abs(x) << " y: " << abs(y) << endl;
    //cout << "manhattan distance: " << dis << endl;
    
    return abs(x)+abs(y);
}


class Floodfill : public PathFinder {
public:
    Floodfill(bool shouldPause = false) : pause(shouldPause) {
        shouldGoForward = false;
        visitedStart = false;
        //shortest.setmdis(2*MazeDefinitions::MAZE_LEN);
        travel =1;
        initiation();
        for(int i=0;i<17;i++)
        {
            for(int j=0;j<17;j++)
            {
                NSwall[i][j]=0;
                EWwall[i][j]=0;
            }
        }
        NSwall[0][0]=1;
        currentDirection=NORTH;
        leavingcenter=false;
        leaveorigin=true;
        

    }
    
    int actualDistance (Location a)
    {
        if(travel%2==1)
        {if(a.getx()<MazeDefinitions::MAZE_LEN/2&&a.gety()<MazeDefinitions::MAZE_LEN/2&&a.getx()>=0&&a.gety()>=0)
        {
            Location center1;
            center1.setx(MazeDefinitions::MAZE_LEN/2-1);
            center1.sety(MazeDefinitions::MAZE_LEN/2-1);
            return manhattanDistance(a, center1);
        }
        else if(a.getx()>=0&&a.getx()<MazeDefinitions::MAZE_LEN/2&&a.gety()>=MazeDefinitions::MAZE_LEN/2&&a.gety()<MazeDefinitions::MAZE_LEN)
        {
            Location center2;
            center2.setx(MazeDefinitions::MAZE_LEN/2-1);
            center2.sety(MazeDefinitions::MAZE_LEN/2);
            return manhattanDistance(a, center2);
        }
        else if(a.getx()>=MazeDefinitions::MAZE_LEN/2&&a.getx()<MazeDefinitions::MAZE_LEN&&a.gety()>=0&&a.gety()<MazeDefinitions::MAZE_LEN/2)
        {
            Location center3;
            center3.setx(MazeDefinitions::MAZE_LEN/2);
            center3.sety(MazeDefinitions::MAZE_LEN/2-1);
            return manhattanDistance(a, center3);
        }
        else //this assume that a is valid
        {
            Location center4;
            center4.setx(MazeDefinitions::MAZE_LEN/2);
            center4.sety(MazeDefinitions::MAZE_LEN/2);
            return manhattanDistance(a, center4);
        }
        }
        else
        {
            Location origin;
            origin.setx(0);
            origin.sety(0);
            return manhattanDistance(a, origin);
        }
    }

    void initiation()
    {
        for (int i = 0; i < MazeDefinitions::MAZE_LEN; i++) {
            for (int j = 0; j < MazeDefinitions::MAZE_LEN; j++) {
                Location a;
                a.setx(i),a.sety(15-j);
                dist[i][15-j] = actualDistance(a);
            }
        }
        for(int i=0;i<17;i++)
        {
            for(int j=0;j<17;j++)
            {
                fill[i][j]=false;
            }
        }


    }
    void reinitiation()
    {
        for (int i = 0; i < MazeDefinitions::MAZE_LEN; i++) {
            for (int j = 0; j < MazeDefinitions::MAZE_LEN; j++) {
                Location a;
                a.setx(i),a.sety(15-j);
                dist[15-i][15-j] = actualDistance(a);
            }
        }
        for(int i=0;i<17;i++)
        {
            for(int j=0;j<17;j++)
            {
                fill[i][j]=false;
            }
        }


    }
    
    //it's recommended that we put the stack as a global variable since it can help to save memory
    void update(Location a)
    {
        stack<Location> position;
        position.push(a);
        //first make sure that the stack is empty
        while (!position.empty()) {
            Location cur = position.top();
            position.pop();
            int shortest = MAX;
            int x_=MazeDefinitions::MAZE_LEN-1-cur.gety();
            int y_=cur.getx();
            fill[x_][y_] = true;
            if(dist[x_][y_]==0)
                continue;
            Location NeighborNorth;
            NeighborNorth.setx(cur.getx());
            NeighborNorth.sety(cur.gety()+1);
            Location NeighborSouth;
            NeighborSouth.setx(cur.getx());
            NeighborSouth.sety(cur.gety()-1);
            Location NeighborEast;
            NeighborEast.setx(cur.getx()+1);
            NeighborEast.sety(cur.gety());
            Location NeighborWest;
            NeighborWest.setx(cur.getx()-1);
            NeighborWest.sety(cur.gety());
            if(NSwall[cur.getx()][cur.gety()+1]==0)
            {
                if(dist[x_-1][y_]<shortest&&x_>=1&&x_<17&&y_>=0&&y_<16)
                {
                    shortest=dist[x_-1][y_];
                    if(fill[x_-1][y_]==false)
                    {
                        position.push(NeighborNorth);
                    }
                }
            }
            if(NSwall[cur.getx()][cur.gety()]==0)
            {
                if(dist[x_+1][y_]<shortest&&x_>=0&&x_<15&&y_>=0&&y_<16)
                {   shortest=dist[x_+1][y_];
                    if(fill[x_+1][y_]==false)
                    {
                        position.push(NeighborSouth);
                    }
                }
                
            }
            
            if(EWwall[cur.getx()+1][cur.gety()]==0&&x_>=0&&x_<16&&y_>=0&&y_<15)
            {
               
                if(dist[x_][y_+1]<shortest)
                {
                    shortest=dist[x_][y_+1];
                    if(fill[x_][y_+1]==false)
                        position.push(NeighborEast);
                }
            }
            
            if(EWwall[cur.getx()][cur.gety()]==0&&x_>=0&&x_<16&&y_>=1&&y_<17)
            {
                if(dist[x_][y_-1]<shortest)
                {  shortest=dist[x_][y_-1];
                    if(fill[x_][y_-1]==false)
                        position.push(NeighborWest);
                }
            }
            if(shortest==MAX)
                continue;
            if(dist[x_][y_]==shortest+1&&x_>=0&&x_<16&&y_>=0&&y_<16)
                continue;
            if(x_>=0&&x_<16&&y_>=0&&y_<16)
            {
                dist[x_][y_]=shortest+1;
                /*for(int i=0;i<MazeDefinitions::MAZE_LEN;i++)
                {
                    for(int j=0;j<MazeDefinitions::MAZE_LEN;j++)
                    {
                        cout<<dist[i][j]<<" ";
                    }
                    cout<<endl;
                }
                cout<<endl;*/

            if(NSwall[cur.getx()][cur.gety()+1]==0) // this part somehow doesn't work here
            {
                position.push(NeighborNorth);
            }
            if(NSwall[cur.getx()][cur.gety()]==0)
            {
                position.push(NeighborSouth);
            }
            if(EWwall[cur.getx()+1][cur.gety()]==0)
            {
                position.push(NeighborEast);
            }
            if(EWwall[cur.getx()][cur.gety()]==0)
            {
                position.push(NeighborWest);
            }
            }
            //then check the cell's open neighbors
            //so that we can get the shortest distance among the neighbors
            //and therefore update it
        }
    }
    
    //to determine the next movement, we follow the stpes:
    //first we update the wall map by adding wall into the data structure so then we can get back
    //second update the distance values. we do this by using floodfill
    //after we update the distance we try to move to the cell that has the shortest distance value
    
    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze) {
        const bool frontWall = maze.wallInFront();
        const bool leftWall  = maze.wallOnLeft();
        const bool rightWall = maze.wallOnRight();
        
        // Pause at each cell if the user requests it.
        // It allows for better viewing on command line.
        
        if(pause) {
            std::cout << "Hit enter to continue..." << std::endl;
            std::cin.ignore(10000, '\n');
            std::cin.clear();
        }
        
        for(int i=0;i<MazeDefinitions::MAZE_LEN;i++)
        {
            for(int j=0;j<MazeDefinitions::MAZE_LEN;j++)
            {
                cout<<dist[i][j]<<" ";
            }
            cout<<endl;
        }
        cout<<endl;
        std::cout << maze.draw(5) << std::endl << std::endl;
        
        // If we somehow miraculously hit the center
        // of the maze, just terminate and celebrate!
        if(isAtCenter(x, y)&&!leavingcenter) {
                travel++;
                reinitiation();
                
                /*for(int i=0;i<MazeDefinitions::MAZE_LEN;i++)
                {
                    for(int j=0;j<MazeDefinitions::MAZE_LEN;j++)
                    {
                        cout<<dist[i][j]<<" ";
                    }
                    cout<<endl;
                }
                cout<<endl;*/

                Location center;
                center.setx(x);
                center.sety(y);
                update(center);
                for(int i=0;i<MazeDefinitions::MAZE_LEN;i++)
                {
                    for(int j=0;j<MazeDefinitions::MAZE_LEN;j++)
                    {
                        cout<<dist[i][j]<<" ";
                    }
                    cout<<endl;
                }
                cout<<endl;
                if(currentDirection==NORTH)
                    currentDirection=SOUTH;
                else if(currentDirection==SOUTH)
                    currentDirection=NORTH;
                else if (currentDirection==EAST)
                    currentDirection=WEST;
                else if (currentDirection==WEST)
                    currentDirection=EAST;
                
                cout<<"Found center, need to get back"<<endl;
                leavingcenter=true;
                leaveorigin=false;
                return TurnAround;
               
            
    }
        
        // If we hit the start of the maze a second time, then
        // we couldn't find the center and never will...
        if(x== 0 && y== 0&&!leaveorigin) {
            if(visitedStart) {
                if(travel==0)
                {
                    std::cout << "Unable to find center, giving up." << std::endl;
                    return Finish;
                }
                else if (travel==4)
                {
                    cout<<"found center twice, game over"<<endl;
                    return Finish;
                }
                else
                {
                    travel++;
                    initiation();
                    Location origin;
                    origin.setx(x);
                    origin.sety(y);
                    update(origin);
                    cout<<"founnd center and go back"<<endl;
                    if(currentDirection==NORTH)
                        currentDirection=SOUTH;
                    else if(currentDirection==SOUTH)
                        currentDirection=NORTH;
                    else if (currentDirection==EAST)
                        currentDirection=WEST;
                    else if (currentDirection==WEST)
                        currentDirection=EAST;
                    leaveorigin=true;
                    leavingcenter=false;
                    return TurnAround;
                }
                } else {
                visitedStart = true;
            }
        }
        
        //update the wall information into a data structure/however this also depends on current direction
        
        if(currentDirection==NORTH)
        {
            if (maze.wallInFront()) {
                NSwall[x][y+1]=1;
            }
            
            if (maze.wallOnRight()) {
                
                EWwall[x+1][y]=1;
            }
            
            if (maze.wallOnLeft()) {
                EWwall[x][y]=1;
                
            }
        }
        if(currentDirection==SOUTH)
        {
            if (maze.wallInFront()) {
                NSwall[x][y]=1;
            }
            
            if (maze.wallOnRight()) {
                
                EWwall[x][y]=1;
            }
            
            if (maze.wallOnLeft()) {
                EWwall[x+1][y]=1;
                
            }
        }
        if(currentDirection==EAST)
        {
            if (maze.wallInFront()) {
                EWwall[x+1][y]=1;
            }
            if(maze.wallOnLeft())
            {
                NSwall[x][y+1]=1;
            }
            if(maze.wallOnRight())
            {
                NSwall[x][y]=1;
            }
        }
        if(currentDirection==WEST)
        {
            if(maze.wallInFront())
            {
                EWwall[x][y]=1;
            }
            if(maze.wallOnLeft())
            {
                NSwall[x][y]=1;
            }
            if(maze.wallOnRight())
            {
                NSwall[x][y+1]=1;
            }
        }
        
        
        
        Dir Tentative_dir;
        
        int shortest_distance = MAX;
        
        
        
        
        //get the distance of the neighbor, return the one that let us go to the cell which has shortest distance
        if(currentDirection==NORTH)
        {
            if(!frontWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-2][x]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-2][x];
                    Tentative_dir=NORTH;
                }
            }
            if(!leftWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-1][x-1]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x-1];
                    Tentative_dir=WEST;
                }
            }
            if(!rightWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-1][x+1]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x+1];
                    Tentative_dir=EAST;
                }
            }
        }
        if(currentDirection==SOUTH)
        {
            if(!frontWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y][x]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y][x];
                    Tentative_dir = SOUTH;
                }
            }
            if(!leftWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-1][x+1]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x+1];
                    Tentative_dir = EAST;
                }
            }
            if(!rightWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-1][x-1]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x-1];
                    Tentative_dir=WEST;
                }
            }
        }
        if(currentDirection==EAST)
        {
            if(!frontWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-1][x+1]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x+1];
                    Tentative_dir = EAST;
                }
            }
            if(!leftWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-2][x]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-2][x];
                    Tentative_dir = NORTH;
                }
            }
            if(!rightWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y][x]<shortest_distance)
                {
                    shortest_distance=dist[MazeDefinitions::MAZE_LEN-y][x];
                    Tentative_dir = SOUTH;
                }
            }
        }
        if(currentDirection==WEST)
        {
            if(!frontWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-1][x-1]<shortest_distance)
                {
                    shortest_distance = dist[MazeDefinitions::MAZE_LEN-y-1][x-1];
                    Tentative_dir = WEST;
                }
            }
            if(!leftWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y][x]<shortest_distance)
                {
                    shortest_distance = dist[MazeDefinitions::MAZE_LEN-y][x];
                    Tentative_dir = SOUTH;
                }
            }
            if(!rightWall)
            {
                if(dist[MazeDefinitions::MAZE_LEN-y-2][x]<shortest_distance)
                {
                    shortest_distance = dist[MazeDefinitions::MAZE_LEN-y-2][x];
                    Tentative_dir = NORTH;
                }
            }
        }
        
        if(shortest_distance>dist[MazeDefinitions::MAZE_LEN-y-1][x])
        {
            
            Location current;
            current.setx(x);
            current.sety(y);
            update(current);
            
            for(int i=0;i<MazeDefinitions::MAZE_LEN;i++)
            {
                for(int j=0;j<MazeDefinitions::MAZE_LEN;j++)
                {
                    cout<<dist[i][j]<<" ";
                }
                cout<<endl;
            }
            cout<<endl;
            
            if(currentDirection==NORTH)
            {
                if(!frontWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-2][x]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-2][x];
                        Tentative_dir=NORTH;
                    }
                }
                if(!leftWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-1][x-1]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x-1];
                        Tentative_dir=WEST;
                    }
                }
                if(!rightWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-1][x+1]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x+1];
                        Tentative_dir=EAST;
                    }
                }
            }
            if(currentDirection==SOUTH)
            {
                if(!frontWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y][x]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y][x];
                        Tentative_dir = SOUTH;
                    }
                }
                if(!leftWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-1][x+1]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x+1];
                        Tentative_dir = EAST;
                    }
                }
                if(!rightWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-1][x-1]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x-1];
                        Tentative_dir=WEST;
                    }
                }
            }
            if(currentDirection==EAST)
            {
                if(!frontWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-1][x+1]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-1][x+1];
                        Tentative_dir = EAST;
                    }
                }
                if(!leftWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-2][x]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y-2][x];
                        Tentative_dir = NORTH;
                    }
                }
                if(!rightWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y][x]<shortest_distance)
                    {
                        shortest_distance=dist[MazeDefinitions::MAZE_LEN-y][x];
                        Tentative_dir = SOUTH;
                    }
                }
            }
            if(currentDirection==WEST)
            {
                if(!frontWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-1][x-1]<shortest_distance)
                    {
                        shortest_distance = dist[MazeDefinitions::MAZE_LEN-y-1][x-1];
                        Tentative_dir = WEST;
                    }
                }
                if(!leftWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y][x]<shortest_distance)
                    {
                        shortest_distance = dist[MazeDefinitions::MAZE_LEN-y][x];
                        Tentative_dir = SOUTH;
                    }
                }
                if(!rightWall)
                {
                    if(dist[MazeDefinitions::MAZE_LEN-y-2][x]<shortest_distance)
                    {
                        shortest_distance = dist[MazeDefinitions::MAZE_LEN-y-2][x];
                        Tentative_dir = NORTH;
                    }
                }
            }

            
        }
        
        if(frontWall&&leftWall&&rightWall)
        {
            if(currentDirection==NORTH)
                currentDirection=SOUTH;
            else if(currentDirection==SOUTH)
                currentDirection=NORTH;
            else if (currentDirection==WEST)
                currentDirection=EAST;
            else if (currentDirection==EAST)
                currentDirection=WEST;
            return TurnAround;
        }
        switch (Tentative_dir) {
            case NORTH:
            {
                if(currentDirection==NORTH)
                    return MoveForward;
                if(currentDirection==SOUTH)
                {
                    currentDirection = NORTH;
                    return TurnAround;
                }
                if(currentDirection==EAST)
                {
                    currentDirection=NORTH;
                    return TurnCounterClockwise;
                }
                if(currentDirection==WEST)
                {
                    currentDirection=NORTH;
                    return TurnClockwise;
                }
                    
            }
            case SOUTH:
            {
                if(currentDirection==NORTH)
                {
                    currentDirection=SOUTH;
                    return TurnAround;
                }
                if(currentDirection==SOUTH)
                    return MoveForward;
                if(currentDirection==EAST)
                {
                    currentDirection=SOUTH;
                    return TurnClockwise;
                }
                if(currentDirection==WEST)
                {
                    currentDirection=SOUTH;
                    return TurnCounterClockwise;
                }
            }
            case EAST:
            {
                if(currentDirection==NORTH)
                {
                    currentDirection=EAST;
                    return TurnClockwise;
                }
                if(currentDirection==SOUTH)
                {
                    currentDirection=EAST;
                    return TurnCounterClockwise;
                }
                if(currentDirection==EAST)
                    return MoveForward;
                if(currentDirection==WEST)
                {
                    currentDirection=EAST;
                    return TurnAround;
                }
            }
            case WEST:
            {
                if(currentDirection==NORTH)
                {
                    currentDirection=WEST;
                    return TurnCounterClockwise;
                }
                if(currentDirection==SOUTH)
                {
                    currentDirection=WEST;
                    return TurnClockwise;
                }
                if(currentDirection==WEST)
                    return MoveForward;
                if(currentDirection==EAST)
                {
                    currentDirection=WEST;
                    return TurnAround;
                }
            }
            case INVALID:
            default:
                break;
        }
        
        
        // If we get stuck somehow, just terminate.
        std::cout << "Got stuck..." << std::endl;
        return Finish;
    }
    
protected:
    // Helps us determine that we should go forward if we have just turned left.
    bool shouldGoForward;
    
    // Helps us determine if we've made a loop around the maze without finding the center.
    bool visitedStart;
    
    // Indicates we should pause before moving to next cell.
    // Useful for command line usage.
    const bool pause;
    
    bool isAtCenter(unsigned x, unsigned y) const {
        unsigned midpoint = MazeDefinitions::MAZE_LEN / 2;
        
        if(MazeDefinitions::MAZE_LEN % 2 != 0) {
            return x == midpoint && y == midpoint;
        }
        
        return  (x == midpoint     && y == midpoint    ) ||
        (x == midpoint - 1 && y == midpoint    ) ||
        (x == midpoint     && y == midpoint - 1) ||
        (x == midpoint - 1 && y == midpoint - 1);
    }
    
    
    Dir currentDirection;
    //Location shortest;
    int dist[16][16];
    bool fill[16][16];
    
    int NSwall[17][17];
    int EWwall[17][17];
    int travel;
    bool leavingcenter;
    bool leaveorigin;
};



int main(int argc, char * argv[]) {
    MazeDefinitions::MazeEncodingName mazeName = MazeDefinitions::MAZE_CAMM_2012;
    bool pause = false;
    
    // Since Windows does not support getopt directly, we will
    // have to parse the command line arguments ourselves.
    
    // Skip the program name, start with argument index 1
    for(int i = 1; i < argc; i++) {
        if(strcmp(argv[i], "-m") == 0 && i+1 < argc) {
            int mazeOption = atoi(argv[++i]);
            if(mazeOption < MazeDefinitions::MAZE_NAME_MAX && mazeOption > 0) {
                mazeName = (MazeDefinitions::MazeEncodingName)mazeOption;
            }
        } else if(strcmp(argv[i], "-p") == 0) {
            pause = true;
        } else {
            std::cout << "Usage: " << argv[0] << " [-m N] [-p]" << std::endl;
            std::cout << "\t-m N will load the maze corresponding to N, or 0 if invalid N or missing option" << std::endl;
            std::cout << "\t-p will wait for a newline in between cell traversals" << std::endl;
            return -1;
        }
    }
    
    Floodfill floodFill(pause);
    Maze maze(mazeName, &floodFill);
    std::cout << maze.draw(5) << std::endl << std::endl;
    
    maze.start();
}

