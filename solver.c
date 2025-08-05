#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "api.h"

#define SIZE 16
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

int dx[4] = {0, 1, 0, -1};
int dy[4] = {1, 0, -1, 0};

uint8_t walls[SIZE][SIZE][4] = {0};
uint8_t flood[SIZE][SIZE];
uint8_t optimized_path[SIZE][SIZE] = {0};

int curX = 0, curY = 0;
int direction = NORTH;
int last_optimized_steps = 0;
int current_run_steps = 0;
bool optimization_complete = false;

bool isValid(int x, int y) {
    return (x >= 0 && x < SIZE && y >= 0 && y < SIZE);
}

void updateWalls() {
    // Set walls for current cell based on sensor readings
    walls[curX][curY][direction] = API_wallFront();
    walls[curX][curY][(direction + 1) % 4] = API_wallRight();
    walls[curX][curY][(direction + 3) % 4] = API_wallLeft();
    
    // Update opposite walls for neighbors
    for (int i = 0; i < 4; i++) {
        int nx = curX + dx[i];
        int ny = curY + dy[i];
        if (isValid(nx, ny)) {
            int opp = (i + 2) % 4;
            if (walls[curX][curY][i]) {
                walls[nx][ny][opp] = 1;
            }
        }
    }
}

void floodFill(bool to_center) {
    // Initialize all cells with high value
    for (int x = 0; x < SIZE; x++) {
        for (int y = 0; y < SIZE; y++) {
            flood[x][y] = 255;
        }
    }
    
    // Create a queue for BFS
    int queue[SIZE*SIZE][2];
    int front = 0, rear = 0;
    
    if (to_center) {
        // Set center goals to 0
        int centers[4][2] = {{7,7}, {7,8}, {8,7}, {8,8}};
        for (int i = 0; i < 4; i++) {
            int gx = centers[i][0];
            int gy = centers[i][1];
            if (isValid(gx, gy)) {
                flood[gx][gy] = 0;
                queue[rear][0] = gx;
                queue[rear][1] = gy;
                rear++;
            }
        }
    } else {
        // Set start (0,0) to 0
        flood[0][0] = 0;
        queue[rear][0] = 0;
        queue[rear][1] = 0;
        rear++;
    }
    
    while (front < rear) {
        int x = queue[front][0];
        int y = queue[front][1];
        front++;
        
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d];
            int ny = y + dy[d];
            
            // Skip invalid cells or walls blocking the path
            if (!isValid(nx, ny) || walls[x][y][d]) 
                continue;
            
            // Calculate new potential flood value
            uint8_t newVal = flood[x][y] + 1;
            
            // Update neighbor if we found a shorter path
            if (newVal < flood[nx][ny]) {
                flood[nx][ny] = newVal;
                queue[rear][0] = nx;
                queue[rear][1] = ny;
                rear++;
            }
        }
    }
}

void displayFloodValues() {
    for (int x = 0; x < SIZE; x++) {
        for (int y = 0; y < SIZE; y++) {
            if (flood[x][y] < 255) {
                char text[4];
                snprintf(text, sizeof(text), "%d", flood[x][y]);
                API_setText(x, y, text);
            }
        }
    }
}

void turnTo(int newDir) {
    int diff = (newDir - direction + 4) % 4;
    if (diff == 1) {
        API_turnRight();
    } else if (diff == 2) {
        API_turnRight();
        API_turnRight();
    } else if (diff == 3) {
        API_turnLeft();
    }
    direction = newDir;
}

void moveForward() {
    if (API_moveForward()) {
        curX += dx[direction];
        curY += dy[direction];
        
        // Mark visited cell as green
        API_setColor(curX, curY, 'G');
        
        // Update flood value display
        char text[4];
        snprintf(text, sizeof(text), "%d", flood[curX][curY]);
        API_setText(curX, curY, text);
        
        // Count steps for optimization comparison
        current_run_steps++;
    }
}

bool isCenter() {
    return (curX >= 7 && curX <= 8 && curY >= 7 && curY <= 8);
}

bool isStart() {
    return (curX == 0 && curY == 0);
}
void moveToCenter() {
    floodFill(true);  // Flood fill to center
    displayFloodValues();
    
    while (!isCenter()) {
        updateWalls();
        floodFill(true);  // Recalculate with new walls
        displayFloodValues();
        
        // Find best direction to move
        int nextDir = -1;
        int minVal = flood[curX][curY];
        
        for (int d = 0; d < 4; d++) {
            int nx = curX + dx[d];
            int ny = curY + dy[d];
            
            if (!isValid(nx, ny) || walls[curX][curY][d]) 
                continue;
                
            if (flood[nx][ny] < minVal) {
                minVal = flood[nx][ny];
                nextDir = d;
            }
        }
        
        // If no better value, pick any accessible direction
        if (nextDir == -1) {
            for (int d = 0; d < 4; d++) {
                if (!walls[curX][curY][d]) {
                    nextDir = d;
                    break;
                }
            }
        }
        
        // Move if we found a valid direction
        if (nextDir != -1) {
            turnTo(nextDir);
            moveForward();
        } else {
            // Shouldn't happen in a valid maze
            break;
        }
    }
}

void moveToStart() {
    floodFill(false);  // Flood fill to start (0,0)
    displayFloodValues();
    
    while (!isStart()) {
        updateWalls();
        floodFill(false);  // Recalculate with new walls
        displayFloodValues();
        
        // Find best direction to move
        int nextDir = -1;
        int minVal = flood[curX][curY];
        
        for (int d = 0; d < 4; d++) {
            int nx = curX + dx[d];
            int ny = curY + dy[d];
            
            if (!isValid(nx, ny) || walls[curX][curY][d]) 
                continue;
                
            if (flood[nx][ny] < minVal) {
                minVal = flood[nx][ny];
                nextDir = d;
            }
        }
        
        if (nextDir == -1) {
            for (int d = 0; d < 4; d++) {
                if (!walls[curX][curY][d]) {
                    nextDir = d;
                    break;
                }
            }
        }
        
        if (nextDir != -1) {
            turnTo(nextDir);
            moveForward();
        } else {
            break;
        }
    }
}

void solver() {
    // Initial run to center
    moveToCenter();
    last_optimized_steps = current_run_steps;
    current_run_steps = 0;
    
    // Optimization loop
    while (!optimization_complete) {
        // Return to start
        moveToStart();
        
        // Optimized run to center
        current_run_steps = 0;
        moveToCenter();
        
        // Check if we found a better path
        if (current_run_steps >= last_optimized_steps) {
            optimization_complete = true;
            API_setText(0, 0, "OPTIMAL!");
        } else {
            last_optimized_steps = current_run_steps;
            char msg[20];
            snprintf(msg, sizeof(msg), "BETTER: %d", current_run_steps);
            API_setText(0, 0, msg);
        }
    }
    
    // Final message
    char msg[20];
    snprintf(msg, sizeof(msg), "BEST: %d STEPS", last_optimized_steps);
    API_setText(0, 0, msg);
}
