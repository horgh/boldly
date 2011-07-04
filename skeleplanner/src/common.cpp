/* (c) Peter Neufeld 2011 */

const unsigned char black[] = { 0,0,0 };
const unsigned char blue[] = { 0,0,255 };
const unsigned char red[] = { 255,0,0 };
const unsigned char green[] = { 0,255,0 };
const unsigned char white[] = { 255,255,255 };
    
//iterative mapping around a point
extern const int bridgemap[8][2] = { {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0} };    
