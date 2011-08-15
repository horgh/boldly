#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <math.h>
#include <float.h>

using namespace std;

inline double dist(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

int main(int argc, char **argv)
{
    if(argc >= 2)
    {
        int minrange = 0;
        int maxrange = 10;
        if(argc >= 3)
            minrange = atoi(argv[2]);
        if(argc >= 4)
            maxrange = atoi(argv[3]);
    
        for(int i = minrange; i <= maxrange; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                FILE * file;
                stringstream filename;
                filename << argv[1] << "/" << i << "-" << j << "-" << argv[1];
                file = fopen(filename.str().c_str(), "r");
                
                if(file != NULL)
                {
                    FILE * output;
                    stringstream outname;
                    outname << filename.str() << ".processed";
                    output = fopen(outname.str().c_str(), "w");
                    
                    double travelled = 0.0;
                    double lastx = DBL_MAX;
                    double lasty = DBL_MAX;
                    double time;
                    double maxval;
                    
                    while(!feof(file))
                    {
                        if(lastx == DBL_MAX)
                        {
                            fscanf(file, "%lf %lf %lf %lf", &time, &lastx, &lasty, &maxval);
                            
                        }else{
                            double newx = 0.0;
                            double newy = 0.0;
                            
                            fscanf(file, "%lf %lf %lf %lf", &time, &newx, &newy, &maxval);
                            travelled += dist(lastx, lasty, newx, newy);
                            lastx = newx;
                            lasty = newy;
                        }
                                        
                        fprintf(output, "%lf %lf\n", travelled, maxval);
                    }
                    
                    fclose(output);
                    fclose(file);
                    
                }else{
                    printf("Error: could not open file %s\n", filename.str().c_str());
                    return 2;
                }
            }
        }
        
        return 0;
    }else
        printf("Usage: compute <mapname> [min_range] [max_range]\n");
    
    return 1;
}
